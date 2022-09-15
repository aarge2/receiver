#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "coeffs.h"
#include "display_4x7_segment.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "display_control.h"
#include "lora.h"
#include "receiver_id.h"
#include "soft_timer.h"
#include "edge_detection.h"
#include "board_modbus_slave.h"
#include "esp_modbus_slave.h"
#include "checksum.h"
#include "circularbuffer.h"
#include "macro.h"
#include <string.h>
#include "board_type.h"
//#include "esp_task_wdt.h"

// Error and Alarm Messages
const char *err_modbus = "Err1";
const char *err_lora = "Err2";

// lora vars
lora_t radio;
uint16_t lora_msg_size = 0;
uint8_t ac_lora_rx_buff[k_LORA_DATA_SIZE];

// pedometer vars
pedometer_data_t s_ped_cb_buff[k_CB_BUFFER_SIZE];
CircularBufferContext cb;

// rtos vars
SemaphoreHandle_t s_smph_data_ready_lora = NULL;
TimerHandle_t timer_display;
TimerHandle_t timer_startup;
TimerHandle_t timer_1sec;

// general device scenario vars
uint8_t  b_startup_seq_ctrl = 0;
bits_t	s_error_bits;
bits_t	s_state_bits;
uint32_t modbus_err_cnt = 0;
uint32_t lora_err_cnt = 0;

// function prototypes
void lora_init_sequence(void);
void gpio_init_sequence(void);
void startup_sequence(void);
void create_timers(void);
void IRAM_ATTR isr_lora(void *arg);
void lora_data_ready_task(void *arg);
void modbus_task(void *arg);
void alarm_supervisor_task(void *arg);
void callback_timer_1sec(void *arg);
void callback_timer_display(TimerHandle_t xTimer);





// MAIN
void app_main(void)
{
	s_smph_data_ready_lora = xSemaphoreCreateBinary();
//	cb_init(CB_ID_0, s_ped_cb_buff, CB_BUFFER_SIZE, sizeof(s_ped_cb_buff));
	CircularBufferInit(&cb, s_ped_cb_buff, sizeof(s_ped_cb_buff), sizeof(s_ped_cb_buff[0]));
	gpio_init_sequence();
	dw_rec_id = (uint32_t)get_receiver_hardware_id() + 1; // real modbus address
	spi_display_init();
	modbus_slave_init();
	dw_rec_id += 329; // to show on display
	xTaskCreatePinnedToCore(modbus_task, "modbus_task", 4096, NULL, 9, NULL, 0);
	start_display_refresh();
	display_set_rgb(0, LED_OFF);
	display_on(0);

	create_timers();
	startup_sequence();

	spi_lora_init();
	lora_init_sequence();
	xTaskCreatePinnedToCore(lora_data_ready_task, "lora_data_ready_task", 4096, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(alarm_supervisor_task, "alarm_supervisor_task", 4096, NULL, 8, NULL, 0);

	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(3000));
	}
}






/************* FUNCTION DECLERATIONS *************/
void lora_init_sequence(void)
{
	radio.Modulation = LORA;
	radio.COB = RFM98;
	radio.Frequency = 434000;
	radio.OutputPower = 17; //dBm
	radio.PreambleLength = 12;
	radio.FixedPktLength = false; //explicit header mode for LoRa
	radio.PayloadLength = k_LORA_DATA_SIZE;
	radio.CrcDisable = true;
	radio.SFSel = SF9;
	radio.BWSel = BW125K;
	radio.CRSel = CR4_5;
	lora_init(&radio);
	lora_rx_mode(&radio);
}

void gpio_init_sequence(void)
{
	gpio_reset_pin(R_CLK);
	gpio_set_direction(R_CLK, GPIO_MODE_OUTPUT);
	gpio_reset_pin(CS_LORA);
	gpio_set_direction(CS_LORA, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(DIO0_PIN);
	gpio_set_direction(DIO0_PIN, GPIO_MODE_INPUT);
	gpio_pad_select_gpio(ID1_PIN);
	gpio_set_direction(ID1_PIN, GPIO_MODE_INPUT);
	gpio_pad_select_gpio(ID2_PIN);
	gpio_set_direction(ID2_PIN, GPIO_MODE_INPUT);
	gpio_pad_select_gpio(ID3_PIN);
	gpio_set_direction(ID3_PIN, GPIO_MODE_INPUT);
	gpio_pad_select_gpio(ID4_PIN);
	gpio_set_direction(ID4_PIN, GPIO_MODE_INPUT);

	gpio_pad_select_gpio(LORA_DIO0);
	gpio_set_direction(LORA_DIO0, GPIO_MODE_INPUT);
	gpio_set_intr_type(LORA_DIO0, GPIO_INTR_POSEDGE);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(LORA_DIO0, isr_lora, NULL);
}

void startup_sequence(void)
{
	sprintf(ac_display, "UEr1");
	vTaskDelay(pdMS_TO_TICKS(3000));
	sprintf(ac_display, "U100");
	vTaskDelay(pdMS_TO_TICKS(3000));
	sprintf(ac_display, "C001");
	vTaskDelay(pdMS_TO_TICKS(3000));
	sprintf(ac_display, "%.4d", dw_rec_id);
}

void IRAM_ATTR isr_lora(void *arg)
{
	xSemaphoreGiveFromISR(s_smph_data_ready_lora, NULL);
}

void lora_data_ready_task(void *arg)
{
	pedometer_data_t s_ped_data;
	uint16_t w_crc;
	bool o_crc_ok = false;

	for (;;)
	{
		if ((xSemaphoreTake(s_smph_data_ready_lora, portMAX_DELAY) == pdTRUE))
		{
			lora_err_cnt = 0;
			lora_msg_size = lora_get_msg(&radio, ac_lora_rx_buff);
			s_ped_data.w_crc = (uint32_t)ac_lora_rx_buff[13] | (ac_lora_rx_buff[14] << 8);
			w_crc = crc_16(ac_lora_rx_buff + 1, 12);
			o_crc_ok = (s_ped_data.w_crc == w_crc);

//			const char * ac_crc = (o_crc_ok == s_ped_data.w_crc) ? "OK" : "NOK";

			if(o_crc_ok && (lora_msg_size == k_LORA_DATA_SIZE) && (ac_lora_rx_buff[0] == k_SOH) && (ac_lora_rx_buff[k_LORA_DATA_SIZE - 1] == k_EOT))
			{
				s_ped_data.dw_id = (uint32_t)ac_lora_rx_buff[1] | (ac_lora_rx_buff[2] << 8) | (ac_lora_rx_buff[3] << 16) | (ac_lora_rx_buff[4] << 24);
				s_ped_data.dw_ref = (uint32_t)ac_lora_rx_buff[5] | (ac_lora_rx_buff[6] << 8) | (ac_lora_rx_buff[7] << 16) | (ac_lora_rx_buff[8] << 24);
				s_ped_data.w_steps = (uint32_t)ac_lora_rx_buff[9] | (ac_lora_rx_buff[10] << 8);
				s_ped_data.b_time_of_rest = (uint32_t)ac_lora_rx_buff[11];
				s_ped_data.b_rest_stand_cnt = (uint32_t)ac_lora_rx_buff[12];


				int32_t l_buff_state = CircularBufferPushBack(&cb, (void*)&s_ped_data);

				if(!s_error_bits.bit0 && !l_buff_state)
				{
					if(!xTimerIsTimerActive(timer_display)) {xTimerStart(timer_display, 0);}
					char ab_tmp[15] = {0};
					sprintf(ab_tmp, "%d", s_ped_data.dw_id);
					memcpy(ac_display, ab_tmp + 4, 4);
					display_set_rgb(0, LED_GREEN);
				}
				else if(l_buff_state == -1)
				{
					if(!xTimerIsTimerActive(timer_display)) {xTimerStart(timer_display, 0);}
					sprintf(ac_display, "8UFF");
					display_set_rgb(0, LED_YELLOW);
				}
			}
			else
			{
				if(!xTimerIsTimerActive(timer_display)) {xTimerStart(timer_display, 0);}
				sprintf(ac_display, "crc0");
				display_set_rgb(0, LED_YELLOW);
			}
		}
	}
}

void modbus_task(void *arg)
{
	pedometer_data_t s_ped_data_send;
	CLEAR_STRUCT(s_ped_data_send);
	TickType_t 	xLastWakeTime;
	TickType_t	wait =  pdMS_TO_TICKS(100);
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, wait);

		if(modbus_check_read_event_blocking()) // wait until read event occurred by Master
		{
			modbus_err_cnt = 0;
			if(!CircularBufferPopFront(&cb, &s_ped_data_send))
			{
				holding_regs[0] = s_ped_data_send.dw_id;
				holding_regs[1] = s_ped_data_send.dw_ref;
				holding_regs[2] = s_ped_data_send.w_steps;
				holding_regs[3] = s_ped_data_send.b_time_of_rest;
				holding_regs[4] = s_ped_data_send.b_rest_stand_cnt;
				holding_regs[5] = dw_rec_id;
				CLEAR_STRUCT(s_ped_data_send);
			}
			else
			{
				memset(holding_regs, 0, sizeof(holding_regs));
			}
		}
//		if(modbus_check_write_event_blocking()) // wait until write event occurred which master write the data
//		{
//			modbus_handle_data_for_write_event();
//		}
//		esp_task_wdt_reset();
	}
}

void alarm_supervisor_task(void *arg)
{
	TickType_t 	xLastWakeTime;
	TickType_t	wait =  pdMS_TO_TICKS(500);
	xLastWakeTime = xTaskGetTickCount();
	static uint32_t local_tick = 0;
	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, wait);
		++local_tick;
//		s_error_bits.bit0 = TON(TON_ID_0, s_state_bits.bit0, local_tick, k_MODBUS_ERR_TIMEOUT);

		// modbus err detection BEGIN
		if(++modbus_err_cnt >= k_MODBUS_ERR_TIMEOUT) {s_error_bits.bit0 = true;}
		else {s_error_bits.bit0 = false;}

		if(s_error_bits.bit0 && !s_error_bits.bit1)
		{
			sprintf(ac_display, "%s", err_modbus);
			display_set_rgb(0, LED_RED);
			if(!xTimerIsTimerActive(timer_1sec)) {xTimerStart(timer_1sec, 0);}
		}

		if(edge_detection(ED_ID_0, !s_error_bits.bit0))
		{
			sprintf(ac_display, "%.4d", dw_rec_id);
			display_set_rgb(0, LED_OFF);
		}
		// modbus err detection END

		// lora err detection BEGIN
		if(++lora_err_cnt >= k_LORA_ERR_TIMEOUT) {s_error_bits.bit1 = true;}
		else {s_error_bits.bit1 = false;}

		if(s_error_bits.bit1 || (s_error_bits.bit0 && s_error_bits.bit1)) // lora error is privileged
		{
			sprintf(ac_display, "%s", err_lora);
			display_set_rgb(0, LED_RED);
			if(!xTimerIsTimerActive(timer_1sec)) {xTimerStart(timer_1sec, 0);}
		}

		if(edge_detection(ED_ID_1, !s_error_bits.bit1))
		{
			sprintf(ac_display, "%.4d", dw_rec_id);
			display_set_rgb(0, LED_OFF);
		}
		// lora err detection END
	}
}

void callback_timer_1sec(void *arg)
{
	display_set_rgb(0, LED_OFF);
}

void callback_timer_display(TimerHandle_t xTimer)
{
	sprintf(ac_display, "%.4d", dw_rec_id);
	display_set_rgb(0, LED_OFF);
}

void create_timers(void)
{
	timer_display = xTimerCreate
	                   ("timer_display",
	                     pdMS_TO_TICKS(2000),
	                     pdFALSE, // one-shot
	                     (void *)0,
						 callback_timer_display);

	timer_1sec = xTimerCreate
	                   ("timer_1sec",
	                     pdMS_TO_TICKS(1000),
	                     pdFALSE, // one-shot
	                     (void *)0,
						 callback_timer_1sec);
}
