/*
 * board_modbus_slave.c
 *
 *  Created on: 25 Aðu 2022
 *      Author: coccomelon
 */

#include "board_modbus_slave.h"
#include "driver/uart.h"
#include "esp_modbus_slave.h"

#define MB_PORT_NUM     2
#define MB_RTS			18
#define MB_RX			22
#define MB_TX			23
#define MB_DEV_SPEED    9600

#define MB_READ_MASK                        MB_EVENT_HOLDING_REG_RD
#define MB_WRITE_MASK                       MB_EVENT_HOLDING_REG_WR
#define MB_READ_WRITE_MASK                  (MB_READ_MASK | MB_WRITE_MASK)

#define MB_PAR_INFO_GET_TOUT                (20) // Timeout for get parameter info

mb_param_info_t reg_info; // keeps the Modbus registers access information
mb_communication_info_t comm_info; // Modbus communication parameters
mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure

uint32_t holding_regs[HOLDING_REG_SIZE];

static const char *TAG = "SLAVE_TEST:";

uint32_t dw_rec_id = 0;

void modbus_slave_init(void)
{
	// Set UART log level
	esp_log_level_set(TAG, ESP_LOG_INFO);
	void *mbc_slave_handler = NULL;

	ESP_ERROR_CHECK(mbc_slave_init(MB_PORT_SERIAL_SLAVE, &mbc_slave_handler)); // Initialization of Modbus controller

	comm_info.mode = MB_MODE_RTU;
	comm_info.slave_addr = dw_rec_id;
	comm_info.port = MB_PORT_NUM;
	comm_info.baudrate = MB_DEV_SPEED;
	comm_info.parity = MB_PARITY_NONE;
	ESP_ERROR_CHECK(mbc_slave_setup((void* )&comm_info));

	reg_area.type = MB_PARAM_HOLDING;
	reg_area.start_offset = 0;
	reg_area.address = (void*) &holding_regs;
	reg_area.size = HOLDING_REG_SIZE * sizeof(uint32_t);
	ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

	ESP_ERROR_CHECK(mbc_slave_start());

	ESP_ERROR_CHECK(
			uart_set_pin(MB_PORT_NUM, MB_TX, MB_RX, MB_RTS, UART_PIN_NO_CHANGE));
	ESP_ERROR_CHECK(uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX));
	ESP_LOGI(TAG, "Modbus slave stack initialized.");
}

/**
 * @fn bool modbus_check_read_write_event_blocking(void)
 * @brief
 * @return 1 is read
 */
bool modbus_check_read_event_blocking(void)
{
	mb_event_group_t event = mbc_slave_check_event(MB_EVENT_HOLDING_REG_RD);
	return (bool) (event & MB_EVENT_HOLDING_REG_RD);
}

/**
 * @fn bool modbus_check_write_event_blocking(void)
 * @brief
 * @return 1 is write
 */
bool modbus_check_write_event_blocking(void)
{
	mb_event_group_t event = mbc_slave_check_event(MB_EVENT_HOLDING_REG_WR);
	return (bool) (event & MB_EVENT_HOLDING_REG_WR);
}

void modbus_handle_data_for_read_event(void)
{
//	for (uint8_t i = 0; i < 2; ++i)
//	{
//		if(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT) == ESP_OK)
//		{
//			if (reg_info.mb_offset == 0)
//			{
//				printf("holding_regs[0](%d)\n", holding_regs[0]);
//				printf("holding_regs[1](%d)\n", holding_regs[1]);
//			}
//		}
//
//		ESP_LOGI(
//				TAG,
//				"HOLDING %s (%u us), ADDR:%u, TYPE:%u, INST_ADDR:0x%x, SIZE:%u ",
//				"READ",
//				(uint32_t )reg_info.time_stamp,
//				(uint32_t )reg_info.mb_offset,
//				(uint32_t )reg_info.type,
//				(uint32_t )reg_info.address,
//				(uint32_t )reg_info.size);
//	}

}

void modbus_handle_data_for_write_event(void)
{
//	for (uint8_t i = 0; i < 2; ++i)
//	{
//		ESP_ERROR_CHECK(
//				mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
//
//		ESP_LOGI(
//				TAG,
//				"HOLDING %s (%u us), ADDR:%u, TYPE:%u, INST_ADDR:0x%x, SIZE:%u ",
//				"WRITE",
//				(uint32_t )reg_info.time_stamp,
//				(uint32_t )reg_info.mb_offset,
//				(uint32_t )reg_info.type,
//				(uint32_t )reg_info.address,
//				(uint32_t )reg_info.size);

//		if (reg_info.mb_offset == 2/*(uint8_t*)&holding_regs[1]*/)
//		{
//			printf("holding_regs[2](%d)\n", holding_regs[2]);
//			printf("holding_regs[3](%d)\n", holding_regs[3]);
//			++holding_regs[0];
//			holding_regs[1]+=3;
//		}
//		if (reg_info.mb_offset == 0)
//		{
//			printf("param0(%d)\n", holding_regs[0]);
//		}
//	}
}

