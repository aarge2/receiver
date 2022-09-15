/*
 * display_4x7_segment.c
 *
 *  Created on: Aug 1, 2022
 *      Author: DeLL
 */

#include "display_4x7_segment.h"
#include "ascii_table.h"
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/task.h"

#define CS_DISPLAY_H()	gpio_set_level(26, true)
#define CS_DISPLAY_L()	gpio_set_level(26, false)
spi_device_handle_t spi2;

static void spi_transmit(uint8_t *data);

typedef struct
{
	int8_t b_dp_cnt;
	bool o_is_dp;
	uint8_t i;
	uint8_t ab_digit_ascii[2];
	const uint8_t *ab_buff;
	uint8_t b_size;
	uint8_t rgb;
	uint8_t off;
}display_4x7_t;

display_4x7_t s_4x7[DISPLAY_4X7_OBJ_SIZE];

void display_start_it(uint8_t b_id, const uint8_t *ab_buff, uint8_t b_size)
{
	s_4x7[b_id].ab_buff = ab_buff;
	s_4x7[b_id].i = 0;
	s_4x7[b_id].o_is_dp = false;
	s_4x7[b_id].b_dp_cnt = 0;
	s_4x7[b_id].b_size = b_size;
}

void display_print_isr(uint8_t b_id)
{

	uint8_t j = s_4x7[b_id].b_size - s_4x7[b_id].i - 1;

	if (s_4x7[b_id].ab_buff[j] == '.')
	{
		++s_4x7[b_id].b_dp_cnt;
		s_4x7[b_id].ab_digit_ascii[0] = ab_digit_pos[s_4x7[b_id].b_dp_cnt - 1];
		s_4x7[b_id].ab_digit_ascii[1] = ab_display_ascii_table[s_4x7[b_id].ab_buff[j]];
		s_4x7[b_id].o_is_dp = true;
	}
	else
	{
		if (s_4x7[b_id].o_is_dp)
		{
			s_4x7[b_id].o_is_dp = false;
			s_4x7[b_id].ab_digit_ascii[0] = ab_digit_pos[s_4x7[b_id].b_dp_cnt - 1];
			s_4x7[b_id].ab_digit_ascii[1] = ab_display_ascii_table[s_4x7[b_id].ab_buff[j]];
		}
		else
		{
			++s_4x7[b_id].b_dp_cnt;
			s_4x7[b_id].ab_digit_ascii[0] = ab_digit_pos[s_4x7[b_id].b_dp_cnt - 1];
			s_4x7[b_id].ab_digit_ascii[1] = ab_display_ascii_table[s_4x7[b_id].ab_buff[j]];
		}
	}

	s_4x7[b_id].ab_digit_ascii[0] |= s_4x7[b_id].rgb;
	s_4x7[b_id].ab_digit_ascii[0] &= s_4x7[b_id].off;

	if (s_4x7[b_id].i < s_4x7[b_id].b_size)
	{
		++s_4x7[b_id].i;
		CS_DISPLAY_L();
		spi_transmit(s_4x7[b_id].ab_digit_ascii);
		CS_DISPLAY_H();
	}

	if(s_4x7[b_id].i >= s_4x7[b_id].b_size)
	{
		s_4x7[b_id].i = 0;
		s_4x7[b_id].o_is_dp = false;
		s_4x7[b_id].b_dp_cnt = 0;
	}
}

void display_set_rgb(uint8_t b_id, uint8_t rgb)
{
	s_4x7[b_id].rgb = rgb;
}

void display_off(uint8_t b_id)
{
	s_4x7[b_id].off = 0xF0;
}

void display_on(uint8_t b_id)
{
	s_4x7[b_id].off = 0xFF;
}


void display_print(const uint8_t *buff, uint8_t size)
{
	uint8_t j = 0;
	int8_t b_dp_cnt = 0;
	bool o_is_dp = false;
	uint8_t b_digit_pos = 0;
	uint8_t b_ascii = 0;
	uint8_t ab_data[2];

	for (uint8_t i = 0; i < size; ++i)
	{
		j = size - i - 1;

		if (buff[j] == '.')
		{
			++b_dp_cnt;
			b_digit_pos = ab_digit_pos[b_dp_cnt - 1];
			b_ascii = ab_display_ascii_table[buff[j]];
			o_is_dp = true;
		}
		else
		{
			if (o_is_dp)
			{
				o_is_dp = false;
				b_digit_pos = ab_digit_pos[b_dp_cnt - 1];
				b_ascii = ab_display_ascii_table[buff[j]];
			}
			else
			{
				++b_dp_cnt;
				b_digit_pos = ab_digit_pos[b_dp_cnt - 1];
				b_ascii = ab_display_ascii_table[buff[j]];
			}
		}

		ab_data[0] = b_digit_pos;
		ab_data[1] = b_ascii;
		CS_DISPLAY_L();
		spi_transmit(ab_data);
		CS_DISPLAY_H();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

void spi_display_init(void)
{
	spi_bus_config_t buscfg =
	{
			.miso_io_num = -1,
			.mosi_io_num = 27,
			.sclk_io_num = 25,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.max_transfer_sz = 0,
	};

	spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);

	spi_device_interface_config_t devcfg =
	{
			.clock_speed_hz = 100000,
			.mode = 0,                  //SPI mode 0
			.spics_io_num = -1,
			.queue_size = 1,
			.flags = SPI_DEVICE_HALFDUPLEX,
			.pre_cb = NULL,
			.post_cb = NULL,
	};
	spi_bus_add_device(SPI2_HOST, &devcfg, &spi2);
}

static void spi_transmit(uint8_t *ab_data)
{
    spi_transaction_t t =
    {
       .flags = 0,
       .length = 16,
       .tx_buffer = ab_data,
       .rx_buffer = NULL,
    };

    CS_DISPLAY_L();
    spi_device_transmit(spi2, &t);
    CS_DISPLAY_H();
}
