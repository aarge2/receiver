/*
 * receiver_id.c
 *
 *  Created on: 25 Aðu 2022
 *      Author: sevimli tavsan momo
 */

#include "receiver_id.h"
#include "driver/gpio.h"
#include "macro.h"

uint16_t get_receiver_hardware_id(void)
{
	uint16_t w_ret = 0;

	for(uint8_t i = 32; i < 36; ++i)
	{
		if(gpio_get_level(i))
		{
			SET_BIT_POS(w_ret, i);
		}
		else
		{
			RESET_BIT_POS(w_ret, i);
		}
	}
	return w_ret;
}



