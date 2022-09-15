/*
 * board_type.h
 *
 *  Created on: 6 Eyl 2022
 *      Author: DeLL
 */

#ifndef BOARD_TYPE_H
#define BOARD_TYPE_H_


typedef struct
{
	uint32_t bit0:1U;
	uint32_t bit1:1U;
	uint32_t bit2:1U;
	uint32_t bit3:1U;
	uint32_t bit4:1U;
	uint32_t bit5:1U;
	uint32_t bit6:1U;
	uint32_t bit8:1U;
}bits_t;

typedef struct
{
	// SOH 1
	uint32_t dw_id; // 2345
	uint32_t dw_ref; // 6789
	uint16_t w_steps; // 10 11
	uint16_t w_crc;//12 13
	uint8_t b_time_of_rest; // 14
	uint8_t b_rest_stand_cnt; // 15
	//EOT // 16 -> total byte size = 16
}pedometer_data_t;


#endif /* BOARD_TYPE_H */
