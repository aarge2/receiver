/*
 * display_4x7_segment.h
 *
 *  Created on: Aug 1, 2022
 *      Author: DeLL
 */

#ifndef DISPLAY_4X7_SEGMENT_H
#define DISPLAY_4X7_SEGMENT_H

#include "stdint.h"

#define k_MAX_CHAR_NUM	8

#define LED_OFF		0x70
#define LED_BLUE	0x60
#define LED_GREEN	0x50
#define LED_RED		0x30
#define LED_WHITE	LED_BLUE & LED_GREEN & LED_RED
#define LED_YELLOW	LED_GREEN & LED_RED

//#define LED_BLUE	0x6F
//#define LED_GREEN	0x5F
//#define LED_RED	0x3F
//#define LED_WHITE	LED_BLUE & LED_GREEN & LED_RED



enum
{
	DISPLAY_0 = 0,
	DISPLAY_END,
};

#define DISPLAY_4X7_OBJ_SIZE	DISPLAY_END

void display_start_it(uint8_t b_id, const uint8_t *ab_buff, uint8_t b_size);
void display_print_isr(uint8_t b_id);
void display_set_rgb(uint8_t b_id, uint8_t rgb);
void display_off(uint8_t b_id);
void display_on(uint8_t b_id);
void display_print(const uint8_t *buff, uint8_t size);

void spi_display_init(void);


#endif /* DISPLAY_4X7_SEGMENT_H */
