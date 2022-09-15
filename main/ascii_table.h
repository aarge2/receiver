/*
 * ascii_table.h
 *
 *  Created on: Aug 2, 2022
 *      Author: DeLL
 */

#ifndef ASCII_TABLE_H_
#define ASCII_TABLE_H_

uint8_t ab_display_ascii_table[] =
{
	[0] = 0x00,
	[45] = 0xBF, //'-'
	[46] = 0x7F, //'.'
	[48] = 0xC0, //'0'
	[49] = 0xF9, //'1'
	[50] = 0xA4, //'2'
	[51] = 0xB0, //'3'
	[52] = 0x99, //'4'
	[53] = 0x92, //'5'
	[54] = 0x82, //'6'
	[55] = 0xF8, //'7'
	[56] = 0x80, //'8'
	[57] = 0x90, //'9'
	[65] = 0x88, //'A'
	[67] = 0xC6, //'C'
	[69] = 0x86, //'E'
	[70] = 0x8E, //'F'
	[72] = 0x89, //'H'
	[76] = 0xC7, //'L'
	[80] = 0x8C, //'P'
	[85] = 0xC1, //'U'
	[95] = 0xF7, //'_'
	[99] = 0xA7, //'c'
	[114] = 0xAF,// 'r'
};

uint8_t ab_digit_pos[] = {0x08, 0x04, 0x02, 0x01}; // regular situation
//uint8_t ab_digit_pos[] = {0x78, 0x74, 0x72, 0x71}; // For RGB control


#endif /* ASCII_TABLE_H_ */
