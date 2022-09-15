/*
 * display_control.h
 *
 *  Created on: 23 Aðu 2022
 *      Author: Power Rangers
 */

#ifndef MAIN_DISPLAY_CONTROL_H_
#define MAIN_DISPLAY_CONTROL_H_

#include <stdint.h>

extern char ac_display[10];
extern volatile uint32_t dw_refresh_tick;

void start_display_refresh(void);


#endif /* MAIN_DISPLAY_CONTROL_H_ */
