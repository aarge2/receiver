/*
 * display_control.c
 *
 *  Created on: 23 Aðu 2022
 *      Author: Power Rangers
 */
#include "display_control.h"
#include "esp_timer.h"
#include "display_4x7_segment.h"
#include <stdio.h>


char ac_display[10];
static esp_timer_handle_t refresh_display_timer;
volatile uint32_t dw_refresh_tick;

void callback_refresh_display(void* arg);

void start_display_refresh(void)
{
	 const esp_timer_create_args_t refresh_display_args =
	 {
			.callback = &callback_refresh_display,
	 };

	 sprintf(ac_display, "%.4d", 0);
	 display_set_rgb(0, LED_WHITE);

	 display_start_it(0, (const uint8_t *)ac_display, 4);

	 esp_timer_create(&refresh_display_args, &refresh_display_timer);
	 esp_timer_start_periodic(refresh_display_timer, 3000);
}

void stop_display_refresh(void)
{
	esp_timer_stop(refresh_display_timer);
}

void callback_refresh_display(void* arg)
{
	display_print_isr(0);
	++dw_refresh_tick;
}

