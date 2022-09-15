/*
 * board_modbus.h
 *
 *  Created on: 25 Aðu 2022
 *      Author: coccomelon team
 */

#ifndef BOARD_MODBUS_SLAVE_H
#define BOARD_MODBUS_SLAVE_H

#include <stdint.h>
#include <stdbool.h>

#define HOLDING_REG_SIZE	10

extern uint32_t holding_regs[HOLDING_REG_SIZE];
extern uint32_t dw_rec_id;

void modbus_slave_init(void);
bool modbus_check_read_event_blocking(void);
bool modbus_check_write_event_blocking(void);
void modbus_handle_data_for_read_event(void);
void modbus_handle_data_for_write_event(void);


#endif /* BOARD_MODBUS_SLAVE_H */
