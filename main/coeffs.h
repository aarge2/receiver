/*
 * coeffs.h
 *
 *  Created on: 22 Aðu 2022
 *      Author: Niloya and Tospik
 */

#ifndef COEFFS_H
#define COEFFS_H

#define R_CLK		26
#define CS_LORA		15
#define DIO0_PIN	19
#define ID1_PIN		32
#define ID2_PIN		33
#define ID3_PIN		34
#define ID4_PIN		35
#define LORA_DIO0 	19

#define	k_SOH				0x01
#define k_EOT				0x04
#define k_LORA_DATA_SIZE 	16

#define k_CB_BUFFER_SIZE	16
// This value needs to check max device to scan. 1 is 500 ms acc. to alarm_supervisor_task running
#define k_MODBUS_ERR_TIMEOUT	10
#define k_LORA_ERR_TIMEOUT	10
#endif /* COEFFS_H */
