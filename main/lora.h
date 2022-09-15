	/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: 
 *  (1) "AS IS" WITH NO WARRANTY; 
 *  (2) TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, HopeRF SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) HopeRF
 *
 * website: www.HopeRF.com
 *          www.HopeRF.cn   
 */

/*! 
 * file       lora.h
 * brief      driver for RFM92/95/96/98
 * hardware   HopeDuino with HopeRF's LoRa COB rf-module
 *            
 *
 * version    1.0
 * date       Jun 3 2014
 * author     QY Ruan
 */

#ifndef LORA_H
#define LORA_H

#include <stdint.h>
#include <stdbool.h>

/** Hardware brief **/

#define DIO0
#define	POR

//PORTD
/** DIO1~DIO4 can be select to connect **/
#define	DIO4
#define	DIO3
#define	DIO2
#define	DIO1

//Output
#define POROut()	 // POR set output
#define	PORIn()		// POR set input
#define SetPOR()	//HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET)
#define	ClrPOR()	//HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET)

typedef enum 
{
	OOK,
	FSK,
	GFSK,
	LORA,
}lora_modulation_t;

typedef enum
{
	RFM92,
	RFM93,
	RFM95,
	RFM96,
	RFM97,
	RFM98,
}lora_modul_t;

typedef enum
{
	SF6,
	SF7,
	SF8,
	SF9,
	SF10,
	SF11,
	SF12,
}lora_sf_t;

typedef enum
{
	BW62K,
	BW125K,
	BW250K,
	BW500K,
}lora_bw_t;

typedef enum
{
	CR4_5,
	CR4_6,
	CR4_7,
	CR4_8,
}lora_cr_t;

typedef struct
{
	lora_modulation_t Modulation;					//OOK/FSK/GFSK/LORA
	lora_modul_t COB;//Chip on board

	//common parameter
	uint32_t 	Frequency;//unit: KHz
	uint8_t 	OutputPower;//unit: dBm   range: 2-20 [2dBm~+20dBm] 
	uint16_t 	PreambleLength;//unit: uint8_t

	bool FixedPktLength;//OOK/FSK/GFSK:
						//	 		true-------fixed packet length
						//   		false------variable packet length
						//		LoRa:
						//	 		true-------implicit header mode
						//      false------explicit header mode

	bool CrcDisable;//OOK/FSK/GFSK:
					//		true-------CRC disable
					//		fasle------CRC enable with CCITT
					//LoRa:
					//		true-------Header indicates CRC off
					//		false------Header indicates CRC on
	uint8_t PayloadLength;//PayloadLength is need to be set a value, when FixedPktLength is true. 		

	//for OOK/FSK/GFSK parameter
	uint32_t SymbolTime;//unit: ns
	uint32_t Devation;//unit: KHz
	uint16_t BandWidth;//unit: KHz
	uint8_t SyncLength;//unit: none, range: 1-8[uint8_t], value '0' is not allowed!
	uint8_t SyncWord[8];

	//for LoRa parameter
	lora_sf_t SFSel;//unit: none, range: SF6~SF12
	lora_bw_t BWSel;
	lora_cr_t CRSel;

	uint16_t BitRateValue;
	uint16_t DevationValue;
	uint8_t BandWidthValue;
	uint8_t SFValue;
	uint8_t BWValue;
	uint8_t CRValue;
	bool RsOptimize;
}lora_t;


void lora_init(lora_t *s);
void lora_rx_mode(lora_t *s);
void lora_standby(void);
void lora_sleep(void);
bool lora_send_msg(lora_t *s, const uint8_t *msg, uint8_t length);
uint8_t lora_get_msg(lora_t *s, uint8_t *msg);
void lora_reset(lora_t *s);

void spi_lora_init(void);

#endif
