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
 * @file       lora.c
 * @brief      driver for RFM92/95/96/98
 * hardware   HopeRF's RFduino TRx & with HopeRF's LoRa COB rf-module  
 *            
 *
 * @version    1.0
 * @date       Jun 3 2014
 * @author     QY Ruan
 */

#include "lora.h"
#include "coeffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

/********************************************************** 
 **RF69 Regsister define                                      
 **********************************************************/
//Common Register
#define 	RegFifo 			0x00 
#define 	RegOpMode 			0x01 
#define 	RegFrMsb 			0x06 
#define 	RegFrMid 			0x07 
#define 	RegFrLsb 			0x08
#define		RegPaConfig			0x09
#define 	RegPaRamp 			0x0A 
#define 	RegOcp 			    0x0B
#define 	RegLna 			    0x0C 

#define 	RegDioMapping1 	    0x40 
#define 	RegDioMapping2 	    0x41 
#define 	RegVersion 		    0x42 

//for 6/7/8
#define		RegPllHop			0x44
#define		RegTcxo				0x4B
#define		RegPaDac			0x4D
#define		RegFormerTemp		0x5B
#define		RegBitrateFrac		0x5D
#define		RegAgcRef			0x61
#define		RegAgcThresh1		0x62
#define		RegAgcThresh2		0x63
#define		RegAgcThresh3		0x64
#define		RegPll				0x70

//for 2/3
#define		RegAgcRef_2			0x43
#define		RegAgcThresh1_2		0x44
#define		RegAgcThresh2_2		0x45
#define		RegAgcThresh3_2		0x46
#define		RegPllHop_2			0x4B
#define		RegTcxo_2			0x58
#define		RegPaDac_2			0x5A
#define		RegPll_2			0x5C
#define		RegPllLowPn			0x5E
#define		RegFormerTemp_2		0x6C
#define		RegBitrateFrac_2	0x70

//ASK/FSK/GFSK Regsister
#define 	RegBitrateMsb 		0x02
#define 	RegBitrateLsb 		0x03 
#define 	RegFdevMsb 		    0x04 
#define 	RegFdevLsb 		    0x05 

#define		RegRxConfig			0x0D
#define 	RegRssiConfig 		0x0E
#define		RegRssiCollision	0x0F
#define 	RegRssiThresh 		0x10 
#define 	RegRssiValue 		0x11
#define 	RegRxBw 			0x12
#define 	RegAfcBw 			0x13
#define 	RegOokPeak 		    0x14 
#define 	RegOokFix 			0x15 
#define 	RegOokAvg 			0x16  
#define 	RegAfcFei 			0x1A
#define 	RegAfcMsb 			0x1B
#define 	RegAfcLsb 			0x1C 
#define 	RegFeiMsb 			0x1D
#define 	RegFeiLsb 			0x1E 
#define		RegPreambleDetect	0x1F
#define 	RegRxTimeout1 		0x20 
#define 	RegRxTimeout2 		0x21 
#define 	RegRxTimeout3 		0x22 
#define		RegRxDelay			0x23
#define 	RegOsc 				0x24
#define 	RegPreambleMsb 	    0x25 
#define 	RegPreambleLsb 	    0x26 
#define 	RegSyncConfig 		0x27
#define 	RegSyncValue1		0x28 
#define 	RegSyncValue2       0x29 
#define 	RegSyncValue3       0x2A 
#define 	RegSyncValue4       0x2B 
#define 	RegSyncValue5       0x2C 
#define 	RegSyncValue6       0x2D 
#define 	RegSyncValue7       0x2E 
#define 	RegSyncValue8       0x2F
#define 	RegPacketConfig1 	0x30 
#define 	RegPacketConfig2 	0x31 
#define 	RegPayloadLength 	0x32 
#define 	RegNodeAdrs 		0x33 
#define 	RegBroadcastAdrs 	0x34 
#define 	RegFifoThresh 		0x35
#define		RegSeqConfig1		0x36
#define		RegSeqConfig2		0x37
#define		RegTimerResol		0x38
#define		RegTimer1Coef		0x39
#define		RegTimer2Coef		0x3A
#define		RegImageCal			0x3B
#define		RegTemp				0x3C
#define		RegLowBat			0x3D
#define 	RegIrqFlags1 		0x3E
#define 	RegIrqFlags2 		0x3F

//LoRa Regsister
#define		RegFifoAddrPtr				0x0D
#define 	RegFifoTxBaseAddr			0x0E
#define		RegFifoRxBaseAddr			0x0F
#define 	RegFifoRxCurrentAddr		0x10 
#define 	RegIrqFlagsMask				0x11
#define 	RegIrqFlags					0x12
#define 	RegRxNbBytes				0x13
#define 	RegRxHeaderCntValueMsb    	0x14 
#define 	RegRxHeaderCntValueLsb		0x15 
#define 	RegRxPacketCntValueMsb		0x16  
#define 	RegRxPacketCntValueLsb		0x17
#define 	RegModemStat				0x18
#define 	RegPktSnrValue				0x19
#define 	RegPktRssiValue				0x1A
#define 	RegRssiValue_LR				0x1B
#define		RegHopChannel				0x1C
#define 	RegModemConfig1				0x1D
#define 	RegModemConfig2 			0x1E
#define 	RegSymbTimeoutLsb 			0x1F
#define		RegPreambleMsb_LR			0x20
#define 	RegPreambleLsb_LR			0x21
#define 	RegPayloadLength_LR 	    0x22
#define		RegMaxPayloadLength			0x23
#define		RegHopPeriod				0x24
#define		RegFifoRxByteAddr			0x25
#define		RegModemConfig3				0x26		// only for 6/7/8

/**********************************************************      
 **RF69 mode status                                          
 **********************************************************/
#define		RADIO_SLEEP			(0x00)
#define		RADIO_STANDBY		(0x01)
#define		RADIO_TX			(0x03)
#define		RADIO_RX			(0x05)

#define		FskMode				(0<<5)
#define		OokMode				(1<<5) 

#define		Shaping				2

#define 	MODE_MASK			0xF8

#define		MOUDLE_MASK_2		0x87			//for RFM92/93
#define		MOUDLE_MASK_1		0x9F			//for RFM95/96/97/98

#define		AFC_ON				(1<<4)
#define		AGC_ON				(1<<3)
#define		RX_TRIGGER			0x06

#define		PREAMBLE_DECT_ON	(1<<7)
#define		PREAMBLE_DECT_1BYTE	(0<<5)
#define		PREAMBLE_DECT_2BYTE	(1<<5)
#define		PREAMBLE_DECT_3BYTE	(2<<5)

#define		AUTO_RST_RX_OFF		(0<<6)
#define		AUTO_RST_RX_ON		(1<<6)
#define		AUTO_RST_RX_ONwPLL	(2<<6)	
#define		SYNC_ON				(1<<4)

//for PacketConfig
#define		VariablePacket		(1<<7)
#define		DcFree_NRZ			(0<<5)
#define		DcFree_MANCHESTER	(1<<5)
#define		DcFree_WHITENING	(2<<5)
#define		CrcOn				(1<<4)
#define		CrcDisAutoClear		(1<<3)
#define		AddrFilter_NONE		(0<<1)
#define		AddrFilter_NODE		(1<<1)
#define		AddrFilter_ALL		(2<<1)
#define		CrcCalc_CCITT		0x00
#define		CrcCalc_IBM			0x01

#define		PacketMode			(1<<6)
#define		ContinuousMode		(0<<6)

//for LoRa
#define		AllIrqMask				0xFF
#define		RxTimeoutMask			(1<<7)
#define		RxDoneMask				(1<<6)
#define		PayloadCrcErrorMask		(1<<5)
#define		ValidHeaderMask			(1<<4)
#define		TxDoneMask				(1<<3)
#define		CadDoneMask				(1<<2)
#define		FhssChangeChannelMask	(1<<1)
#define		CadDetectedMask			(1<<0)

//Input
#define	DIO0_H()	(gpio_get_level(19) == true)
#define	DIO0_L()	(gpio_get_level(19) == false)

#define CS_LORA_L()	gpio_set_level(15, false)
#define CS_LORA_H()	gpio_set_level(15, true)

spi_device_handle_t spi3;

void spi_lora_init(void)
{
	spi_bus_config_t buscfg =
	{
			.miso_io_num = 12,
			.mosi_io_num = 13,
			.sclk_io_num = 14,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.max_transfer_sz = 0,
	};
	spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_DISABLED);

	spi_device_interface_config_t devcfg =
	{
			.clock_speed_hz = 10000,
			.mode = 0,
			.spics_io_num = -1,
			.queue_size = 1,
			.pre_cb = NULL,
			.post_cb = NULL,
			.flags = 0,
	};
	spi_bus_add_device(SPI3_HOST, &devcfg, &spi3);
}

static void spi_transmit(const uint8_t *ab_data, uint16_t w_size)
{
    spi_transaction_t t =
    {
       .flags = 0,
       .length = 8 * w_size,
       .tx_buffer = ab_data,
       .rx_buffer = NULL,
    };
    spi_device_transmit(spi3, &t);
}

static void spi_transmit_receive(uint8_t *ab_data, uint16_t w_size)
{
    spi_transaction_t t =
    {
       .flags = 0,
       .length = 8 * w_size,
       .tx_buffer = NULL,
       .rx_buffer = ab_data,
    };
    spi_device_transmit(spi3, &t);
}

static void spi_write(uint16_t w_data)
{
	uint8_t b_data = (uint8_t) ((w_data >> 8) | 0x80);
	CS_LORA_L();
	spi_transmit(&b_data, 1);
	b_data = (uint8_t) w_data;
	spi_transmit(&b_data, 1);
	CS_LORA_H();
}

static void spi_burst_write(uint8_t b_addr, const uint8_t *ab_data, uint16_t w_size)
{
	uint8_t b_tmp = b_addr | 0x80;
	CS_LORA_L();
	spi_transmit(&b_tmp, 1);
	spi_transmit(ab_data, w_size);
	CS_LORA_H();
}

static void spi_read(uint8_t b_addr, uint8_t *ab_data, uint16_t w_size)
{
	CS_LORA_L();
	spi_transmit(&b_addr, 1);
	spi_transmit_receive(ab_data, w_size);
	CS_LORA_H();
}

static void delay_ms(uint32_t ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms + 10));
}

static uint8_t lora_select_bw(uint16_t rx_bw);
static uint8_t lora_select_ramping(uint32_t symbol);
/**
 * @brief: initialize rfm9x
 * @param: *s lora_t type handler
 * @retval: none
 */
void lora_init(lora_t *s)
{
	uint8_t i = 0, j = 0;
	uint8_t sync = 0;

//	ClrPOR(); PORIn(); DIO0In();
	uint32_t freq = (s->Frequency << 11) / 125;					//calc frequency
	uint32_t BitRateValue = (s->SymbolTime << 5) / 1000;		//calc bitrate
	uint32_t DevationValue = (s->Devation << 11) / 125;			//calc deviation
	uint32_t BandWidthValue = lora_select_bw(s->BandWidth);

	switch (s->SFSel)
	{
	case SF6:
		s->SFValue = 6;
		break;
	case SF7:
		s->SFValue = 7;
		break;
	case SF8:
		s->SFValue = 8;
		break;
	case SF10:
		s->SFValue = 10;
		break;
	case SF11:
		s->SFValue = 11;
		break;
	case SF12:
		s->SFValue = 12;
		break;
	case SF9:
	default:
		s->SFValue = 9;
		break;
	}
	switch (s->BWSel)
	{
	case BW62K:
		s->BWValue = 6;
		break;				//for RFM95/96/97/98
	case BW250K:
		s->BWValue = 8;
		break;
	case BW500K:
		s->BWValue = 9;
		break;
	case BW125K:
	default:
		s->BWValue = 7;
		break;
	}
	switch (s->CRSel)
	{
	default:
	case CR4_5:
		s->CRValue = 1;
		break;
	case CR4_6:
		s->CRValue = 2;
		break;
	case CR4_7:
		s->CRValue = 3;
		break;
	case CR4_8:
		s->CRValue = 4;
		break;
	}

	if ((s->SFValue - 4) >= s->BWValue)
	{
		s->RsOptimize = true;
	}
	else
	{
		s->RsOptimize = false;
	}

	lora_reset(s);
	lora_standby();
	//Frequency
	spi_write((uint16_t) ((uint32_t) RegFrMsb << 8) + ((freq >> 16) & 0xFF));
	spi_write((uint16_t) ((uint32_t) RegFrMid << 8) + ((freq >> 8) & 0xFF));
	spi_write((uint16_t) ((uint32_t) RegFrLsb << 8) + (freq & 0xFF));
	//PA Config
	spi_read(RegPaConfig, &i, 1);
	i &= 0x70;
	if (s->OutputPower >= 20)
	{
		i |= 0x0F;
		j = 0x87;
	}
	else if (s->OutputPower > 17)
	{
		i |= (s->OutputPower - 3 - 2);
		j = 0x87;
	}
	else if (s->OutputPower >= 2)
	{
		i |= (s->OutputPower - 2);
		j = 0x84;
	}
	else
	{
		i |= 0;
		j = 0x84;
	}

	spi_write(((uint16_t) RegPaConfig << 8) + 0x80 + i);	//PA_BOOST
	switch (s->COB)
	{
	case RFM92:
	case RFM93:
		spi_write(((uint16_t) RegPaDac_2 << 8) + j);
		break;
	default:
		spi_write(((uint16_t) RegPaDac << 8) + j);
		break;
	}
	spi_read(RegPaRamp, &j, 1);
	j &= 0x0F;
	j |= lora_select_ramping(s->SymbolTime);
	spi_write(((uint16_t) RegPaRamp << 8) + j);

	//Ocp
	spi_write(((uint16_t) RegOcp << 8) + 0x0F);			//Disable Ocp

	//LNA
	//spi_write(((uint16_t)RegLna<<8)+0x20);		//High & LNA Enable

	// Mode spi_read(uint8_t b_addr, uint8_t *ab_data, uint16_t w_size)
	spi_read(RegOpMode, &i, 1);
	spi_read(RegPaRamp, &j, 1); //for RFM95/96/97/98

	if (s->Modulation == LORA)
	{
		i &= 0x87;
		switch (s->COB)
		{
		case RFM96:
		case RFM98:
			i |= 0x08;
			break;
		default:
			break;
		}
		spi_write(((uint16_t) RegOpMode << 8) + i);

		spi_read(0x31, &i, 1);			//SetNbTrigPeaks
		i &= 0xF8;
		if (s->SFSel == SF6)
		{
			i |= 0x05;
			spi_write(0x3100 + i);
			spi_write(0x3700 + 0x0C);
			s->FixedPktLength = true;			//SF6 must be ImplicitHeaderMode
		}
		else
		{
			i |= 0x03;
			spi_write(0x3100 + i);
		}

		switch (s->COB)
		{
		uint8_t tmp;
	case RFM92:
	case RFM93:
		if (s->BWValue > 6)
			tmp = s->BWValue - 7;
		else
			tmp = 0;
		tmp <<= 6;					//BandWidth
		tmp |= (s->CRValue << 3);
		if (s->FixedPktLength)			//ImplicitHeader
			tmp |= 0x04;
		if (!s->CrcDisable)				//
			tmp |= 0x02;
		if (s->RsOptimize)	//mandated for when the symbol length exceeds 16ms
			tmp |= 0x01;
		spi_write(((uint16_t) RegModemConfig1 << 8) + tmp);
		tmp = (s->SFValue << 4);			//SF rate
		tmp |= (0x04 + 0x03);			//AGC ON & Max timeout
		spi_write(((uint16_t) RegModemConfig2 << 8) + tmp);
		break;
	case RFM95:
	case RFM97:
	case RFM96:
	case RFM98:
	default:
		tmp = (s->BWValue << 4) + (s->CRValue << 1);
		if (s->FixedPktLength)
			tmp |= 0x01;
		spi_write(((uint16_t) RegModemConfig1 << 8) + tmp);
		tmp = (s->SFValue << 4);
		if (!s->CrcDisable)
			tmp |= 0x04;
		tmp += 0x03;
		spi_write(((uint16_t) RegModemConfig2 << 8) + tmp);
		tmp = 0x04;				//AGC ON
		if (s->RsOptimize)	//mandated for when the symbol length exceeds 16ms
			tmp |= 0x08;
		spi_write(((uint16_t) RegModemConfig3 << 8) + tmp);
		break;
		}
		spi_write(((uint16_t) RegSymbTimeoutLsb << 8) + 0xFF);//RegSymbTimeoutLsb Timeout = 0x3FF(Max)	
		spi_write(((uint16_t) RegPreambleMsb_LR << 8) + (uint8_t) (s->PreambleLength >> 8));
		spi_write(((uint16_t) RegPreambleLsb_LR << 8) + (uint8_t) s->PreambleLength);

		spi_write(((uint16_t) RegDioMapping2 << 8) + 0x40);	//RegDioMapping2	DIO5=00(ModeReady), DIO4=01(PllLock)
	}
	else
	{
		switch (s->COB)
		{
		case RFM92:
		case RFM93:
			i &= MOUDLE_MASK_2;
			switch (s->Modulation)
			{
			case OOK:
				i |= OokMode + (Shaping << 3);
				break;
			case GFSK:
				i |= FskMode + (Shaping << 3);
				break;
			case FSK:
			default:
				i |= FskMode;
				break;
			}
			break;
		default:
			i &= MOUDLE_MASK_1;
			j &= 0x9F;
			switch (s->Modulation)
			{
			case OOK:
				i |= OokMode;
				j |= (Shaping << 5);
				break;
			case GFSK:
				i |= FskMode;
				j |= (Shaping << 5);
				break;
			case FSK:
			default:
				i |= FskMode;
				break;
			}
			break;
		}
		spi_write(((uint16_t) RegOpMode << 8) + i);
		spi_write(((uint16_t) RegPaRamp << 8) + j);

		//BitRate 
		spi_write(((uint16_t) RegBitrateMsb << 8) + (uint8_t) (s->BitRateValue >> 8));
		spi_write(((uint16_t) RegBitrateLsb << 8) + (uint8_t) s->BitRateValue);

		//Devation
		spi_write(((uint16_t) RegFdevMsb << 8) + (((uint8_t) (s->DevationValue >> 8)) & 0x3F));
		spi_write(((uint16_t) RegFdevLsb << 8) + (uint8_t) (s->DevationValue & 0xFF));

		//RxConfig
		spi_write(((uint16_t) RegRxConfig << 8) + AGC_ON + RX_TRIGGER);

		//RxBw
		spi_write(((uint16_t) RegRxBw << 8) + s->BandWidthValue);

		//OOK
		spi_write(((uint16_t) RegOokPeak << 8) + 0x20 + (0x02 << 3) + 0x00);

		//PreambleDetect
		spi_write(((uint16_t) RegPreambleDetect << 8) + PREAMBLE_DECT_ON + PREAMBLE_DECT_3BYTE);
		spi_write(((uint16_t) RegPreambleMsb << 8) + (uint8_t) (s->PreambleLength >> 8));
		spi_write(((uint16_t) RegPreambleLsb << 8) + (uint8_t) s->PreambleLength);

		//Osc
		spi_write(((uint16_t) RegOsc << 8) + 0x07);	//Close OscClk Output

		//SyncConfig
		if (s->SyncLength == 0)
			sync = 0;
		else
			sync = s->SyncLength - 1;
		spi_write(((uint16_t) RegSyncConfig << 8) + AUTO_RST_RX_ONwPLL + SYNC_ON + (sync & 0x07));
		for (i = 0; i < 8; i++)								//SyncWordSetting
			spi_write(((uint16_t) (RegSyncValue1 + i) << 8) + s->SyncWord[i]);

		i = DcFree_NRZ + AddrFilter_NONE + CrcCalc_CCITT;
		if (!s->FixedPktLength)
			i += VariablePacket;
		if (!s->CrcDisable)
			i += CrcOn;
		spi_write(((uint16_t) RegPacketConfig1 << 8) + i);
		spi_write(((uint16_t) RegPacketConfig2 << 8) + PacketMode);

		if (s->FixedPktLength)						//Set Packet length
			spi_write(((uint16_t) RegPayloadLength << 8) + s->PayloadLength);
		else
			spi_write(((uint16_t) RegPayloadLength << 8) + 0xFF);

		spi_write(((uint16_t) RegFifoThresh << 8) + 0x01);
		spi_write(((uint16_t) RegDioMapping2 << 8) + 0x61);	//DIO4 PllLock / DIO5 Data / PreambleDetect
	}

	lora_standby();
}

/**********************************************************
 **Name:     lora_rx_mode
 **Function: set rf9x to receive mode
 **Input:    none
 **Output:   none
 **********************************************************/
void lora_rx_mode(lora_t *s)
{
	uint8_t tmp;

	if (s->Modulation == LORA)
	{
		if (s->FixedPktLength)							//Set Packet length
			spi_write(((uint16_t) RegPayloadLength_LR << 8) + s->PayloadLength);
		else
			spi_write(((uint16_t) RegPayloadLength_LR << 8) + 0xFF);

		spi_write(((uint16_t) RegDioMapping1 << 8) + 0x22);	//DIO0 RxDone / DIO1 CadDetected / DIO2 FhssChangeChannel /DIO3 PayloadCrcError
		spi_write(((uint16_t) RegIrqFlags << 8) + AllIrqMask);//Clear All Interrupt
		spi_write(((uint16_t) RegIrqFlagsMask << 8) + (AllIrqMask & (~(RxDoneMask | RxTimeoutMask))));//just enable RxDone & Timeout

		spi_read(RegFifoRxBaseAddr, &tmp, 1);				//Read RxBaseAddr
		spi_write(((uint16_t) RegFifoAddrPtr << 8) + tmp);//RxBaseAddr -> FiFoAddrPtr��
	}
	else
	{
		if (s->CrcDisable)
			spi_write(((uint16_t) RegDioMapping1 << 8) + 0x00);	//DIO0 PayloadReady / DIO1 FifoLevel / DIO2 FifoFull /DIO3 FifoEmpty
		else
			spi_write(((uint16_t) RegDioMapping1 << 8) + 0x40);	//DIO0 CrcOk  / DIO1 FifoLevel / DIO2 FifoFull /DIO3 FifoEmpty 	
	}

	spi_read(RegOpMode, &tmp, 1);
	tmp &= MODE_MASK;
	tmp |= RADIO_RX;
	spi_write(((uint16_t) RegOpMode << 8) + tmp);
}

/**********************************************************
 **Name:     lora_standby
 **Function: set rf9x to standby mode
 **Input:    none
 **Output:   none
 **********************************************************/
void lora_standby(void)
{
	uint8_t tmp;
	spi_read(RegOpMode, &tmp, 1);
	tmp &= MODE_MASK;
	tmp |= RADIO_STANDBY;
	spi_write(((uint16_t) RegOpMode << 8) + tmp);
}

/**********************************************************
 **Name:     lora_sleep
 **Function: set rf9x to sleep mode
 **Input:    none
 **Output:   none
 **********************************************************/
void lora_sleep(void)
{
	uint8_t tmp;
	spi_read(RegOpMode, &tmp, 1);
	tmp &= MODE_MASK;
	tmp |= RADIO_SLEEP;
	spi_write(((uint16_t) RegOpMode << 8) + tmp);
}

/**********************************************************
 **Name:     lora_send_msg
 **Function: set rf9x to send message
 **Input:    msg------for which message to send
 length---message length
 **Output:   true-----send ok
 false----send error/over time
 **********************************************************/
bool lora_send_msg(lora_t *s, const uint8_t *msg, uint8_t length)
{
	uint8_t tmp;
	uint32_t overtime;
	uint16_t bittime;

	if (s->Modulation == LORA)
	{
		spi_write(((uint16_t) RegPayloadLength_LR << 8) + length);
		spi_write(((uint16_t) RegDioMapping1 << 8) + 0x62);	//DIO0 TxDone / DIO1 CadDetected / DIO2 FhssChangeChannel /DIO3 PayloadCrcError
		spi_write(((uint16_t) RegIrqFlags << 8) + AllIrqMask);//Clear All Interrupt
		spi_write(((uint16_t) RegIrqFlagsMask << 8) + (AllIrqMask & (~TxDoneMask)));//just enable TxDone

		spi_read(RegFifoTxBaseAddr, &tmp, 1);				//Read TxBaseAddr
		spi_write(((uint16_t) RegFifoAddrPtr << 8) + tmp);//RxBaseAddr -> FiFoAddrPtr��

		spi_burst_write(RegFifo, (uint8_t*) msg, length);
	}
	else
	{
		spi_write(((uint16_t) RegDioMapping1 << 8) + 0x00);	//DIO0 PacketSend  / DIO1 FifoLevel / DIO2 FifoFull /DIO3 FifoEmpty

		if (!s->FixedPktLength)
		{
			spi_write(((uint16_t) RegFifo << 8) + length);
		}

		spi_burst_write(RegFifo, (uint8_t*) msg, length);
	}

	spi_read(RegOpMode, &tmp, 1);
	tmp &= MODE_MASK;
	tmp |= RADIO_TX;
	spi_write(((uint16_t) RegOpMode << 8) + tmp);

	if (s->Modulation == LORA)
	{
		overtime = s->PreambleLength + 5 + 8;	//unit: uint8_t
		bittime = (length << 3) + 28 - (s->SFValue << 2);
		if (!s->CrcDisable)
			bittime += 16;
		if (s->FixedPktLength)
			bittime -= 20;
		switch (s->CRSel)
		{
		default:
		case CR4_5:
			bittime *= 5;
			break;
		case CR4_6:
			bittime *= 6;
			break;
		case CR4_7:
			bittime *= 7;
			break;
		case CR4_8:
			bittime *= 8;
			break;
		}
		bittime >>= 2;
		if (s->RsOptimize)
			bittime /= 10;
		else
			bittime /= 12;
		overtime += bittime;//unit: uint8_t,  total = Payload + Preamble + Header

		if (s->SFValue >= s->BWValue)			//unit: ms
			overtime <<= (s->SFValue - s->BWValue);
		else
			overtime >>= (s->BWValue - s->SFValue);
		delay_ms(overtime);			//
		for (bittime = 0; bittime < 5000; bittime++)//about 500ms for overtime
		{
			if (DIO0_H())
				break;
			delay_ms(1);			//in orginal library, it is 100us
		}
		spi_write(((uint16_t) RegIrqFlags << 8) + AllIrqMask);//Clear All Interrupt 		
		lora_standby();
		if (bittime >= 5000)
			return (false);
		else
			return (true);
	}
	else
	{

		bittime = s->SymbolTime / 1000;		//unit: us
		overtime = s->SyncLength + s->PreambleLength + length;
		if (!s->FixedPktLength)				//SyncWord & PktLength & 2ByteCRC
			overtime += 1;
		if (!s->CrcDisable)
			overtime += 2;
		overtime <<= 3;						//8bit == 1byte
		overtime *= bittime;
		overtime /= 1000;					//unit: ms
		if (overtime == 0)
			overtime = 1;
		delay_ms(overtime);				//
		for (tmp = 0; tmp < 100; tmp++)			//about 10ms for overtime
		{
			if (DIO0_H())
				break;
			delay_ms(1); // 100 us
		}
		lora_standby();
		if (tmp >= 100)
			return (false);
		else
			return (true);
	}
}

/**********************************************************
 **Name:     lora_get_msg
 **Function: check receive packet
 **Input:    msg------for which message to read
 **Output:   packet length
 0--------have no packet
 **********************************************************/
uint8_t lora_get_msg(lora_t *s, uint8_t *msg)
{
	uint8_t length = 0;
	uint8_t b_tmp = 0;

//	if (DIO0_H())				//Receive CrcOk or PayloadReady
//	{
		if (s->Modulation == LORA)
		{
			uint8_t addr;
			spi_read(RegIrqFlags, &b_tmp, 1);//Read Interrupt Flag for do something
			spi_read(RegPktSnrValue, &b_tmp, 1);
			spi_read(RegRssiValue_LR, &b_tmp, 1);
			spi_read(RegPktRssiValue, &b_tmp, 1);

			spi_read(RegFifoRxCurrentAddr, &addr, 1);
			spi_write(((uint16_t) RegFifoAddrPtr << 8) + addr);
			spi_read(RegRxNbBytes, &length, 1);
			spi_read(RegFifo, msg, length);
			spi_write(((uint16_t) RegIrqFlags << 8) + AllIrqMask);//Clear All Interrupt
		}
		else
		{
			if (s->FixedPktLength)
			{
				length = s->PayloadLength;
			}
			else
			{
				spi_read(RegFifo, &length, 1);
			}
			spi_read(RegFifo, msg, length);
		}
		lora_standby();
		lora_rx_mode(s);
//	}
	return length;
}

/**********************************************************
 **Name:     lora_reset
 **Function: hardware reset rf69 chipset
 **Input:    none
 **Output:   none
 **********************************************************/
void lora_reset(lora_t *s)
{
	uint8_t tmp;
	POROut();
	switch (s->COB)
	{
	case RFM92:						//High Reset; Normal for Low
	case RFM93:
		ClrPOR();
		delay_ms(1);				//at least 100us for reset
		SetPOR();
		break;
	case RFM95:
	case RFM96:
	case RFM97:
	case RFM98:
	default:
		SetPOR();
		delay_ms(1);				//at least 100us for reset
		ClrPOR();
		break;
	} PORIn();							//set POR for free
	delay_ms(6);					//wait for ready	
	ClrPOR();							//note: help for LowReset
	spi_read(RegOpMode, &tmp, 1);
	tmp &= MODE_MASK;
	tmp |= RADIO_SLEEP;
	spi_write(((uint16_t) RegOpMode << 8) + tmp);
	tmp &= 0x7F;
	if (s->Modulation == LORA)
		tmp |= 0x80;
	spi_write(((uint16_t) RegOpMode << 8) + tmp);
}

/**********************************************************
 **Name:     lora_select_bandwidth
 **Function: 
 **Input:    BandWidth
 **Output:   BandWidthValue
 **********************************************************/
static uint8_t lora_select_bw(uint16_t rx_bw)
{
	if (rx_bw <= 10)
		return 0x15;						//10.4KHz 	Min
	else if (rx_bw < 13)
		return 0x0D;						//12.5KHz	
	else if (rx_bw < 16)
		return 0x05;						//15.6KHz
	else if (rx_bw < 21)
		return 0x14;						//20.8KHz
	else if (rx_bw <= 25)
		return 0x0C;						//25.0KHz
	else if (rx_bw < 32)
		return 0x04;						//31.3KHz
	else if (rx_bw < 42)
		return 0x13;						//41.7KHz
	else if (rx_bw <= 50)
		return 0x0B;						//50.0KHz
	else if (rx_bw < 63)
		return 0x03;						//62.5KHz
	else if (rx_bw < 84)
		return 0x12;						//83.3KHz
	else if (rx_bw <= 100)
		return 0x0A;						//100KHz
	else if (rx_bw <= 125)
		return 0x02;						//125KHz
	else if (rx_bw < 167)
		return 0x11;						//167KHz
	else if (rx_bw <= 200)
		return 0x09;						//200KHz
	else if (rx_bw <= 250)
		return 0x01;						//250KHz
	else if (rx_bw <= 333)
		return 0x10;						//333KHz
	else if (rx_bw <= 400)
		return 0x08;						//400KHz
	else
		return 0x00;						//500KHz Max
}

/**********************************************************
 **Name:     lora_select_ramping
 **Function: 
 **Input:    symbol time
 **Output:   ramping value
 **********************************************************/
static uint8_t lora_select_ramping(uint32_t symbol)
{
	uint32_t SymbolRate;

	SymbolRate = symbol / 1000;			//ns->us
	SymbolRate = SymbolRate / 4;			// 1/4 ramping

	if (SymbolRate <= 10)
		return 0x0F;					//10us
	else if (SymbolRate <= 12)
		return 0x0E;					//12us
	else if (SymbolRate <= 15)
		return 0x0D;					//15us
	else if (SymbolRate <= 20)
		return 0x0C;					//20us
	else if (SymbolRate <= 25)
		return 0x0B;					//25us
	else if (SymbolRate <= 31)
		return 0x0A;					//31us
	else if (SymbolRate <= 40)
		return 0x09;					//40us
	else if (SymbolRate <= 50)
		return 0x08;					//50us
	else if (SymbolRate <= 62)
		return 0x07;					//62us
	else if (SymbolRate <= 100)
		return 0x06;					//100us
	else if (SymbolRate <= 125)
		return 0x05;					//125us
	else if (SymbolRate <= 250)
		return 0x04;					//250us
	else if (SymbolRate <= 500)
		return 0x03;					//500us
	else if (SymbolRate <= 1000)
		return 0x02;					//1000us
	else if (SymbolRate <= 2000)
		return 0x01;					//2000us
	else
		return 0x00;
}
