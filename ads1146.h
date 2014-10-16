/*
 * ads1146.h
 *
 *  Created on: 02/10/2014
 *      Author: BYC
 */

#ifndef ADS1146_H_
#define ADS1146_H_

#define GAIN1

#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>

#include <driverlib/debug.h>
#include <driverlib/fpu.h>
#include <driverlib/systick.h>
#include <driverlib/uart.h>
#include <driverlib/interrupt.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/pin_map.h>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/ssi.h>
#include <driverlib/uart.h>
#include "utils/uartstdio.h"


#include <stdint.h>

#define USE_ADS_INTERRUPT	1

//#define G 74.5294
#define G 334.3333


signed long offset;

unsigned long  data_obtain;
unsigned long readed[5];
signed long val1;

char stringNumber1[25];	// Tension leida
char stringNumber2[25];	// Velocidad leida
uint8_t string_size;

void Delay(unsigned long ulSeconds);


/****************************************
 * 		Registros del ADS 1146 / 1246
 *
 ****************************************/

#define ADC_REG_BCS         0x00  //Burnout Current Source Register.
#define ADC_REG_VBIAS       0x01  //Bias Voltage Register.
#define ADC_REG_MUX         0x02  //Multiplexer Control Register.
#define ADC_REG_SYS0        0x03  //System Control Register 0.
#define ADC_REG_OFC0        0x04  //Offset Calibration Coefficient Register 0
#define ADC_REG_OFC1        0x05  //Offset Calibration Coefficient Register 1
#define ADC_REG_OFC2        0x06  //Offset Calibration Coefficient Register 2
#define ADC_REG_FSC0        0x07  //Full-Scale Calibration Coefficient Register 0
#define ADC_REG_FSC1        0x08  //Full-Scale Calibration Coefficient Register 1
#define ADC_REG_FSC2        0x09  //Full-Scale Calibration Coefficient Register 2
#define ADC_REG_ID          0x0A  //ID Register

//ADS1246
#define ADC_GAIN_1          0x00
#define ADC_GAIN_2          0x10
#define ADC_GAIN_4          0x20
#define ADC_GAIN_8          0x30
#define ADC_GAIN_16         0x40
#define ADC_GAIN_32         0x50
#define ADC_GAIN_64         0x60
#define ADC_GAIN_128        0x70

//ADS1246
#define ADC_SPS_5           0x00
#define ADC_SPS_10          0x01
#define ADC_SPS_20          0x02
#define ADC_SPS_40          0x03
#define ADC_SPS_80          0x04
#define ADC_SPS_160         0x05
#define ADC_SPS_320         0x06
#define ADC_SPS_640         0x07
#define ADC_SPS_1000        0x08
#define ADC_SPS_2000        0x09

//ADS1246
#define ADC_MODE_SINGLECOV      0x00        //
#define ADC_MODE_CONTINUOUS     0x01        //

/****************************************
 * 		Comandos del ADS 1146/1246
 *
 ****************************************/


#define ADC_CMD_WAKEUP      0x00            //	Exit sleep mode
#define ADC_CMD_SLEEP       0x02            //	Enter sleep mode
#define ADC_CMD_SYNC        0x04            //	Synchronize the A/D conversion
#define ADC_CMD_RESET       0x06            //	Reset to power-up values
#define ADC_CMD_NOP     	0xFF            //	No operation
#define ADC_CMD_RDATA       0x12            //  Read data once
#define ADC_CMD_RDATAC      0x14            //  Read data continuously
#define ADC_CMD_SDATAC      0x16            //  Stop reading data continuously
#define ADC_CMD_RREG        0x20            //  Read from register rrrr  ( RREG = 0010 rrrr => RREG|0brrrr)
#define ADC_CMD_WREG        0x40            //  Write to register rrrr	 ( WREG = 0100 rrrr => WREG|0brrrr)
#define ADC_CMD_SYSOCAL     0x60            //
#define ADC_CMD_SYSGCAL     0x61            //
#define ADC_CMD_SELFOCAL    0x62            //
#define ADC_CMD_RESTRICTED  0xF1            //


/*********************************************
 *
 ********************************************/
#define ADS_CS_PIN GPIO_PIN_3
#define ADS_CS_PORT GPIO_PORTA_BASE

#define ADS_START_PIN GPIO_PIN_6
#define ADS_START_PORT GPIO_PORTC_BASE

#define ADS_DRY_PIN GPIO_PIN_7
#define ADS_DRY_PORT GPIO_PORTC_BASE

#define ADS_RST_PIN GPIO_PIN_4
#define ADS_RST_PORT GPIO_PORTF_BASE


#define ADS_CHIP_SELECT_CLR() MAP_GPIOPinWrite(ADS_CS_PORT, ADS_CS_PIN, ADS_CS_PIN) // Pongo a  cero chip select
#define ADS_CHIP_SELECT() MAP_GPIOPinWrite(ADS_CS_PORT, ADS_CS_PIN, 0x00) // Pongo a  cero chip select

#define ADS_START() MAP_GPIOPinWrite(ADS_START_PORT, ADS_START_PIN, ADS_START_PIN) // Pongo a  cero chip select
#define ADS_START_CLR() MAP_GPIOPinWrite(ADS_START_PORT, ADS_START_PIN, 0x00) // Pongo a  cero chip select

#define ADS_RESET() MAP_GPIOPinWrite(ADS_RST_PORT, ADS_RST_PIN, 0x00) // Pongo a  cero chip select
#define ADS_RESET_CLR() MAP_GPIOPinWrite(ADS_RST_PORT, ADS_RST_PIN, ADS_RST_PIN) // Pongo a  cero chip select

#define ADS_DATA_READY() MAP_GPIOPinRead(ADS_DRY_PORT, ADS_DRY_PIN)


void AdsPinConfig(void);

unsigned long Ads_Read();
void Ads_Config(uint8_t CovGain,uint8_t CovRate);
void Ads_Stop();
void Ads_Start(uint8_t CovMode);

void Ads_WriteReg(uint8_t RegAddr,uint8_t *Buffer,uint8_t Length);
void Ads_ReadReg(uint8_t RegAddr,uint8_t *Buffer,uint8_t Length);
void Ads_WriteCmd(uint8_t Cmd);
void Ads_ReadBytes(uint8_t *RxBuffer,uint16_t RxLenth);
void Ads_WriteBytes(uint8_t *TxBuffer,uint16_t TxLenth);
uint8_t Ads_Init(uint8_t Gain);

uint32_t Ads_ReadData();

#define ADS_PROM 16
#define GAIN1

uint32_t  ads_cont;
signed long ads_suma;

uint8_t Ads_WaitBusy();
uint8_t Ads_Calibrate(uint8_t Gain);


void DataReadyIntHandler(void);

uint8_t spi_send0(uint8_t c);
static void spi_init0(void) ;






#endif /* ADS1146_H_ */
