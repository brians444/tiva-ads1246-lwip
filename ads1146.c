/*
 * ads1146.c
 *
 *  Created on: 02/10/2014
 *      Author: BYC
 */

#include "ads1146.h"
#include <math.h>
#include "stdio.h"



unsigned long Ads_Read(void)
{
	ADS_CHIP_SELECT();
	signed long val;
	float presion, velocidad, tension;
	float tleida;
	/*MAP_SSIDataPut(SSI0_BASE, ADC_CMD_RDATA);	// Pido datos
	MAP_SSIDataGet(SSI0_BASE, &readed[4]);
	MAP_SSIDataPut(SSI0_BASE, ADC_CMD_NOP);		// Leo primer byte
	MAP_SSIDataGet(SSI0_BASE, &readed[3]);
	//data_obtain = (val & 0xFF);
	//data_obtain = data_obtain<<8;*/
	MAP_SSIDataPut(SSI0_BASE, ADC_CMD_NOP);
	MAP_SSIDataGet(SSI0_BASE, &readed[2]);

	//data_obtain += (val & 0xFF);
	MAP_SSIDataPut(SSI0_BASE, ADC_CMD_NOP);
	MAP_SSIDataGet(SSI0_BASE, &readed[1]);

	MAP_SSIDataPut(SSI0_BASE, ADC_CMD_NOP);
	MAP_SSIDataGet(SSI0_BASE, &readed[0]);

	ADS_CHIP_SELECT_CLR();

	val = (readed[2]<<16) + (readed[1] << 8) + readed[0];


	/*********************************************
	 * PGA = 1
	 ********************************************/
#ifdef GAIN1
	if(readed[2]> 0x40)
	{
		//UARTprintf("datos leidos : %x %x %x \n", readed[2], readed[1], readed[0]);
		val = (~val)&0x7FFFFF;
		val = -val;
	}
	val = val+4194304;

	ads_suma = ads_suma + val;
	ads_cont++;

	if(ads_cont == ADS_PROM)
	{
		ads_suma = ads_suma /ADS_PROM;
		ads_cont = 0;
		ads_suma = ads_suma - offset;
		if(ads_suma<0)
			ads_suma = 0;
		tleida = ads_suma * 3.3000;
		tleida = tleida/8388608.0 ;
		ads_suma = 0;
#endif

#ifndef GAIN1
	/*
	 * PGA=  2
	 */
	if(readed[2]> 0x80)
	{
		//UARTprintf("datos leidos : %x %x %x \n", readed[2], readed[1], readed[0]);
		val = (~val)&0x7FFFFF;
		val = -val;
	}

	val = val+8388607;
	tleida = val * 3.3000;
	tleida = tleida/(2.0*8388608.0) ;
#endif

		presion = (tleida*1000.0)/(G*0.0035); // Presion en Pascales
		velocidad = (2.0*presion);
		velocidad /= 1.19;
		velocidad = sqrt(velocidad);
		velocidad = velocidad * 3.6;


		sprintf(stringNumber1, "%f", tleida);
		//UARTprintf("datos leidos : %d tension = %s \n", val, stringNumber1);
		sprintf(stringNumber2, "%f", velocidad);
		//UARTprintf("Velocidad = %s km/h \n \n", stringNumber2);


		//UARTprintf("Velocidad = %s m/s \n \n", stringNumber);
		//UARTprintf("Timer = %d\n", g_ulSysTickCount);
		return val;
	}
	else
		return 0;

}

uint8_t Ads_Init(uint8_t Gain)
{
    uint8_t i,Cmd;
    ADS_RESET();
    for(i=0;i<200;i++);
    ADS_RESET_CLR();
    for(i=0;i<200;i++);
    Ads_WriteCmd(ADC_CMD_RESET);
    Delay(1);
    Ads_WriteReg(ADC_REG_SYS0,&Gain,1);             //
    Cmd=0x00;
    /*Ads_WriteReg(ADC_REG_ID,&Cmd,1);                //DOUTDRDY
    if((Cmd&0x08)>0)return 1;*/
    Cmd=ADC_CMD_NOP;
    Ads_ReadReg(ADC_REG_ID,&Cmd,1);                 //ID
    UARTprintf("ID: %d \n", Cmd);

    //Cmd=Ads_Calibrate(Gain);                    //
    return Cmd;
}

uint32_t Ads_ReadData()
{
    uint8_t  Cmd[5];
    uint32_t D;
    Cmd[0]=ADC_CMD_RDATA;
    Cmd[1]=ADC_CMD_NOP;
    Cmd[2]=ADC_CMD_NOP;
    Cmd[3]=ADC_CMD_NOP;
    Cmd[4]=ADC_CMD_NOP;
    ADS_CHIP_SELECT();
    Ads_ReadBytes(Cmd,3);
    ADS_CHIP_SELECT_CLR();
    //Cmd[0]= (Cmd[1]&0x80)!=0 ? 0xFF:0x00;
    D = (Cmd[0]<<8*3) + (Cmd[1]<<8*2) +(Cmd[2]<<8*1) + (Cmd[3]<<8*0);
    UARTprintf("datos leidos : %d %d %d %d \n", Cmd[4], Cmd[3], Cmd[2], Cmd[1], Cmd[0]);
    UARTprintf("D = %d \n", D);
    return D;
}

uint8_t Ads_Calibrate(uint8_t Gain)
{
    uint8_t R=0,Cmd;
    Ads_WriteReg(ADC_REG_SYS0,&Gain,1);     //

    Cmd=0x00;
    Ads_WriteReg(ADC_REG_MUX,&Cmd,1);       //
    Ads_WriteCmd(ADC_CMD_SELFOCAL);     //
    R|=Ads_WaitBusy();                   //

    Cmd=0x01;
    Ads_WriteReg(ADC_REG_MUX,&Cmd,1);       //AINP+AINN=(AVDD+AVSS)/2
    Ads_WriteCmd(ADC_CMD_SYSOCAL);      //
    R|=Ads_WaitBusy();                   //

    Cmd=0x02;
    Ads_WriteReg(ADC_REG_MUX,&Cmd,1);       //AINP=VREF+,AINN=VREF-; for gain calibration
    Ads_WriteCmd(ADC_CMD_SYSGCAL);      //
    R|=Ads_WaitBusy();

    return R;
}

uint8_t Ads_WaitBusy()
{
    uint16_t i;
    ///ADS_CHIP_SELECT();
    i=0;
    while(ADS_DATA_READY()>0)
    	;
    //ADS_CHIP_SELECT_CLR();
    return 0;
}

void Ads_WriteBytes(uint8_t *TxBuffer,uint16_t TxLenth)
{
    uint16_t i=0;
    while(i<TxLenth){
    	MAP_SSIDataPut(SSI0_BASE, TxBuffer[i]);
    	i++;
    }

}

void Ads_ReadBytes(uint8_t *RxBuffer,uint16_t RxLenth)
{
	uint16_t i=0;
	unsigned long val;
    while(i<RxLenth){
    	MAP_SSIDataPut(SSI0_BASE, RxBuffer[i]);
    	MAP_SSIDataGet(SSI0_BASE, &val);
    	RxBuffer[i] = (unsigned long)(val & 0xFF);
    	i++;
    }
}


void Ads_WriteCmd(uint8_t Cmd)
{
	ADS_CHIP_SELECT();
    Ads_WriteBytes(&Cmd,1);
    ADS_CHIP_SELECT_CLR();
}

/*---------------------------------------------------------

---------------------------------------------------------*/
void Ads_ReadReg(uint8_t RegAddr,uint8_t *Buffer,uint8_t Length)
{
    uint8_t Cmd[2];
	ADS_CHIP_SELECT();
    Cmd[0]=ADC_CMD_RREG|RegAddr;
    Cmd[1]=Length-1;
    Ads_WriteBytes(Cmd,2);
    Ads_ReadBytes(Buffer,Length);
    Cmd[0]=ADC_CMD_NOP;
    Ads_WriteBytes(Cmd,1);
    ADS_CHIP_SELECT_CLR();
}

/*---------------------------------------------------------

---------------------------------------------------------*/
void Ads_WriteReg(uint8_t RegAddr,uint8_t *Buffer,uint8_t Length)
{
    uint8_t Cmd[2];
    ADS_CHIP_SELECT();
        Cmd[0]=ADC_CMD_WREG|RegAddr;
    Cmd[1]=Length-1;
    Ads_WriteBytes(Cmd,2);
    Ads_WriteBytes(Buffer,Length);
    ADS_CHIP_SELECT_CLR();
}

void Ads_Config(uint8_t CovGain,uint8_t CovRate)
{
    uint8_t Cmd;
    Cmd=CovGain|CovRate;
    Ads_WriteReg(ADC_REG_SYS0,&Cmd,1);              //

	/*ADS_CHIP_SELECT();
	ADS_START();
	ADS_RESET_CLR();
	uint8_t cmd0, cmd1,reg, data0;
	reg = 0x03;
	cmd0 = 0b0100000 | reg;
	cmd1 = 0x00;
	data0 = 0b00010001;
	MAP_SSIDataPut(SSI0_BASE, 0b00010110);	// Stop data continuously
	while(ADS_DATA_READY())
		;
	MAP_SSIDataPut(SSI0_BASE, cmd0);
	MAP_SSIDataPut(SSI0_BASE, cmd1);
	MAP_SSIDataPut(SSI0_BASE, data0);

	//MAP_SSIDataPut(SSI0_BASE, 0b00010100);	// Start data continuously
	//Ads_Calibrate(ADC_GAIN_2);
	ADS_CHIP_SELECT_CLR();*/
}

void Ads_Stop()
{
    ADS_START_CLR();                          //
}

void Ads_Start(uint8_t CovMode)
{
    ADS_START();                           //ADC
    if(CovMode==ADC_MODE_SINGLECOV)
    	ADS_START_CLR();            //
}


void AdsPinConfig()
{
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	MAP_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	MAP_GPIOPinConfigure(GPIO_PA4_SSI0RX);
	MAP_GPIOPinConfigure(GPIO_PA5_SSI0TX);

	MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);

	MAP_SSIConfigSetExpClk(SSI0_BASE, MAP_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 400000, 8);
	MAP_SSIEnable(SSI0_BASE);

 // MAP_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	MAP_GPIOPinTypeGPIOOutput(ADS_CS_PORT, ADS_CS_PIN); // ~ChipSelect
	MAP_GPIOPinTypeGPIOOutput(ADS_RST_PORT, ADS_RST_PIN); // ~Reset
	MAP_GPIOPinTypeGPIOOutput(ADS_START_PORT, ADS_START_PIN); // Start
	MAP_GPIOPinTypeGPIOInput(ADS_DRY_PORT, ADS_DRY_PIN); // ~DataReady

    //Ads_Init(ADC_GAIN_1);               //
    //Ads_Config(ADC_GAIN_1,ADC_SPS_5);           //1,5sps
    //Ads_Start(ADC_MODE_CONTINUOUS);         //

	ADS_RESET();
	Delay(5);
	// COnfiguro interrupcion de puerto B
	MAP_GPIODirModeSet(ADS_DRY_PORT, ADS_DRY_PIN, GPIO_DIR_MODE_IN);
	MAP_GPIOPadConfigSet(ADS_DRY_PORT, ADS_DRY_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);


#ifndef GAIN1
	ADS_RESET_CLR();
	ADS_START();
	ADS_START_CLR();
	while(ADS_DATA_READY())
		;
	Ads_Config(ADC_GAIN_2,ADC_SPS_10);
#endif



	ADS_START_CLR();
	ADS_CHIP_SELECT_CLR();
	ADS_RESET_CLR();
	ADS_START();

	offset = 0;
	unsigned long off = 0;
	signed long ret;
	uint8_t a;
	for(a = 0; a < 16;)
	{
		while(ADS_DATA_READY()!=0)
			;
		ret = Ads_Read();
		if(ret!=0)
		{
			off += ret;
			a++;
			ret = 0;
		}
	}
	offset = off / a;


	MAP_IntEnable(INT_GPIOC);
	MAP_IntMasterEnable();

//	MAP_SysCtlPeripheralClockGating(false);

	MAP_GPIOIntTypeSet(ADS_DRY_PORT, ADS_DRY_PIN, GPIO_FALLING_EDGE);
	MAP_GPIOPinIntClear(ADS_DRY_PORT, ADS_DRY_PIN);
	MAP_GPIOPinIntEnable(ADS_DRY_PORT, ADS_DRY_PIN);
}


uint8_t spi_send0(uint8_t c) {
  unsigned long val;
  MAP_SSIDataPut(SSI0_BASE, c);
  MAP_SSIDataGet(SSI0_BASE, &val);
  return (uint8_t)val;
}


static void spi_init0(void) {
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  // Configure SSI1 for SPI RAM usage
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
  MAP_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
  MAP_GPIOPinConfigure(GPIO_PA4_SSI0RX);
  MAP_GPIOPinConfigure(GPIO_PA5_SSI0TX);
  MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_2);
  MAP_SSIConfigSetExpClk(SSI0_BASE, MAP_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			 SSI_MODE_MASTER, 1000000, 8);
  MAP_SSIEnable(SSI0_BASE);

  unsigned long b;
  while(MAP_SSIDataGetNonBlocking(SSI0_BASE, &b)) {}
}

//*****************************************************************************
// Delay for the specified number of seconds.  Depending upon the current
// SysTick value, the delay will be between N-1 and N seconds (i.e. N-1 full
// seconds are guaranteed, along with the remainder of the current second).
//*****************************************************************************
void Delay(unsigned long ulSeconds)
{
    // Loop while there are more seconds to wait.
    while(ulSeconds--)
    {
        // Wait until the SysTick value is less than 1000.
        while(MAP_SysTickValueGet() > 1000)
        {        }
        // Wait until the SysTick value is greater than 1000.
        while(MAP_SysTickValueGet() < 1000)
        {        }
    }
}
