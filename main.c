#include "main.h"

static volatile unsigned long g_ulFlags;

volatile unsigned long g_ulTickCounter = 0;

volatile unsigned long g_ulSysTickCount;

const uint8_t mac_addr[] = { 0x00, 0xC0, 0x033, 0x50, 0x48, 0x12 };



int main(void) {
  cpu_init();
  uart_init();
  systick_init();
  AdsPinConfig();
  spi_init();

  init_ethernet();

  httpd_init();



  last_arp_time = last_tcp_time = 0;

  while(1) {
    //MAP_SysCtlSleep();

    task_lwip();
    task_enc();
    task_ads();

  }

  return 0;
}


void task_ads()
{
	if(HWREGBITW(&g_ulFlags, FLAG_ADS_INT) == 1)
	{
		HWREGBITW(&g_ulFlags, FLAG_ADS_INT) = 0;

		ADS_START_CLR();
		Ads_Read();
		ADS_START();

		ROM_GPIOPinIntClear(ADS_DRY_PORT, 0xff);
		MAP_IntEnable(INT_GPIOC);
		MAP_GPIOPinIntEnable(ADS_DRY_PORT, ADS_DRY_PIN);
	}
}

void task_lwip(void)
{
	if(HWREGBITW(&g_ulFlags, FLAG_SYSTICK) == 1) {
	      HWREGBITW(&g_ulFlags, FLAG_SYSTICK) = 0;

	      if( (g_ulTickCounter - last_arp_time) * TICK_MS >= ARP_TMR_INTERVAL) {
	    	  etharp_tmr();
	    	  last_arp_time = g_ulTickCounter;
	      }

	      if( (g_ulTickCounter - last_tcp_time) * TICK_MS >= TCP_TMR_INTERVAL) {
	    	  tcp_tmr();
	    	  last_tcp_time = g_ulTickCounter;
	      }

	      if( (g_ulTickCounter - last_dhcp_coarse_time) * TICK_MS >= DHCP_COARSE_TIMER_MSECS) {
		dhcp_coarse_tmr();
		last_dhcp_coarse_time = g_ulTickCounter;
	      }

	      if( (g_ulTickCounter - last_dhcp_fine_time) * TICK_MS >= DHCP_FINE_TIMER_MSECS) {
		dhcp_fine_tmr();
		last_dhcp_fine_time = g_ulTickCounter;
	      }
	    }


}

void task_enc(void)
{
	if( HWREGBITW(&g_ulFlags, FLAG_ENC_INT) == 1 )
	{
		HWREGBITW(&g_ulFlags, FLAG_ENC_INT) = 0;
		enc_action(&netif_g);
	}
}

void init_ethernet(void)
{
	  enc28j60_comm_init();
	  UARTprintf("Welcome\n");

	  enc_init(mac_addr);
	  systick_init();

	  lwip_init();

	#if !LWIP_DHCP
	  IP4_ADDR(&gw_g, 10,0,0,1);
	  IP4_ADDR(&ipaddr_g, 10,0,0,100);
	  IP4_ADDR(&netmask_g, 255, 255, 255, 0)
	#else
	  IP4_ADDR(&gw_g, 0,0,0,0);
	  IP4_ADDR(&ipaddr_g, 0,0,0,0);
	  IP4_ADDR(&netmask_g, 0, 0, 0, 0);
	#endif

	  netif_add(&netif_g, &ipaddr_g, &netmask_g, &gw_g, NULL, enc28j60_init, ethernet_input);
	  netif_set_default(&netif_g);

	#if !LWIP_DHCP
	  netif_set_up(&netif_g);
	#else
	  dhcp_start(&netif_g);
	#endif

}

void systick_init()
{
  //
  // Configure SysTick for a periodic interrupt.
  //
  MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / SYSTICKHZ);
  MAP_SysTickEnable();
  MAP_SysTickIntEnable();

  //MAP_IntEnable(INT_GPIOA);
  MAP_IntEnable(INT_GPIOE);
  MAP_IntMasterEnable();

  MAP_SysCtlPeripheralClockGating(false);

  MAP_GPIOIntTypeSet(GPIO_PORTE_BASE, ENC_INT, GPIO_FALLING_EDGE);
  MAP_GPIOPinIntClear(GPIO_PORTE_BASE, ENC_INT);
  MAP_GPIOPinIntEnable(GPIO_PORTE_BASE, ENC_INT);
  UARTprintf("int enabled\n");
}

uint8_t spi_send(uint8_t c) {
  unsigned long val;
  MAP_SSIDataPut(SSI2_BASE, c);
  MAP_SSIDataGet(SSI2_BASE, &val);
  return (uint8_t)val;
}


void
SysTickIntHandler(void)
{
    //
    // Increment the system tick count.
    //
    g_ulTickCounter++;
    g_ulSysTickCount++;
    //
    // Indicate that a SysTick interrupt has occurred.
    //
    HWREGBITW(&g_ulFlags, FLAG_SYSTICK) = 1;
    //g_ulFlags |= FLAG_SYSTICK;
}


void GPIOPortEIntHandler(void) {
  uint8_t p = MAP_GPIOPinIntStatus(GPIO_PORTE_BASE, true) & 0xFF;

  MAP_GPIOPinIntClear(GPIO_PORTE_BASE, p);

  HWREGBITW(&g_ulFlags, FLAG_ENC_INT) = 1;
}

static void cpu_init(void) {
	g_ulSysTickCount = 0;
		// Enable lazy stacking for interrupt handlers.  This allows floating-point
			// instructions to be used within interrupt handlers, but at the expense of extra stack usage.
	ROM_FPULazyStackingEnable();

  // A safety loop in order to interrupt the MCU before setting the clock (wrongly)
  int i;
  for(i=0; i<1000000; i++);

  // Setup for 16MHZ external crystal, use 200MHz PLL and divide by 4 = 50MHz
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_16 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
      SYSCTL_XTAL_16MHZ);
}

static void uart_init(void) {
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  // Configure PD0 and PD1 for UART
  MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
  MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  /*UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                      UART_CONFIG_WLEN_8| UART_CONFIG_STOP_ONE| UART_CONFIG_PAR_NONE);*/
  UARTStdioInitExpClk(0, 115200);
}

static void spi_init(void) {
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  // Configure SSI1 for SPI RAM usage
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
  MAP_GPIOPinConfigure(GPIO_PB4_SSI2CLK);
  MAP_GPIOPinConfigure(GPIO_PB6_SSI2RX);
  MAP_GPIOPinConfigure(GPIO_PB7_SSI2TX);
  MAP_GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);
  MAP_SSIConfigSetExpClk(SSI2_BASE, MAP_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			 SSI_MODE_MASTER, 1000000, 8);
  MAP_SSIEnable(SSI2_BASE);

  unsigned long b;
  while(MAP_SSIDataGetNonBlocking(SSI2_BASE, &b)) {}
}

static void enc28j60_comm_init(void) {
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, ENC_CS);
  MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, ENC_INT);

  MAP_GPIOPinWrite(ENC_CS_PORT, ENC_CS, ENC_CS);
}

void DataReadyIntHandler(void)
{
	uint8_t p = ROM_GPIOPinIntStatus(ADS_DRY_PORT, true) & 0xFF;

	MAP_IntDisable(INT_GPIOC);
	MAP_GPIOPinIntDisable(ADS_DRY_PORT, ADS_DRY_PIN);

	GPIOPinIntClear(ADS_DRY_PORT, p);

	HWREGBITW(&g_ulFlags, FLAG_ADS_INT) = 1;

}


