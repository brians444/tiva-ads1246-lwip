/*
 * main.h
 *
 *  Created on: 15/10/2014
 *      Author: Vegito
 */

#ifndef MAIN_H_
#define MAIN_H_


#include <inc/hw_ints.h>
#include <stdint.h>
#include "common.h"
#include "enc28j60.h"
#include "spi.h"
#include <driverlib/systick.h>
#include <driverlib/interrupt.h>

#include <lwip/init.h>
#include <lwip/netif.h>
#include <lwip/dhcp.h>
#include <lwip/tcp_impl.h>
#include <netif/etharp.h>

#include <utils/uartstdio.h>

#include "ads1146.h"
#include "http.h"

#define FLAG_SYSTICK		0
#define FLAG_RXPKT		1
#define FLAG_TXPKT		2
#define FLAG_RXPKTPEND		3
#define FLAG_ENC_INT		4
#define FLAG_ADS_INT		5

#define TICK_MS			250
#define SYSTICKHZ		(1000/TICK_MS)


struct netif netif_g;
ip_addr_t ipaddr_g, netmask_g, gw_g;
unsigned long last_arp_time, last_tcp_time, last_dhcp_coarse_time, last_dhcp_fine_time;

static void cpu_init(void);
static void uart_init(void);
static void spi_init(void);
static void enc28j60_comm_init(void);
void systick_init();

void init_ethernet(void);
void task_lwip(void);
void task_enc(void);
void task_ads(void);
void DataReadyIntHandler(void);





#endif /* MAIN_H_ */
