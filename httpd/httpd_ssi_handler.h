/*
 * httpd_ssi_handler.h
 *
 *  Created on: 13/10/2014
 *      Author: Vegito
 */

#ifndef HTTPD_SSI_HANDLER_H_
#define HTTPD_SSI_HANDLER_H_

#include "lwipopts.h"

#include "lwip/debug.h"
#include "http.h"
#include "lwip/tcp.h"
//#include "fs.h"

#include <string.h>
#include <stdlib.h>

#if LWIP_HTTPD_SSI


void httpd_ssi_init(void);
uint16_t ADC_Handler(int iIndex, char *pcInsert, int iInsertLen);

#endif

#if LWIP_HTTPD_CGI

/* CGI handler for LED control */
//const char * LEDS_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);

/* Html request for "/leds.cgi" will start LEDS_CGI_Handler */
//const tCGI LEDS_CGI={"/leds.cgi", LEDS_CGI_Handler};

/* Cgi call table, only one CGI used */
//tCGI CGI_TAB[1];

#endif
#endif /* HTTPD_SSI_HANDLER_H_ */
