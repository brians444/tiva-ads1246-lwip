/*
 * httpd_ssi_handler.c
 *
 *  Created on: 13/10/2014
 *      Author: Brian Schmidt
 */

#include "httpd_ssi_handler.h"
#include <stdio.h>
#include <utils/ustdlib.h>
#include "http.h"
#include "ads1146.h"

#if LWIP_HTTPD_SSI

//tSSIHandler ADC_Page_SSI_Handler;
uint32_t ADC_not_configured=1;

/* we will use character "t" as tag for CGI */
char const* TAGCHAR="t";
char const** TAGS=&TAGCHAR;

/**
  * @brief  ADC_Handler : SSI handler for ADC page
  */
u16_t ADC_Handler(int iIndex, char *pcInsert, int iInsertLen)
{
  /* We have only one SSI handler iIndex = 0 */
  if (iIndex ==0)
  {
     /* get ADC conversion value */
     //ADCVal = ADC_GetConversionValue(ADC3);

     /* get digits to display */
     //char string[6];

     /* prepare data to be inserted in html */
     /**pcInsert       = (char)(string[0]+0x30);
     *(pcInsert + 1) = (char)(string[1]+0x30);
     *(pcInsert + 2) = (char)(string[2]+0x30);
     *(pcInsert + 3) = (char)(string[3]+0x30);
     *(pcInsert + 4) = (char)(string[4]+0x30);
     *(pcInsert + 5) = (char)(string[5]+0x30);*/



	  /*
	   * Compruebo que no se exceda el tamaño del buffer
	   */
	  if(iInsertLen>string_size){
		  strncpy(pcInsert, stringNumber2, string_size);
		  return string_size;
	  }
	  else
	  {
		  strncpy(pcInsert, stringNumber2, iInsertLen);
		  return iInsertLen;
	  }

  }
  return 0;
}


/**
 * Initialize SSI handlers
 */
void httpd_ssi_init(void)
{
  /* configure SSI handlers (ADC page SSI) */
  string_size = strlen(stringNumber2);
  http_set_ssi_handler(ADC_Handler, (char const **)TAGS, 1);
}


#endif

/**
  * @brief  CGI handler for LEDs control
  */
#if LWIP_HTTPD_CGI

const char * LEDS_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
  uint32_t i=0;

  /* We have only one SSI handler iIndex = 0 */
  if (iIndex==0)
  {
    /* All leds off *//*
    STM_EVAL_LEDOff(LED1);
    STM_EVAL_LEDOff(LED2);
    STM_EVAL_LEDOff(LED3);
    STM_EVAL_LEDOff(LED4);*/

    /* Check cgi parameter : example GET /leds.cgi?led=2&led=4 */
    for (i=0; i<iNumParams; i++)
    {
      /* check parameter "led" */
      if (strcmp(pcParam[i] , "led")==0)
      {
        /* switch led1 ON if 1 */
        //if(strcmp(pcValue[i], "1") ==0)
          //STM_EVAL_LEDOn(LED1);

        /* switch led2 ON if 2 */
        //else if(strcmp(pcValue[i], "2") ==0)
          //STM_EVAL_LEDOn(LED2);

        /* switch led3 ON if 3 */
        //else if(strcmp(pcValue[i], "3") ==0)
          //STM_EVAL_LEDOn(LED3);

        /* switch led4 ON if 4 */
        //else if(strcmp(pcValue[i], "4") ==0)
          //STM_EVAL_LEDOn(LED4);
      }
    }
  }
  /* uri to send after cgi call*/
  return "/index.html";
}

/**
 * Initialize CGI handlers
 */
void httpd_cgi_init(void)
{
  /* configure CGI handlers (LEDs control CGI) */
  //CGI_TAB[0] = LEDS_CGI;
  //http_set_cgi_handlers(CGI_TAB, 1);
}

#endif
