/**
  ******************************************************************************
  * @file    debug.c
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    19 - May - 2015
  * @brief   CDebug functionality
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
#include <stdio.h>
#include <string.h>
#include "serial.h"
#include "debug.h"
/* ---------------------------------------------------------------------------*/
/* Global Debug variables ----------------------------------------------------*/
int dbgActiveISRid = 0;
int dbgLastActiveISRid = 0;
char dbgbuf[300]; /*buffer for debug printf*/

/* Private feature defines ---------------------------------------------------*/
/* Private debug defines -----------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* Private prototypes                                                         */
/* ---------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/* functions                                                                  */
/* ---------------------------------------------------------------------------*/



/* ---------------------------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
inline void dbgTraceStoreISRBegin(int id)
{
  dbgActiveISRid = id;
}

inline void dbgTraceStoreISREnd()
{
  dbgLastActiveISRid = dbgActiveISRid;
  dbgActiveISRid = 0;
}

char *dbgPrintIsr(char *buf)
{
  sprintf(dbgbuf, "Active ISR: %d LastActive ISR: %d\n", dbgActiveISRid, dbgLastActiveISRid);
  return dbgbuf;
}

/* ---------------------------------------------------------------------------*/
void send_msg_on_monitor(char * response)
{
  if(USART3_intitalized)
  {
    char i = 0;
    int len = strlen(response)+3;
    char message[strlen(response)+3];
    strcpy(message, "\0");
    strcat(message, response);
    strcat(message, "\r\n");
    while(i<len)
    {
      while(USART_GetFlagStatus(USART3, USART_FLAG_TXE)==RESET);
      USART_SendData(USART3,*(message+i));
      i++;
    }
  }
}

/* ---------------------------------------------------------------------------*/
void printHeap(void) {
  extern size_t xFreeBytesRemaining;
  sprintf(dbgbuf, "Heap free bytes: %d", xFreeBytesRemaining);
  send_msg_on_monitor(dbgbuf);
}


