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
#include "debug.h"
/* ---------------------------------------------------------------------------*/
/* Global Debug variables ----------------------------------------------------*/
int dbgActiveISRid = 0;
int dbgLastActiveISRid = 0;

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
  sprintf(buf, "Active ISR: %d LastActive ISR: %d\n", dbgActiveISRid, dbgLastActiveISRid);
  return buf;
}

