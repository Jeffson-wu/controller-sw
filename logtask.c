/**
  ******************************************************************************
  * @file    logtask.c
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    10-Feb - 2014
  * @brief   Temperature logging task
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 Xtel </center></h2>
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "stm3210c-eval.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "signals.h"
#include "timers.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"
#include "../heater-sw/heater_reg.h"
#include "debug.h"
#include "logtask.h"
#include "util.h"

/* Private feature defines ---------------------------------------------------*/

/* Private debug define ------------------------------------------------------*/

xQueueHandle LogQueueHandle;

void LogOn(int log_time);
void LogOff();

/* Private define ------------------------------------------------------------*/
#define NUM_OF_TUBES 16+1
#define LOG_QUEUE_SIZE 4 /*valid values for this is 2,4,8,16,....*/
#define LOG_ELEMENT_SIZE (DATA_LOG_SIZE/2)
#define MAX_LOG_DATA_REQUESTED 20 /*Two chunks of log data*/
typedef struct ldata {
  u16 stage_num;
  u16 temp;
}ldata_t;

/* Private typedef -----------------------------------------------------------*/
typedef struct LOG_DATA_ELEMENT {
  u16 seqNum;
  ldata_t ldata[LOG_ELEMENT_SIZE];
} logDataElement_t;

typedef struct LOG_DATA_QUEUE {
  u8 head;  // ##### u8 is sufficient for head and tail  
  u8 tail;
  logDataElement_t logDataElement[LOG_QUEUE_SIZE];
  u32 padding; // ####debug padding
} logDataQueue_t;

/* Private macro -------------------------------------------------------------*/
//#define DEBUG /*General debug shows state changes of tubes (new temp, new time etc.)*/
#ifdef DEBUG
#define DEBUG_LOG_PRINTF(fmt, args...)      sprintf(dbgbuf, fmt, ## args);  send_msg_on_monitor(dbgbuf);
#else
#define DEBUG_LOG_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif
/* Private variables ---------------------------------------------------------*/
bool log_tubes[NUM_OF_TUBES]={FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE};

static logDataQueue_t __attribute__ ((aligned (16))) logDataQueue[NUM_OF_TUBES]; // #### Aligned for debug

int logcount[NUM_OF_TUBES]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
u8 cur_tubeid = 0;
/* Private function prototypes -----------------------------------------------*/
void SERIAL_String(const char *string);

/* Private functions ---------------------------------------------------------*/
void SERIAL_String(const char *string)
{
  USART_TypeDef *uart = USART1;
  if (string) {
    while (*string != '\0') {
      USART_SendData(uart, *(string++));
      while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
    }
  }
}

/* ---------------------------------------------------------------------------*/
void initQueue()
{
  int i;
  for(i=0; NUM_OF_TUBES > i; i++)
  {
    logDataQueue[i].head = 0; 
    logDataQueue[i].tail = 0;
    // ####Fill in debug padding
    logDataQueue[i].padding = 0xA5A5A5A5;
  }
}

/* ---------------------------------------------------------------------------*/
// Data log is {seq#,stage#,t1,stage#,t2,stage#,t3,stage#,t4,stage#,t5,stage#,t6,stage#,t7,stage#,t8,stage#,t9,stage#,t10}
// Enqueue elament. Return pointer to element so it can be written.
logDataElement_t * enqueue(logDataQueue_t * pQueue)
{
#ifdef DEBUG
  int h, t;
#endif
  logDataElement_t * pElement;

  taskENTER_CRITICAL(); //push irq state
  if((pQueue->tail - LOG_QUEUE_SIZE) == pQueue->head) { pElement = NULL; } // Return null if queue is full
  else {
    pQueue->tail++;
    pElement = &pQueue->logDataElement[pQueue->tail % LOG_QUEUE_SIZE];
  }
#ifdef DEBUG
  h = pQueue->head; 
  t = pQueue->tail;
#endif
  taskEXIT_CRITICAL();
  DEBUG_LOG_PRINTF("T%d:enqueue: head %d tail %d",cur_tubeid, h, t);
  return pElement;
}

/* ---------------------------------------------------------------------------*/
// Dequeue elrment. Return pointer to element so it can be read.
logDataElement_t * dequeue(logDataQueue_t * pQueue)
{
#ifdef DEBUG
    int h, t;
#endif
  logDataElement_t * pElement;

  taskENTER_CRITICAL(); //push irq state
  if(pQueue->tail == pQueue->head) {pElement = NULL; } // Return null if queue is empty
  else {
    pQueue->head++;
    pElement = &pQueue->logDataElement[pQueue->head % LOG_QUEUE_SIZE];
  }
#ifdef DEBUG
  h = pQueue->head; 
  t = pQueue->tail;
#endif
  taskEXIT_CRITICAL();
  DEBUG_LOG_PRINTF("T%d:dequeue: head %d tail %d",cur_tubeid, h, t);
  return pElement;
}

/* ---------------------------------------------------------------------------*/
void dataQueueAdd(u8 tubeId, u16 seqNumber, u8 data[])
{
  cur_tubeid = tubeId;
  int i;
  logDataElement_t * poutData=0;
  if(tubeId<=16 && tubeId>=1)
  {
    poutData = enqueue(&logDataQueue[tubeId-1]); //tubeId=[1..16],idx=[0..15]
  }
  if(NULL != poutData)
  {
    DEBUG_LOG_PRINTF("dataQueueAdd @0x%08X", poutData);
    poutData->seqNum = seqNumber;
    for(i = 0; i < LOG_ELEMENT_SIZE; i ++)
    {
      poutData->ldata[i].stage_num = (((u16)(data[i*4])<<8)  |(data[i*4+1]));
      poutData->ldata[i].temp =      (((u16)(data[i*4+2])<<8)|(data[i*4+3]));
      //DEBUG_LOG_PRINTF("dataQueueAdd Tube %d - stage %d, %d", tubeId,poutData->ldata[i].stage_num,poutData->ldata[i].temp);
    }
  }
  else
  {
    DEBUG_LOG_PRINTF("T%d:dataQueueAdd - buffer full",tubeId);
  }
  
}

/* ---------------------------------------------------------------------------*/
void emptyLog(int tubeId)
{
  while(NULL != dequeue(&logDataQueue[tubeId-1]));  //idx=[0..15]
}

/* ---------------------------------------------------------------------------*/
int addLog(char *poutText,int tubeId, int size )
{
  int i = 0;
  int nElements = 0;
  cur_tubeid = tubeId;
  logDataElement_t * pinData;
  char str[20];
  int dataAdded = 0;
  int counter=0;
  
  // Send all available log elements for each tube
  while( (counter < MAX_LOG_DATA_REQUESTED/LOG_ELEMENT_SIZE) && (NULL != (pinData = dequeue(&logDataQueue[tubeId-1]))) ) //idx=[0..15]
  {
    counter++;
    dataAdded = 1;
    if(nElements == 0)
    {
      addStrToBuf(poutText,";log={", size);
      Itoa(pinData->seqNum*LOG_ELEMENT_SIZE, str); /*total number of log elements*/
      addStrToBuf(poutText,str, size);
      addStrToBuf(poutText,",", size);
    }
    for(i=0; i<LOG_ELEMENT_SIZE; i++)
    {
      int templen=0;
      Itoa(pinData->ldata[i].stage_num, str);
      templen = strlen(str);
      str[templen]=',';
      Itoa(pinData->ldata[i].temp, &str[templen+1]);
      addStrToBuf(poutText, str, size);
      if(LOG_ELEMENT_SIZE - 1 > i)
      { 
        addStrToBuf(poutText, ",", size);
      }
    }
    addStrToBuf(poutText, ",", size); 
    nElements+=LOG_ELEMENT_SIZE;

  }
  
  if(dataAdded) { 
    poutText[strlen(poutText)-1]=0;
    addStrToBuf(poutText, "}",size);
  } 
  logcount[tubeId] = logcount[tubeId] + nElements; 
  DEBUG_LOG_PRINTF("Lenght of log %d",strlen(poutText));
  return nElements;
}

/* ---------------------------------------------------------------------------*/
/* Development logging */
void sendLog()
{
  int tubeId = 0;
  int i = 0;
  logDataElement_t * pinData;
  char str[20];

  SERIAL_String("LOG:(");
  for(tubeId = 1; tubeId < NUM_OF_TUBES; tubeId++) //tubeId=[1..16]
  {    
    pinData = dequeue(&logDataQueue[tubeId-1]); //idx=[0..15]
    // Send all available log elements for each tube
    while(NULL != pinData )
    {
      sprintf(str,"tube%d:",tubeId);
      
      SERIAL_String(str); 
      //? = pinData->seqNum;  Handle Seq# ?? Ignored for now
       sprintf(str,"%03X,",pinData->seqNum); // 3 digits allows for temperatures up to 409,5 degrees
      for(i=0; i<LOG_ELEMENT_SIZE; i++)
      {
        sprintf(str,"%03X",pinData->ldata[i].temp); // 3 digits allows for temperatures up to 409,5 degrees
        SERIAL_String(str);
        if(LOG_ELEMENT_SIZE - 1 > i)
        { 
          SERIAL_String(","); 
        }
      }
      SERIAL_String(";");
    }
  }
  SERIAL_String(")");
}
/* Public functions ----------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
void vReadTubeTemp(xTimerHandle pxTimer )
{
  xMessage *msg;
  ReadModbusRegsReq *p;
  int tube;

  for(tube = 1; tube < 17; tube++)
  {
    if(log_tubes[tube]== TRUE)
    {
      msg = pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq));
      msg->ucMessageID = READ_MODBUS_REGS;
      p = (ReadModbusRegsReq *)msg->ucData;
      if((tube%4) == 0)
      {
        p->addr=TUBE4_TEMP_REG;      
      }else if((tube%4) == 1)
      {
        p->addr=TUBE3_TEMP_REG;
      }else if((tube%4) == 2)
      {
        p->addr=TUBE2_TEMP_REG;
      }else if((tube%4) == 3)
      {
        p->addr=TUBE1_TEMP_REG;
      }
      p->datasize = 1;
      p->reply = LogQueueHandle;
      p->slave = tube;
      xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
    }
  }
}

/* ---------------------------------------------------------------------------*/
void LogTask( void * pvParameters )
{
  ReadModbusRegsRes *preg;
  uint16_t modbus_data;
  uint16_t modbus_addr;
  xMessage *msg;
  long TubeId;
  initQueue();

  while(1)
  {
    if( xQueueReceive( LogQueueHandle, &msg, portMAX_DELAY) == pdPASS )
    {
      switch(msg->ucMessageID)
      {
      case READ_MODBUS_REGS_RES:
        preg=(ReadModbusRegsRes *)msg->ucData;
        TubeId = preg->slave;
        modbus_addr = preg->addr;
        if(preg->resultOk == NO_ERROR) {
          if(DATA_LOG == modbus_addr)
          { // Auto Logging
            // Data log is {seq#,stage#,t1,stage#,t2,stage#,t3,stage#,t4,stage#,t5,stage#,t6,stage#,t7,stage#,t8,stage#,t9,stage#,t10}
            modbus_data =(((u16)(preg->data[0])<<8)|(preg->data[1])); //Seq num
            dataQueueAdd(TubeId, modbus_data, &preg->data[2] );       //data after seq num
          }
          else
          {
            DEBUG_LOG_PRINTF("Tube[%d]ERROR MODBUS read log FAILED!!! %d",TubeId,preg->resultOk);
          }
        }
      break;
      }
    }
   // DEBUG_LOG_PRINTF("msg done");
    vPortFree(msg);
  }
}

