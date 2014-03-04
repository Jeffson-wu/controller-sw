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

extern xQueueHandle ModbusQueueHandle;
extern xTimerHandle yTimer[];
xQueueHandle LogQueueHandle;

/* Private define ------------------------------------------------------------*/
#define NUM_OF_TUBES 16+1
#define LOG_QUEUE_SIZE 4
#define LOG_ELEMENT_SIZE 10

/* Private typedef -----------------------------------------------------------*/
typedef struct LOG_DATA_ELEMENT {
  u16 seqNum;
  u16 data[LOG_ELEMENT_SIZE];
} logDataElement_t;

typedef struct LOG_DATA_QUEUE {
  s16 head;  // ##### s8 is sufficient for head and tail  
  s16 tail;
  logDataElement_t logDataElement[LOG_QUEUE_SIZE];
  u32 padding; // ####debug padding
} logDataQueue_t;

/* Private macro -------------------------------------------------------------*/
//#define DEBUG /*General debug shows state changes of tubes (new temp, new time etc.)*/
#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_response(buf);
#else
#define DEBUG_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif
/* Private variables ---------------------------------------------------------*/
bool log_tubes[NUM_OF_TUBES]={FALSE};

static logDataQueue_t __attribute__ ((aligned (16))) logDataQueue[NUM_OF_TUBES]; // #### Aligned for debug

static char message[40];   /*buffer for printf*/

/* Private function prototypes -----------------------------------------------*/
void SERIAL_String(const char *string);
/* Private functions ---------------------------------------------------------*/

void SERIAL_String(const char *string)
{
#ifdef GDI_ON_USART3
    USART_TypeDef *uart = USART3;
#else
    USART_TypeDef *uart = USART1;
#endif
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
    logDataQueue[i].head = -1; 
    logDataQueue[i].tail = -1;
    // ####Fill in debug padding
    logDataQueue[i].padding = 0xA5A5A5A5;
  }
}

/* ---------------------------------------------------------------------------*/
// Data log is {{seq#Tube1,t1.1,t1.2,t1.3,t1.4,t1.5,t1.6,t1.7,t1.8,t1.9,t1.10},
//              {seq#Tube2,t2.1,t2.2,t2.3,t2.4,t2.5,t2.6,t2.7,t2.8,t2.9,t2.10}}
// Enqueue elament. Return pointer to element so it can be written to.
logDataElement_t * enqueue(logDataQueue_t * pQueue)
{
	if((pQueue->tail - LOG_QUEUE_SIZE) == pQueue->head) { return NULL; } // Return null if queue is full
	pQueue->tail++;
	return &pQueue->logDataElement[pQueue->tail % LOG_QUEUE_SIZE];
}

/* ---------------------------------------------------------------------------*/
// Dequeue elament. Return pointer to element so it can be read from.
logDataElement_t * dequeue(logDataQueue_t * pQueue)
{
	if(pQueue->tail == pQueue->head) {return '\0';}  // Return null if queue is empty
	else
	{
		pQueue->head++;
		return &pQueue->logDataElement[pQueue->head % LOG_QUEUE_SIZE];
	}
}

/* ---------------------------------------------------------------------------*/
int dataQueueAdd(u8 tubeId, u16 seqNumber, u8 data[])
{
  uint16_t modbus_data;
  int i;
  logDataElement_t * poutData;
  
  taskENTER_CRITICAL(); //push irq state
  if(NULL != (poutData = enqueue(&logDataQueue[tubeId-1]))) //tubeId=[1..16],idx=[0..15]
  {
    poutData->seqNum = seqNumber;
    for(i=0; i<LOG_ELEMENT_SIZE; i++)
    {
      poutData->data[i] = (((u16)(data[i*2])<<8)|(data[i*2+1]));
    }
  }
  taskEXIT_CRITICAL();
}

/* ---------------------------------------------------------------------------*/
/* at@gdi:seq_cmd(getlog)\n                                                   */
/* Response:                                                                  */
/* LOG:(tube1:<tempX>,<tempX>,<tempX>,<tempX>,<tempX>,<tempX>,<tempX>;        */
/*      tube2:<tempX>,<tempX>,<tempX>,<tempX>;tube12:<tempX>,<tempX>;)       */
/*                                                                            */
/* Where <tempX> is the logged temperature in deci C and printed in HEX       */ 
/* ---------------------------------------------------------------------------*/
void sendLog()
{
  int tubeId = 0;
  int i = 0;
  logDataElement_t * pinData;
  char str[20];

  SERIAL_String("LOG:(");
  for(tubeId = 1; tubeId < NUM_OF_TUBES; tubeId++) //tubeId=[1..16]
  {    
    taskENTER_CRITICAL(); //push irq state #### Kan critical section laves mindre!
    // Send all available log elements for each tube
    while(NULL != (pinData = dequeue(&logDataQueue[tubeId-1])) ) //idx=[0..15]
    {
      sprintf(str,"tube%d:",tubeId);
      SERIAL_String(str); 
      //? = pinData->seqNum;  Handle Seq# ?? Ignored for now
      for(i=0; i<LOG_ELEMENT_SIZE; i++)
      {
        sprintf(str,"%03X",pinData->data[i]); // 3 digits allows for temperatures up to 409,5 degrees
        SERIAL_String(str);
        if(LOG_ELEMENT_SIZE - 1 > i)
        { 
          SERIAL_String(","); 
        }
      }
      SERIAL_String(";");
    }
    taskEXIT_CRITICAL();
  }
  SERIAL_String(")");
}
/* Public functions ----------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
void vReadTubeTemp(xTimerHandle pxTimer )
{
  xMessage *msg;
  ReadModbusRegsReq *p;
  portBASE_TYPE taskWoken = pdTRUE;
  int tube;

  for(tube=1;tube<17;tube++)
  {
    if(log_tubes[tube]== TRUE)
    {
      msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq));
      msg->ucMessageID=READ_MODBUS_REGS;
      p=(ReadModbusRegsReq *)msg->ucData;
      if((tube%2) == 0)
      {
        p->addr=TUBE2_TEMP_REG;      
      }else
      {
        p->addr=TUBE1_TEMP_REG;
      }
      p->datasize=1;
      p->reply=LogQueueHandle;
      p->slave=tube/*+0x02*/;
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
  long TubeId,log_interval;
  int i = 1;
  static long last_tube_to_log;
  
#ifdef GDI_ON_USART3
  USART_TypeDef *uart = USART3;
#else
  USART_TypeDef *uart = USART1;
#endif

  initQueue();

  while(1)
  {
    if( xQueueReceive( LogQueueHandle, &msg, portMAX_DELAY) == pdPASS )
    {
      switch(msg->ucMessageID)
      {
      case START_LOG:
        i=1;
        TubeId = *((long *)(msg->ucData));
        while((i < NUM_OF_TUBES)&&(log_tubes[i++]== FALSE));/*Check if this is the first tube to enable log on then enable log timer*/
        last_tube_to_log = i-1;
        if(i >= NUM_OF_TUBES)
        {
          LogOn(1000);
          last_tube_to_log = 0; /*Init to 0 since first tube*/
        }
        log_tubes[TubeId]= TRUE;
        if (TubeId > last_tube_to_log) last_tube_to_log = TubeId;
      break;
      case END_LOG:
        i=1;
        TubeId = *((long *)(msg->ucData));
        log_tubes[TubeId]= FALSE;
        while((i < NUM_OF_TUBES)&&(log_tubes[i++]== FALSE));/*Check if this is last tube to log on, then disable log timer*/
        last_tube_to_log = i-1;
        if(i >= NUM_OF_TUBES)
        {
          LogOff();
        }
      break;
      case READ_MODBUS_REGS_RES:
        preg=(ReadModbusRegsRes *)msg->ucData;
        TubeId = preg->slave;
        modbus_addr = preg->addr;
        if(preg->resultOk == NO_ERROR) {
          if( (TUBE1_TEMP_REG == modbus_addr) || (TUBE2_TEMP_REG == modbus_addr) )
          {
            modbus_data =(((u16)(preg->data[0])<<8)|(preg->data[1]));
            sprintf(message,"T%d:%d.%01dC ",TubeId,modbus_data/10,modbus_data%10); //####sprintf(message,"T%d:%d.%01dC ",TubeId,dac_2_temp(modbus_data)/10,dac_2_temp(modbus_data)%10);

            for(i=0;i<strlen(message);i++)
            {
              while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
              USART_SendData(uart, *(message+i));
            }
            if(last_tube_to_log == TubeId) 
            {
              USART_SendData(uart, '\r');
              while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
              USART_SendData(uart, '\n');
              while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
            }
          }
          //#### handle tube 1 logs and tube 2 logs separately or jointly
          //  DATA_LOG_T1 = 50,
          //  DATALOG_T1_end = DATA_LOG_T1 + DATA_LOG_SIZE + 1,
          //  DATA_LOG_T2 = DATALOG_T1_end,
          //  DATALOG_T2_end = DATA_LOG_T2 + DATA_LOG_SIZE + 1,
          // Data log is {{seq#Tube1,t1.1,t1.2,t1.3,t1.4,t1.5,t1.6,t1.7,t1.8,t1.9,t1.10},
          //              {seq#Tube2,t2.1,t2.2,t2.3,t2.4,t2.5,t2.6,t2.7,t2.8,t2.9,t2.10}}
          else if((modbus_addr == DATA_LOG_T1) || (modbus_addr == DATA_LOG_T2))
          { // Auto Logging
            if( (modbus_addr == DATA_LOG_T1) && (DATA_LOG_SIZE*2 + 2 == preg->datasize) )
            { // Logs for tube 1 and tube 2 from that M0
            DEBUG_PRINTF("Enqueue both (%d)", TubeId);
              modbus_data =(((u16)(preg->data[0])<<8)|(preg->data[1]));   //Seq num1
              dataQueueAdd(TubeId,     modbus_data, &preg->data[2] );     //data after seq num
              modbus_data =(((u16)(preg->data[22])<<8)|(preg->data[23])); //Seq num2 
              dataQueueAdd(TubeId + 1, modbus_data, &preg->data[24]);     //data after seq num1 and 10 data and seq num2
            }

            if( (modbus_addr == DATA_LOG_T1) && (DATA_LOG_SIZE+1) == preg->datasize)
            { // Logs for tube 1 only from that M0
              DEBUG_PRINTF("Enqueue T1 (%d)", TubeId);
              modbus_data =(((u16)(preg->data[0])<<8)|(preg->data[1])); //Seq num
              dataQueueAdd(TubeId, modbus_data, &preg->data[2] );       //data after seq num
            }

            if( (modbus_addr == DATA_LOG_T2) && (DATA_LOG_SIZE+1) == preg->datasize)
            { // Logs for tube 2 only from that M0
              DEBUG_PRINTF("Enqueue T2 (%d)", TubeId + 1);
              modbus_data =(((u16)(preg->data[0])<<8)|(preg->data[1])); //Seq num
              dataQueueAdd(TubeId + 1, modbus_data, &preg->data[2] );   //data after seq num
            }
          }
          else
          {
            DEBUG_PRINTF("Tube[%d]ERROR MODBUS read log FAILED!!! %d",TubeId,preg->resultOk);
          }
        }
      break;
      case SET_LOG_INTERVAL:
        log_interval = *((long *)(msg->ucData));
        if(0 == log_interval) {
          LogOff();
        } else {
          LogOn(log_interval);
        }
        if(xTimerChangePeriod( yTimer[2],1 * log_interval,100)!= pdPASS ) {
          //error handling
        }
      break;
      }
    }
    vPortFree(msg);
  }
}

