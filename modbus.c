/**
  ******************************************************************************
  * @file    Demo/src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "stm3210c-eval.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "signals.h"
#include "timers.h"
#include "serial.h"
#include "debug.h"

/* ---------------------------------------------------------------------------*/
#define CHARS_IN_FRAME 10 /*1 startbit, 8 data, 1 stopbit*/
#define CHARS_TO_WAIT 3.5 /* Silence on modbus to detect end of telegram*/
#define USECS_2_NUM 1000000
#define MODBUS_SILENT_INTERVAL (CHARS_TO_WAIT *(CHARS_IN_FRAME * USECS_2_NUM / 115200))

#define MODBUS_RESPONSE_TIMEOUT 100 /*Time to wait for response in ms*/

#define READ_HOLDINGS_REGISTERS 3
#define WRITE_MULTIPLE_REGISTERS 16
#define  RS485_RX_LED GPIOB,GPIO_Pin_0
/* Private feature defines ---------------------------------------------------*/

/* Private debug define ------------------------------------------------------*/
//#define DEBUG_MB /* General debug */
//#define DEBUG_HF /* Searching for a hard fault */
extern void gdi_send_msg_on_monitor(char * response);
#define DEBUG_BUFFER_SIZE 600
char buf[DEBUG_BUFFER_SIZE];                   /* buffer for debug printf*/
#ifdef DEBUG_MB
#define DEBUG_MB_PRINTF(fmt, args...)      snprintf(buf, DEBUG_BUFFER_SIZE, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_MB_PRINTF(fmt, args...)          /* Don't do anything in release builds */
#endif
#ifdef DEBUG_HF
#define DEBUG_HF_PRINTF(fmt, args...)      snprintf(buf, DEBUG_BUFFER_SIZE, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_HF_PRINTF(fmt, args...)          /* Don't do anything in release builds */
#endif

USART_ERROR USART2_ERROR = 0;


static USART_TypeDef *usedUart;
typedef struct
{
int chars;
int buf;
}rx_debug;

rx_debug debug[200];
int debug_cnt = 0;

void UART_SendMsg(USART_TypeDef *uart, u8 *buffer, int len);
void Modbus_init(USART_TypeDef *uart);

/* ---------------------------------------------------------------------------*/
xSemaphoreHandle xModbusSemaphore = NULL;

xQueueHandle ModbusQueueHandle;
static  xTimerHandle TimerHandle;

static  uint8_t *recvBuffer=0;
static int NOFRecvChars=0;
static int recvBufferSize;


/* Defined in main.c. */
void configTimerModbusTimeout( void )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint32_t TimerPeriod_TIM6 = 0;
  uint32_t TIM6_pwm_freq = 1000000;/*Make uS tics*/
  TimerPeriod_TIM6 = (SystemCoreClock / TIM6_pwm_freq ) - 1;
  TimerPeriod_TIM6 = TimerPeriod_TIM6 * MODBUS_SILENT_INTERVAL;

 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);
  TIM_Cmd(TIM6, DISABLE);
  TIM_SetCounter(TIM6,0x00);

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod_TIM6;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0D   /*0x0B - 0x0F */;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; /*dont care*/
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

}

static uint16_t gen_crc16(const uint8_t *buf, int len)
{
  uint16_t crc = 0xFFFF;
  int pos, i;
 
  for (pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
 
    for (i = 8; i != 0; i--) {          // Loop over each bit
      if ((crc & 0x0001) != 0) {        // If the LSB is set
        crc >>= 1;                      // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                              // Else LSB is not set
        crc >>= 1;                      // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;  
}

void modbus_end_of_telegram_Handler()
{/*end of telegram 3.5 chars of silence*/
    portBASE_TYPE xHigherPriorityTaskWoken;
    dbgTraceStoreISRBegin(TRACE_ISR_ID_MB_EOT);

  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
  {
      TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
      TIM_Cmd(TIM6, DISABLE);
    /*Stop updating recvBuffer*/
    recvBuffer=0; //=(uint8_t *)0;
    /*unlock mutex to continue in modbus task*/
    DEBUG_MB_PRINTF("MB-SemGive-Rsp");
    xSemaphoreGiveFromISR( xModbusSemaphore, &xHigherPriorityTaskWoken );
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  }
  dbgTraceStoreISREnd();
}

void response_timeoutCB(xTimerHandle handle)
{/* response telegram did not arrive*/
  if(recvBuffer && (NOFRecvChars==0) )
    {
        USART2_ERROR = TIMEOUT;
      /*Stop updating recvBuffer*/
      recvBuffer=0; //=(uint8_t *)0;
      /*unlock mutex to continue in modbus task*/
      DEBUG_MB_PRINTF("MB-SemGive-Timeout");
      xSemaphoreGive( xModbusSemaphore );
    }
}

void recieveCharISR_CB(void)
{
  while(USART_GetITStatus(USART2, USART_IT_RXNE))
  {
    /*If recvBuffer has a pointer we have  modbus read or write function in progess else discard the charaters*/
    if(recvBuffer && NOFRecvChars<recvBufferSize)
    {
      recvBuffer[NOFRecvChars]=USART_ReceiveData(usedUart);
      NOFRecvChars++;
      if(NOFRecvChars == 1) /*First character recieved start end of telegram timer*/
      {
        TIM_Cmd(TIM6, ENABLE);
      }
      TIM_SetCounter(TIM6,0x00); /*Reset end of telegram timer*/
    }
    else
    {
      USART_ReceiveData(usedUart);
    }
    if(USART_GetFlagStatus(USART2,USART_FLAG_ORE))
    {
      USART_ClearFlag(USART2, USART_IT_ORE);
      USART2_ERROR = OVERRUN;
    }
  }
#ifdef DEBUG_HF
     /*     @arg USART_FLAG_CTS:  CTS Change flag (not available for UART4 and UART5)
      *     @arg USART_FLAG_LBD:  LIN Break detection flag
      *     @arg USART_FLAG_TXE:  Transmit data register empty flag
      *     @arg USART_FLAG_TC:   Transmission Complete flag
      *     @arg USART_FLAG_RXNE: Receive data register not empty flag
      *     @arg USART_FLAG_IDLE: Idle Line detection flag
      *     @arg USART_FLAG_ORE:  OverRun Error flag
      *     @arg USART_FLAG_NE:   Noise Error flag
      *     @arg USART_FLAG_FE:   Framing Error flag
      *     @arg USART_FLAG_PE:   Parity Error flag*/
      //if(USART_GetFlagStatus(USART2, USART_FLAG_LBD))  { DEBUG_HF_PRINTF("-#-LDB");  USART_ClearFlag(USART2, USART_FLAG_LBD);  }
      if(USART_GetFlagStatus(USART2, USART_FLAG_IDLE)) { DEBUG_HF_PRINTF("-#-IDLE"); }
      if(USART_GetFlagStatus(USART2, USART_FLAG_ORE))  { DEBUG_HF_PRINTF("-#-ORE");  }
      //if(USART_GetFlagStatus(USART2, USART_FLAG_NE))   { DEBUG_HF_PRINTF("-#-NE");   }
      //if(USART_GetFlagStatus(USART2, USART_FLAG_FE))   { DEBUG_HF_PRINTF("-#-FE");   }
      //if(USART_GetFlagStatus(USART2, USART_FLAG_PE))   { DEBUG_HF_PRINTF("-#-PE");   }
#endif
 //portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

USART_ERROR waitForRespons(u8 *telegram, int *telegramSize)
{
  USART_ERROR modbus_err;
  debug[debug_cnt].chars = 0xEE;
  
  /*start response telegram timer,*/
  xTimerReset( TimerHandle, 0 );
  xTimerStart( TimerHandle, 0 );
  NOFRecvChars=0;
  debug[debug_cnt].chars = 0xFF;
  /*wait for mutex*/
  DEBUG_MB_PRINTF("MB-SemTake");
  xSemaphoreTake( xModbusSemaphore, portMAX_DELAY );
  DEBUG_MB_PRINTF("MB-GotSem.");
  DEBUG_HF_PRINTF("* RChr: %d", NOFRecvChars);
  xTimerStop( TimerHandle, 0 ); 
  modbus_err = USART2_ERROR;
  /*handle modbus telegram*/
  if(NOFRecvChars && (modbus_err == NO_ERROR))
  {
    *telegramSize=NOFRecvChars;
    modbus_err = NO_ERROR;
  }
  else
  {
    *telegramSize=0;
  }
  recvBuffer=0;
  USART2_ERROR = NO_ERROR;
  return modbus_err;
}

/* datasize is reg count */
static USART_ERROR ModbusReadRegs(u8 slave, u16 addr, u16 datasize, u8 *buffer)
{
  u16 crc;
  USART_ERROR ERR;
  u8 telegram[256];
  int telegramsize=256;
  telegram[0]=slave;
  telegram[1]=READ_HOLDINGS_REGISTERS;
  telegram[2]=(addr & 0xFF00)>>8;
  telegram[3]=addr & 0x00FF;
  telegram[4]=(datasize & 0xFF00)>>8;
  telegram[5]=datasize & 0x00FF;
  crc = gen_crc16(&telegram[0], 6);
  memcpy(&(telegram[6]), &crc, (size_t)2);
  recvBuffer=telegram;
  recvBufferSize=telegramsize;
  debug[debug_cnt].chars = 0xAA;
  UART_SendMsg(usedUart, telegram, 6+2);
  DEBUG_MB_PRINTF("MBR->");
  ERR = waitForRespons(telegram, &telegramsize);
 // DEBUG_HF_PRINTF("* TelRRS:%d", telegramsize);
  DEBUG_MB_PRINTF("MBR<-");
  if(ERR != NO_ERROR)
  {
    return ERR;
  }
  memcpy(buffer, telegram, datasize*sizeof(u16)>telegramsize?telegramsize:datasize*sizeof(u16));
  if(telegramsize-5<=0)
  {
    return WRONG_TEL_LENGHT;
  }
  else
  {
     if(buffer)
     {
       memcpy(buffer, &telegram[3], telegramsize-5);
     }
     return NO_ERROR;
  }
}

/* datasize is reg count */
static USART_ERROR ModbusWriteRegs(u8 slave, u16 addr, u8 *data, u16 datasize)
{
  u16 crc;
  u8 telegram[256];
  int telegramsize=256;
  USART_ERROR ERR;
  telegram[0]=slave;
  telegram[1]=WRITE_MULTIPLE_REGISTERS;/*write multiple regs*/
  telegram[2]=(addr & 0xFF00)>>8;
  telegram[3]=addr & 0x00FF;
  telegram[4]=(datasize & 0xFF00)>>8;
  telegram[5]=datasize & 0x00FF;
  telegram[6]=datasize*2;
  memcpy(&telegram[7], data, datasize*2);
  crc = gen_crc16(&telegram[0], 7+datasize*2);
  memcpy(&(telegram[7+datasize*2]), &crc, 2);
  recvBuffer=telegram;
  recvBufferSize=telegramsize;
  debug[debug_cnt].chars = 0xBB;
  UART_SendMsg(usedUart, telegram, 7+datasize*2+2);
  DEBUG_MB_PRINTF("MBW->");
  ERR = waitForRespons(telegram, &telegramsize);
  DEBUG_HF_PRINTF("# TelWRS:%d", telegramsize);
  DEBUG_MB_PRINTF("MBW<-");
  return ERR;
}

static USART_ERROR ModbusBroadcast(u16 addr, u8 *data, u16 datasize)
{
  u16 crc;
  u8 telegram[256];
  int telegramsize=256;

  telegram[0]=0; // Id 0 is broadcast
  telegram[1]=WRITE_MULTIPLE_REGISTERS;/*write multiple regs*/
  telegram[2]=(addr & 0xFF00)>>8;
  telegram[3]=addr & 0x00FF;
  telegram[4]=(datasize & 0xFF00)>>8;
  telegram[5]=datasize & 0x00FF;
  telegram[6]=datasize*2;
  memcpy(&telegram[7], data, datasize*2);
  crc = gen_crc16(&telegram[0], 7+datasize*2);
  memcpy(&(telegram[7+datasize*2]), &crc, 2);
  recvBuffer=telegram;
  recvBufferSize=telegramsize;
  debug[debug_cnt].chars = 0xBB;
  UART_SendMsg(usedUart, telegram, 7+datasize*2+2);
  //ERR = waitForRespons(telegram, &telegramsize);
  vTaskDelay(100); /* Wait for "UART2_TX_Handler(void)" to be called but how?? */
  return NO_ERROR;
}

void ModbusTask( void * pvParameters )
{
  xMessage *msg;
  xMessage *msgout;
  Modbus_init(USART2);
  while(1)
  {
    /*wait for queue msg*/
    if( xQueueReceive( ModbusQueueHandle, &msg, portMAX_DELAY) == pdPASS )
    {
      switch(msg->ucMessageID)
      {
        case WRITE_MODBUS_REGS:
        {
          WriteModbusRegsReq *p;
          WriteModbusRegsRes *po;
          p=(WriteModbusRegsReq *)(msg->ucData);
          if(p->reply)
          {
            msgout=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsRes));
            if(msgout)
            {
              msgout->ucMessageID=WRITE_MODBUS_REGS_RES;
              po=(WriteModbusRegsRes *)(msgout->ucData);
              po->slave=p->slave;
              po->datasize=p->datasize;
              po->addr=p->addr;
              po->resultOk=ModbusWriteRegs(p->slave, p->addr, p->data, p->datasize);
              xQueueSend(p->reply, &msgout, portMAX_DELAY);
            }
          }
          else
          {
            ModbusWriteRegs(p->slave, p->addr, p->data, p->datasize);
          }
        }
        break;
        case READ_MODBUS_REGS:
        {
          ReadModbusRegsReq *p;
          ReadModbusRegsRes *po;
          p=(ReadModbusRegsReq *)(msg->ucData);
          if(p->reply)
          {
            msgout=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsRes)+p->datasize*sizeof(u16));
            if(msgout)
            {
              po=(ReadModbusRegsRes *)(msgout->ucData);
              msgout->ucMessageID=READ_MODBUS_REGS_RES;
              po->slave=p->slave;
              po->addr=p->addr;
              po->datasize=p->datasize;
              po->resultOk=FALSE;
              if(ModbusReadRegs(p->slave, p->addr, p->datasize, po->data)== NO_ERROR)
              {
                po->resultOk=NO_ERROR;
              }
              xQueueSend(p->reply, &msgout, portMAX_DELAY);
            }
          }
          else
          {
            ModbusReadRegs(p->slave, p->addr, p->datasize, 0);
          }
        }
        break;
        case BROADCAST_MODBUS:
        {
          WriteModbusRegsReq *p;
          p=(WriteModbusRegsReq *)(msg->ucData);
          // Ignore (p->reply) as no reply can be recieved from broadcasts
          ModbusBroadcast(p->addr, p->data, p->datasize);
        }
        break;
        default:
        break;
      };
      /*dealloc the msg*/
      vPortFree(msg);
    }
  }
}

void Modbus_init(USART_TypeDef *uart)
{
  usedUart=uart;
  TimerHandle=xTimerCreate((char*)"Modbus Response timer", MODBUS_RESPONSE_TIMEOUT, pdTRUE, NULL, response_timeoutCB);
  xModbusSemaphore = xSemaphoreCreateMutex();
  vQueueAddToRegistry(xModbusSemaphore,(char *)"MODBUS sem");
  DEBUG_MB_PRINTF("MB-SemTake-Init");
  xSemaphoreTake( xModbusSemaphore, portMAX_DELAY );
  DEBUG_MB_PRINTF("MB-Sem-Init-OK");
  UART_Init(USART2, recieveCharISR_CB);
  configTimerModbusTimeout();
}


