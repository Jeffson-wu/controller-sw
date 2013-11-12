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
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "signals.h"
#include "timers.h"

#define MODBUS_SILENT_INTERVAL 40000/(19200/(8*4))

#define READ_HOLDINGS_REGISTERS 3
#define WRITE_MULTIPLE_REGISTERS 16

static USART_TypeDef *usedUart;

void UART_SendMsg(USART_TypeDef *uart, u8 *buffer, int len);

static  xSemaphoreHandle xSemaphore;
xQueueHandle ModbusQueueHandle;
static  xTimerHandle TimerHandle;

static  uint8_t *recvBuffer;
static int NOFRecvChars=0;
static int recvBufferSize;

static uint16_t gen_crc16(const uint8_t *buf, int len)
{
  uint16_t crc = 0xFFFF;
  int pos, i;
 
  for (pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
 
    for (i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;  
}

static bool handleModbusTelegram(u8 *telegram, u16 size)
{
    bool result=FALSE;
    u8 bytecount;
    u16 crc;
    u8 slaveID=telegram[0];
    switch(telegram[1])
    {
      case READ_HOLDINGS_REGISTERS:
      {
         u16 addr, datasize;
         bytecount=telegram[2];
         crc=telegram[3+bytecount];
         crc+=telegram[4+bytecount]<<8;
         if(gen_crc16(&telegram[0], 3+bytecount) == crc)
         {
           result=TRUE;
         }
      }
      break;
     case WRITE_MULTIPLE_REGISTERS: /*write regs*/
     {
         u16 addr, datasize;
         addr=telegram[2];
         addr+=telegram[3]<<8;
         datasize=telegram[4];
         datasize+=telegram[5]<<8;
         crc=telegram[6];
         crc+=telegram[7]<<8;
         if(gen_crc16(&telegram[0], 6) == crc)
         {
           result=TRUE;
         }
      }
      break;
     default:
     break;
    };
    return result;
}

void timeoutCB(xTimerHandle handle)
{
  static counter=0;
  if(recvBuffer && (NOFRecvChars!=0 || counter>10))
  {
    counter=0;
    /*Stop updating recvBuffer*/
    recvBuffer=0;
    /*unlock mutex to continue in modbus task*/
    xSemaphoreGive( xSemaphore );
  }
  else
  {
     counter++;
  }
}

void recieveChar(void)
{
  /*restart timer*/
  xTimerResetFromISR(TimerHandle,pdFALSE);
  while(USART_GetITStatus(USART2, USART_IT_RXNE))
  {
    if(recvBuffer && NOFRecvChars<recvBufferSize)
    {
      recvBuffer[NOFRecvChars]=USART_ReceiveData(usedUart);
      NOFRecvChars++;
    }
    else
    {
      USART_ReceiveData(usedUart);
    }
  }
}

void waitForRespons(u8 *telegram, int *telegramSize)
{
  /*start timer*/
  xTimerReset( TimerHandle, 0 );
  NOFRecvChars=0;
  recvBuffer=telegram;
  recvBufferSize=*telegramSize;

  /*wait for mutex*/
  xSemaphoreTake( xSemaphore, portMAX_DELAY );

  /*handle modbus telegram*/
  if(NOFRecvChars)
  {
    *telegramSize=NOFRecvChars;
  }
  else
  {
    *telegramSize=0;
  }
  return;
  #if 0
  int i=0;
  int stopReceiver=0;
  
  while(stopReceiver==0)
  {
    portTickType time=xTaskGetTickCount();
    while(USART_GetFlagStatus(usedUart, USART_FLAG_RXNE)== RESET)
    {
      portTickType currentTime=xTaskGetTickCount();
      if(time>currentTime)
      {
        /*wrap around*/
        portTickType maxtime=-1;
        if(currentTime+(maxtime-time)>MODBUS_SILENT_INTERVAL && i>0)
        {
          stopReceiver=1;
          break;
        }
      }
      else if(currentTime>time+MODBUS_SILENT_INTERVAL && i>0)
      {
        stopReceiver=1;
        break;
      }
    }
    if(stopReceiver==0 && i<*telegramsize)
    {
      if(i<*telegramsize)
      {
        *telegram = USART_ReceiveData(usedUart);
        telegram++;
        i++;
      }
      else
      {
        USART_ReceiveData(usedUart);
      }
    }
  }
  *telegramsize=i;
  #endif
}

static u8 ModbusReadRegs(u8 slave, u16 addr, u16 datasize, u8 *buffer)
{
  u16 crc;
  u8 telegram[255];
  int telegramsize=255;
  telegram[0]=slave;
  telegram[1]=READ_HOLDINGS_REGISTERS;
  telegram[2]=(addr & 0xFF00)>>8;
  telegram[3]=addr & 0x00FF;
  telegram[4]=(datasize & 0xFF00)>>8;
  telegram[5]=datasize & 0x00FF;
  crc = gen_crc16(&telegram[0], 6);
  memcpy(&(telegram[6]), &crc, (size_t)2);
  UART_SendMsg(usedUart, telegram, 6+2);
  waitForRespons(telegram, &telegramsize);
  memcpy(buffer, telegram, datasize>telegramsize?telegramsize:datasize);
  if(telegramsize-5<=0)
  {
    return 0;
  }
  else
  {
     if(buffer)
     {
       memcpy(buffer, &telegram[3], telegramsize-5);
     }
     return telegramsize-5;
  }
}

static bool ModbusWriteRegs(u8 slave, u16 addr, u8 *data, u16 datasize)
{
  u16 crc;
  u8 telegram[255];
  int telegramsize=255;
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
  UART_SendMsg(usedUart, telegram, 7+datasize*2+2);
  waitForRespons(telegram, &telegramsize);
  if(telegramsize==0)
  {
    return FALSE;
  }
  else
  {
    return TRUE;
  }
}

void ModbusTask( void * pvParameters )
{
  short usData;
  xMessage *msg;
  xMessage *msgout;
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
            po=(WriteModbusRegsRes *)(msgout->ucData);
            po->slave=p->slave;
            po->datasize=p->datasize;
            po->addr=p->addr;
            po->resultOk=ModbusWriteRegs(p->slave, p->addr, p->data, p->datasize);
            xQueueSend(p->reply, &msgout, portMAX_DELAY);
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
            msgout=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsRes)+p->datasize);
            po=(ReadModbusRegsRes *)(msgout->ucData);
            po->slave=p->slave;
            po->addr=p->addr;
            po->datasize=p->datasize;
            po->resultOk=FALSE;
            if(ModbusReadRegs(p->slave, p->addr, p->datasize, po->data))
            {
              po->resultOk=TRUE;
            }
            xQueueSend(p->reply, &msgout, portMAX_DELAY);
          }
        }
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
  TimerHandle=xTimerCreate("Modbus timer", MODBUS_SILENT_INTERVAL, pdTRUE, NULL, timeoutCB);
  xTimerStart( TimerHandle, 0 );
  xSemaphore = xSemaphoreCreateMutex();
  xSemaphoreTake( xSemaphore, portMAX_DELAY );
  UART_Init(USART2, recieveChar);

}

u8 DebugModbusReadRegs(u8 slave, u16 addr, u16 datasize, u8 *buffer)
{
   return ModbusReadRegs(slave, addr, datasize, buffer);
}

bool DebugModbusWriteRegs(u8 slave, u16 addr, u8 *data, u16 datasize)
{
   return ModbusWriteRegs(slave, addr, data, datasize);
}


