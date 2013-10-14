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

#define CRC16 0x8888
#define MODBUS_SILENT_INTERVAL 10

#define READ_HOLDINGS_REGISTERS 3
#define WRITE_MULTIPLE_REGISTERS 16

USART_TypeDef *usedUart;

void UART_SendMsg(USART_TypeDef *uart, u8 *buffer, int len);

xQueueHandle ModbusQueueHandle;

static u16 gen_crc16(const u8 *data, u16 size)
{
    u16 out = 0;
    int bits_read = 0, bit_flag, i;
    u16 crc = 0;
    /* Sanity check: */
    if(data == 0)
        return 0;
    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits
        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }
        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;
    }
    // item b) "push out" the last 16 bits
    for (i = 0; i < 16; ++i)
    {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }
    // item c) reverse the bits
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1)
    {
        if (i & out) crc |= j;
    }
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

void waitForRespons(u8 *telegram, int *telegramsize)
{
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
    if(stopReceiver==0)
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
}

static bool ModbusReadRegs(u8 slave, u16 addr, u16 datasize)
{
  u16 crc;
  u8 telegram[255];
  int telegramsize=255;
  telegram[0]=slave;
  telegram[1]=READ_HOLDINGS_REGISTERS;
  telegram[2]=addr & 0x00FF;
  telegram[3]=(addr & 0xFF00)>>8;
  telegram[4]=datasize & 0x00FF;
  telegram[5]=(datasize & 0xFF00)>>8;
  crc = gen_crc16(&telegram[0], 6);
  memcpy(&(telegram[6]), &crc, (size_t)2);
  UART_SendMsg(usedUart, telegram, 6+2);
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

static bool ModbusWriteRegs(u8 slave, u16 addr, u8 *data, u16 datasize)
{
  u16 crc;
  u8 telegram[255];
  int telegramsize=255;
  telegram[0]=slave;
  telegram[1]=WRITE_MULTIPLE_REGISTERS;/*write multiple regs*/
  telegram[2]=addr & 0x00FF;
  telegram[3]=(addr & 0xFF00)>>8;
  telegram[4]=datasize & 0x00FF;
  telegram[5]=(datasize & 0xFF00)>>8;
  telegram[6]=(datasize*2) & 0x00FF;
  telegram[7]=((datasize*2) & 0xFF00)>>8;
  memcpy(&telegram[8], data, datasize*2);
  crc = gen_crc16(&telegram[0], 8+datasize*2);
  memcpy(&(telegram[8+datasize*2]), &crc, 2);
  UART_SendMsg(usedUart, telegram, 8+datasize*2+2);
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
          p=(WriteModbusRegsReq *)(msg->ucData);
          ModbusWriteRegs(p->slave, p->addr, p->data, p->datasize);
        }
        break;
        case READ_MODBUS_REGS:
        {
          ReadModbusRegsReq *p;
          p=(ReadModbusRegsReq *)(msg->ucData);
          ModbusReadRegs(p->slave, p->addr, p->datasize);
        }
        default:
        break;
      };
      /*dealloc the msg*/
      vPortFree(msg);
    }

    /*call msg handler*/
  
  }
}

void Modbus_init(USART_TypeDef *uart)
{
  usedUart=uart;
}


