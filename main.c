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
#include "stm32f10x.h"
#include "stm3210c-eval.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define CRC16 0x8888
#define QUEUESIZE 10
#define MODBUS_SILENT_INTERVAL 10

enum
{
  FIRST_MSG,
  WRITE_MODBUS_REGS,
  READ_MODBUS_REGS,
  LAST_MSG
};


typedef struct 
{
   portCHAR ucMessageID;
   portCHAR ucData[1];
}xMessage;

typedef struct
{
  u8 slave;
  u16 addr;
  u16 datasize;
  u8 data[1];
}WriteModbusRegsReq;

void UART_SendMsg( u8 *buffer, int len);

xQueueHandle ModbusQueueHandle;
xSemaphoreHandle xSemaphore = NULL;

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
      case 3: /*read regs*/
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
     case 16: /*write regs*/
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
    while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE)== RESET)
    {
      portTickType currentTime=xTaskGetTickCount();
      if(time>currentTime)
      {
        /*wrap around*/
        portTickType maxtime=-1;
        if(currentTime+(maxtime-time)>MODBUS_SILENT_INTERVAL && i>1)
        {
          stopReceiver=1;
          break;
        }
      }
      else if(currentTime>time+MODBUS_SILENT_INTERVAL && i>1)
      {
        stopReceiver=1;
        break;
      }
    }
    if(stopReceiver==0)
    {
      if(i<*telegramsize)
      {
        *telegram = USART_ReceiveData(USART2);
        telegram++;
        i++;
      }
      else
      {
        USART_ReceiveData(USART2);
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
  telegram[1]=3;/*write multiple regs*/
  telegram[2]=addr & 0x00FF;
  telegram[3]=(addr & 0xFF00)>>8;
  telegram[4]=datasize & 0x00FF;
  telegram[5]=(datasize & 0xFF00)>>8;
  crc = gen_crc16(&telegram[0], 6);
  memcpy(&(telegram[6]), &crc, (size_t)2);
  UART_SendMsg( telegram, 6+2);
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
  telegram[1]=16;/*write multiple regs*/
  telegram[2]=addr & 0x00FF;
  telegram[3]=(addr & 0xFF00)>>8;
  telegram[4]=datasize & 0x00FF;
  telegram[5]=(datasize & 0xFF00)>>8;
  telegram[6]=(datasize*2) & 0x00FF;
  telegram[7]=((datasize*2) & 0xFF00)>>8;
  memcpy(&telegram[8], data, datasize*2);
  crc = gen_crc16(&telegram[0], 8+datasize*2);
  memcpy(&(telegram[8+datasize*2]), &crc, 2);
  UART_SendMsg( telegram, 8+datasize*2+2);
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

static ModbusHandler(void)
{
  static int i=0;
  if(i>=4)
  {
      i=0;
      /*handle modbus*/
      /*handle incomming requests*/
  }
}

static void AppTask( void * pvParameters )
{
  xMessage *msg;
  WriteModbusRegsReq *p;
  msg=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+20);
  msg->ucMessageID=WRITE_MODBUS_REGS;
  p=(WriteModbusRegsReq *)msg->ucData;
  p->slave=0x22;
  p->addr=0x1111;
  memcpy(p->data, "01234567890123456789", 20);
  p->datasize=20;
  xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
  while(1);
}

static void ModbusTask( void * pvParameters )
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
          p=(WriteModbusRegsReq *)(msg->ucData);
          ModbusReadRegs(p->slave, p->addr, p->data, p->datasize);
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

void UART_SendMsg( u8 *buffer, int len)
{
    DMA_InitTypeDef         DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /*rework KSKS*/
    static char txBuffer[255];
    if(len > 255)
    {
      DMA_InitStructure.DMA_BufferSize = 255;
    }
    else
    {
      DMA_InitStructure.DMA_BufferSize = len;
    }
    memcpy(txBuffer, buffer, DMA_InitStructure.DMA_BufferSize);
    DMA_DeInit(DMA1_Channel7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & USART2->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)txBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);

    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
    USART_ClearFlag(USART2, USART_FLAG_TC);
    DMA_Cmd(DMA1_Channel7, ENABLE);
}


void UART2_TX_Handler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC7);
}

void HW_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2Periph_GPIOD;
  AFIO->MAPR |= AFIO_MAPR_USART2_REMAP;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void UART_Init(void)
{
  USART_InitTypeDef USART_InitStruct;
  USART_StructInit(&USART_InitStruct);
  USART_Init(USART2, &USART_InitStruct);
  USART_Cmd(USART2, ENABLE);
}



/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  int result;
  xTaskHandle pvCreatedTask;
  xTaskHandle modbusCreatedTask;

  xSemaphore = xSemaphoreCreateMutex();
  /*create queue*/
  ModbusQueueHandle=xQueueCreate( QUEUESIZE, ( unsigned portBASE_TYPE ) sizeof( void * ) );

  result=xTaskCreate( ModbusTask, ( const signed char * ) "Modbus task", ( unsigned short ) 200, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &modbusCreatedTask );
  result=xTaskCreate( AppTask, ( const signed char * ) "App task", ( unsigned short ) 200, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &pvCreatedTask );
  vTaskStartScheduler();
  return 0;
}



#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

