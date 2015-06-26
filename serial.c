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
#include "string.h"
#include "serial.h"
#include "debug.h"

#define  RS485_DE GPIOA,GPIO_Pin_1
#define  RS485_TX_LED GPIOB,GPIO_Pin_1
#define  RS485_RX_LED GPIOB,GPIO_Pin_0

static void (*receiveDataCB)()=0;
static void (*receiveUART1CB)()=0;
static void (*receiveUART3CB)()=0;

// Global
int USART3_intitalized = 0;

void UART_SendMsg(USART_TypeDef *uart, u8 *buffer, int len)
{
  if(uart==USART2)
  {
     /*UART2 is used for RS485 communication*/
     DMA_InitTypeDef         DMA_InitStructure;
     NVIC_InitTypeDef NVIC_InitStructure;
     len+=2;/*Delay to wait RS485 to settle*/
     GPIO_SetBits(RS485_DE);
     GPIO_SetBits(RS485_TX_LED);/*TX LED*/
     if(len > 255)
     {
       DMA_InitStructure.DMA_BufferSize = 255;
     }
     else
     {
       DMA_InitStructure.DMA_BufferSize = len;
     }
     DMA_DeInit(DMA1_Channel7);
     DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & uart->DR;
     DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buffer;
     DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
     DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
     DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
     DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
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
 
     USART_DMACmd(uart, USART_DMAReq_Tx, ENABLE);
     USART_ClearFlag(uart, USART_FLAG_TC);
     DMA_Cmd(DMA1_Channel7, ENABLE);
   }
   else
   {
     int i=0;
     while(i<len)
     {
       while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
       USART_SendData(uart,*(buffer+i));
       i++;
     }
   }
}

void UART2_TX_Handler(void)
{
  dbgTraceStoreISRBegin(TRACE_ISR_ID_UART2_TX);

   if(DMA_GetITStatus(DMA1_IT_TC7)==SET)
   {
     GPIO_ResetBits(RS485_DE);
     GPIO_ResetBits(RS485_TX_LED);/*TX LED*/
     DMA_ClearITPendingBit(DMA1_IT_TC7);
   }
   portEND_SWITCHING_ISR( pdTRUE);
   
   dbgTraceStoreISREnd();
}

void UART_Init(USART_TypeDef *uart, void (*recvCallback)())
{
  USART_InitTypeDef USART_InitStruct;
  NVIC_InitTypeDef NVIC_InitStructure;


  USART_StructInit(&USART_InitStruct);
  USART_InitStruct.USART_BaudRate=115200;/*Remember to update frametimer in modbus.c -> MODBUS_SILENT_INTERVAL when changing baudrate*/
  if(uart==USART1) {
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;//USART_HardwareFlowControl_None;
  }
  USART_Init(uart, &USART_InitStruct);
  USART_Cmd(uart, ENABLE);

  if(uart==USART2)
  {
    receiveDataCB=recvCallback;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }
  else if(uart==USART1)
  {
    receiveUART1CB=recvCallback;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =  configMAX_INTERRUPT_PRIORITY+1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }else if(uart==USART3)
  {
    receiveUART3CB=recvCallback;
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =  configMAX_INTERRUPT_PRIORITY+1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART3_intitalized = 1;
  }
  if(uart!=USART3)
    {
    USART_ITConfig(uart, USART_IT_RXNE, ENABLE);
    }
}


void UART3_Handler(void)
{
   dbgTraceStoreISRBegin(TRACE_ISR_ID_UART3);
   if (USART_GetITStatus(USART3, USART_IT_RXNE) && receiveUART3CB)
   {
     receiveUART3CB();
   }
   USART_ClearITPendingBit(USART3,USART_IT_RXNE);
   dbgTraceStoreISREnd();

}


void UART2_Handler(void)
{
  GPIO_SetBits(RS485_RX_LED);/*RX LED*/
  dbgTraceStoreISRBegin(TRACE_ISR_ID_UART2);
  if (USART_GetITStatus(USART2, USART_IT_RXNE) && receiveDataCB)
  {
    receiveDataCB();
  }
  USART_ClearITPendingBit(USART2,USART_IT_RXNE);
   dbgTraceStoreISREnd();
  GPIO_ResetBits(RS485_RX_LED);/*RX LED*/
  GPIO_ResetBits(RS485_TX_LED);/*TX LED*/
}

void UART1_Handler(void)
{
  dbgTraceStoreISRBegin(TRACE_ISR_ID_UART1);
   if (USART_GetITStatus(USART1, USART_IT_RXNE) && receiveUART1CB)
   {
     receiveUART1CB();
   }
   USART_ClearITPendingBit(USART1,USART_IT_RXNE);
   dbgTraceStoreISREnd();

}

