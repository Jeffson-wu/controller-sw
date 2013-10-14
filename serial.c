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

void UART_SendMsg(USART_TypeDef *uart, u8 *buffer, int len)
{
    DMA_InitTypeDef         DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /*rework KSKS*/
    static char txBuffer[260];
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
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & uart->DR;
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

    USART_DMACmd(uart, USART_DMAReq_Tx, ENABLE);
    USART_ClearFlag(uart, USART_FLAG_TC);
    DMA_Cmd(DMA1_Channel7, ENABLE);
}


void UART2_TX_Handler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC7);
}

void UART_Init(USART_TypeDef *uart)
{
  USART_InitTypeDef USART_InitStruct;
  USART_StructInit(&USART_InitStruct);
  USART_Init(uart, &USART_InitStruct);
  USART_Cmd(uart, ENABLE);
}

