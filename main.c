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

#define QUEUESIZE 10

xQueueHandle ModbusQueueHandle;
xSemaphoreHandle xSemaphore = NULL;

void ModbusTask( void * pvParameters );

static void AppTask( void * pvParameters )
{
  {
    xMessage *msg;
    WriteModbusRegsReq *p;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+20);
    msg->ucMessageID=WRITE_MODBUS_REGS;
    p=(WriteModbusRegsReq *)msg->ucData;
    p->slave=0x01;
    p->addr=0x05;
    memcpy(p->data, "55", 2);
    p->datasize=2;
    xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
  }
  vTaskDelay(5000);
  {
    xMessage *msg;
    ReadModbusRegsReq *p;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq));
    msg->ucMessageID=READ_MODBUS_REGS;
    p=(ReadModbusRegsReq *)msg->ucData;
    p->slave=0x01;
    p->addr=0x05;
    p->datasize=2;
    xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
  }
}

void HW_Init(void)
{
  /*Setup for UART*/
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

  HW_Init();
  UART_Init(USART2);
  Modbus_init(USART2);

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

