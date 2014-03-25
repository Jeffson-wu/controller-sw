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
#include <stm32f10x_gpio.h>
#include <stm32f10x_crc.h>
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "signals.h"
#include "swupdatetask.h"
#include "../m0_bootloader/config.h"

/* Public variables ----------------------------------------------------------*/
// command queue
xQueueHandle SwuQueueHandle;
xTaskHandle pvSWUpdateTask;

/* Private define ------------------------------------------------------------*/
#define  RS485_RE GPIOA,GPIO_Pin_0
#define  RS485_DE GPIOA,GPIO_Pin_1
#define  M0_RESET GPIO_Pin_8
/* Private typedef -----------------------------------------------------------*/
typedef enum PIN_STATE {
  PIN_LOW,
  PIN_HIGH
} pin_state_t;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern void SWupdate_taskkill(void);
void eventPinConfig(GPIOMode_TypeDef gpoi_mode);
void eventPinSet(pin_state_t io);

/* Private function ----------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
int SWU_start_task(void)
{
  int result = 0;

  DMA_DeInit(DMA1_Channel7);
  DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, DISABLE);

  SWupdate_taskkill();
  vTaskDelay(10); //The idle task is responsible for freeing the RTOS kernel allocated memory from tasks that have been deleted.
  result = xTaskCreate( SWUpdateTask, (const signed char *) "SW update task", 
    ( unsigned short ) 300, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &pvSWUpdateTask );
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
#if 1
  GPIO_ResetBits(GPIOA, M0_RESET);    //Activate M0 reset
  eventPinConfig(GPIO_Mode_Out_PP);   //Switch direction of event lines
  eventPinSet(PIN_HIGH);              //Set eventlines high
  GPIO_SetBits(GPIOA, M0_RESET);      //Deactivate M0 reset
#endif
  return result;
}

/* ---------------------------------------------------------------------------*/
void eventPinConfig(GPIOMode_TypeDef gpoi_mode)
{
  // GPIO_Mode_IPD, GPIO_Mode_Out_PP or GPIO_Mode_Out_OD?
  /*Setup for Heater Interrupt*/
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOC Configuration: */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = gpoi_mode;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* GPIOD Configuration: */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/* ---------------------------------------------------------------------------*/
void eventPinSet(pin_state_t io)
{
  if(PIN_LOW == io)
  { //Set eventlines low
    GPIOC->BRR = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIOD->BRR = GPIO_Pin_2;
  }
  else if(PIN_HIGH == io)
  { //Set event lines high
    GPIOC->BSRR = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIOC->BSRR = GPIO_Pin_2;
  }
}

/* ---------------------------------------------------------------------------*/
void SWU_SendMsg(u8 *buffer, int len)
{
  GPIO_SetBits(RS485_RE);//Remove on next PCB
  GPIO_SetBits(RS485_DE);
  int i=0;
  while(i<len)
  {
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)==RESET);
    USART_SendData(USART2,*(buffer+i));
    i++;
  }
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)==RESET);
  vTaskDelay(1); //Wait till last ch has actually been sent
  GPIO_ResetBits(RS485_RE);
  GPIO_ResetBits(RS485_DE);//Remove on next PCB
}

/* Public functions ----------------------------------------------------------*/
void startSWUpdate(void)
{
  int i;
  static data_package_t data_package;
  const end_package_t end_package = {
    FLASH_MAGIC_NUMBER,
    0,
    0,
    0x42cc5dee   //uint32_t crc; Calculated by the CRC unit: 0x42cc5dee
  };

  //CRC_ResetDR(); //generate crc
  //end_package.crc = CRC_CalcBlockCRC((uint32_t *) &end_package, (sizeof(end_package_t)-4)/4);
#if 0
  GPIO_ResetBits(GPIOA, M0_RESET);    //Activate M0 reset
  eventPinConfig(GPIO_Mode_Out_PP);   //Switch direction of event lines
  eventPinSet(PIN_HIGH);              //Set eventlines high
  GPIO_SetBits(GPIOA, M0_RESET);      //Deactivate M0 reset
#endif
  for(i=0; i<FLASH_PAGE_SIZE; i++) {
    data_package.data[i] = i%256;
  }
  data_package.ID = 15;     //Initially testing with only one M0
  data_package.magicNumber = FLASH_MAGIC_NUMBER;
  data_package.pkgNum = 31; //Program last page in flash
  CRC_ResetDR(); //generate crc
  data_package.crc = CRC_CalcBlockCRC((uint32_t *) &data_package, sizeof(data_package_t)-4);

  SWU_SendMsg((u8 *)&data_package, sizeof(data_package_t));  //send data package
  
  vTaskDelay(100); //Allow for M0 to write flash (measured @ 56,8[mS])
  SWU_SendMsg((u8 *)&end_package, sizeof(end_package_t));  //send end package

  vTaskDelay(10);  //Allow for M0 to act

  //inquire as to status
  //loop if not all packages succeded
#if 0
  GPIO_ResetBits(GPIOA, M0_RESET);  //Activate M0 reset
  eventPinConfig(GPIO_Mode_IPD);    //Switch direction of event lines
  GPIO_SetBits(GPIOA, M0_RESET);    //Deactivate M0 reset
#endif
}

/* ---------------------------------------------------------------------------*/
extern void SWUpdateTask( void * pvParameters )
{
  xMessage *msg;
  SetSSUpdateReq *p;
  //void SWupdate_taskkill(void)
  //void vTaskDelete( xTaskHandle xTaskToDelete ) PRIVILEGED_FUNCTION;

  while(1)
  {
    if( xQueueReceive( SwuQueueHandle, &msg, portMAX_DELAY) == pdPASS )
    {
      p=(SetSSUpdateReq *)msg->ucData;
      switch(msg->ucMessageID)
      {
      case START_SWU:
        if(1 == p->value)
        {
        }
        if(2 == p->value)
        {
          startSWUpdate();
        }
        break;
      }
    }
    vPortFree(msg);
  }
  // We are not supposed to end, but if so kill this task.
  vTaskDelete(NULL); //NULL means "this" task
  //taskYIELD();
}


