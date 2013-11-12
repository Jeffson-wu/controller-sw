/**
  ******************************************************************************
  * @file    cooleandlidtask.c
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    30-Oct -2013
  * @brief   Cooler and lid heater task
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
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
#include "timers.h"
#include "semphr.h"
#include "signals.h"
#include "ads1148.h"

void CooleAndLidTask( void * pvParameters )
{
  xSemaphoreHandle xADSSemaphore = NULL;
  int state = 0;
  uint32_t offsetcal;
  uint32_t fullscalecal;
  int16_t  ch0value, ch1value, ch2value, ch3value;

  /* Create ADC synchrinization semaphore and let the ADC ISR know about it */
  vSemaphoreCreateBinary(xADSSemaphore);
  assert_param(NULL != xADSSemaphore);
  xSemaphoreTake(xADSSemaphore, portMAX_DELAY); //Default is taken. ISR will give.

  adsSetIsrSemaphore(xADSSemaphore);
  /* Start convertion and let timeout handle subsequent calls to adsContiniueSequence */
  ads1148Init();
  adsStartSeq();
  adsIrqEnable();
  adsConfigConversionTimer(&adsTimerCallback);

  while(1)
  {

    /* The control task is synchronized to the ADC interrupt by semaphore */
    /* wait indefinitely for the semaphore to become free i.e. the ISR frees it. */
    xSemaphoreTake(xADSSemaphore, portMAX_DELAY);

    adsGetLatest(&ch0value, &ch1value, &ch2value, &ch3value);

    if(0 == state)
    {
      GPIO_SetBits(GPIOC, GPIO_Pin_6);
      state = 1;      
    }
    else
    {
      GPIO_ResetBits(GPIOC, GPIO_Pin_6);
      state = 0;
    }
    //vTaskDelay(500);

  }
  // We are not supposed to end, but if so kill this task.
  vTaskDelete(NULL);
}


