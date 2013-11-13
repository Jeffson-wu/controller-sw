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
#include "pwm.h"
#include "timers.h"
#include "stm32f10x_rcc.h"
#include "cooleandlidtask.h"
#define QUEUESIZE 10

extern xQueueHandle ModbusQueueHandle;
xSemaphoreHandle xSemaphore = NULL;

void ModbusTask( void * pvParameters );
void CooleAndLidTask( void * pvParameters );

void gdi_task(void *pvParameters);



static void AppTask( void * pvParameters )
{
  {
    xMessage *msg;
    WriteModbusRegsReq *p;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+20);
    msg->ucMessageID=WRITE_MODBUS_REGS;
    p=(WriteModbusRegsReq *)msg->ucData;
    p->slave=0x01;
    p->addr=64;
    memcpy(p->data, "\x00\x35\x00\x36", 4);
    p->datasize=2;
    p->reply=0;
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
    p->addr=64;
    p->datasize=2;
    p->reply=0;
    xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
  }
  while(1)
  {
    vTaskDelay(5000);
  }
}


void HeartBeat_Pinconfig()
{
/*HeartBeatLED PC9 to test*/
GPIO_InitTypeDef GPIO_InitStructure;
/* Enable the GPIO_LED Clock */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

/* Configure the GPIO_LED pin */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOC, &GPIO_InitStructure);
GPIO_SetBits(GPIOC,GPIO_Pin_9);
}
#if 1
void set_clock()
{
  RCC_ClocksTypeDef CLOCKS;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_HSEConfig(RCC_HSE_ON);
  RCC_WaitForHSEStartUp();
  RCC_PLLCmd(DISABLE);
  RCC_PLL2Cmd(DISABLE);
  
  /*Make system clock 72MHz expecting that oscillator is 25MHz ((((25MHz /5) * 9)/5)*8))*/

  RCC_PLLConfig(RCC_PLLSource_PREDIV1,RCC_PLLMul_8);
  RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2,RCC_PREDIV1_Div5);
  RCC_PLL2Config(RCC_PLL2Mul_9);
  RCC_PREDIV2Config(RCC_PREDIV2_Div5);
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  RCC_PLLCmd(ENABLE);
  RCC_PLL2Cmd(ENABLE);

  /*Debug output on PA8 to measure actual clock*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA,GPIO_Pin_8);
  
  RCC_MCOConfig(RCC_MCO_SYSCLK);
  
  RCC_GetClocksFreq(&CLOCKS);
  CLOCKS.SYSCLK_Frequency;


#if 0	/* Enable LSE (Low Speed External Oscillation) */
	  RCC_LSEConfig(RCC_LSE_ON);
	  
	  /* Check the LSE Status */
	  while(1)
	  {
		if(LSE_Delay < LSE_FAIL_FLAG)
		{
		  /* check whether LSE is ready, with 4 seconds timeout */
		  Delay (500);
		  LSE_Delay += 0x10;
		  if(RCC_GetFlagStatus(RCC_FLAG_LSERDY) != RESET)
		  {
			/* Set flag: LSE PASS */
			LSE_Delay |= LSE_PASS_FLAG;
			/* Turn Off Led4 */
			STM32vldiscovery_LEDOff(LED4);
			/* Disable LSE */
			RCC_LSEConfig(RCC_LSE_OFF);
			break;
		  } 	   
		}
		
		/* LSE_FAIL_FLAG = 0x80 */	
		else if(LSE_Delay >= LSE_FAIL_FLAG)
		{		   
		  if(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
		  {
			/* Set flag: LSE FAIL */
			LSE_Delay |= LSE_FAIL_FLAG;
			/* Turn On Led4 */
			STM32vldiscovery_LEDOn(LED4);
		  } 	   
		  /* Disable LSE */
		  RCC_LSEConfig(RCC_LSE_OFF);
		  break;
		}
	  }

#endif
}
#endif


void HW_Init(void)
{
  /*Setup for UART*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2Periph_GPIOD;
  AFIO->MAPR |= AFIO_MAPR_USART2_REMAP;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
  RCC->CFGR |= RCC_CFGR_SW_HSE;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  /*HeartBeatLED PC9 to test*/
  /* Enable the GPIO_LED Clock */
  HeartBeat_Pinconfig();

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOD,GPIO_Pin_3);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOD,GPIO_Pin_4);

  /* Configure USART3 for GDI (debug) interface */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* TIM Configuration */
  PWM_PinConfig();
}

void vHeartBeat_LEDToggle(xTimerHandle pxTimer )
{
  GPIOC->ODR ^= GPIO_Pin_9;
}


void ConfigOSTimer ()
{
  xTimerHandle xTimer;
  int x = 10;
  xTimer= xTimerCreate(	  "HeartbeatTimer",		   // Just a text name, not used by the kernel.
					  ( 100 * x ),	   // The timer period in ticks.
   		     		  pdTRUE,		  // The timers will auto-reload themselves when they expire.
  					  ( void * ) x, 	// Assign each timer a unique id equal to its array index.
  					  vHeartBeat_LEDToggle	 // Each timer calls the same callback when it expires.
  					  );
	
  if( xTimer == NULL )
  {
	  // The timer was not created.
  }
  else
  {
	  // Start the timer.  No block time is specified, and even if one was
	  // it would be ignored because the scheduler has not yet been
	  // started.
	  if( xTimerStart( xTimer, 0 ) != pdPASS )
	  {
		  // The timer could not be set into the Active state.
	  }
  }

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
  xTaskHandle gdiCreatedTask;
  xTaskHandle systemtestCreatedTask;
  xTaskHandle pvCooleAndLidTask;
  set_clock();

  HW_Init();
  PWM_Init(500000,250000);
  UART_Init(USART3);
  Modbus_init(USART2);
  PWM_Set(50,TopHeaterCtrlPWM);
  PWM_Set(50,FANctrlPWM);
  PWM_Set(50,PeltierCtrlPWM1);
  PWM_Set(70,PeltierCtrlPWM2);
  PWM_Set(50,PeltierCtrlPWM3);
  ConfigOSTimer();

  //HeartBeatLEDTimer();
  xSemaphore = xSemaphoreCreateMutex();
  /*create queue*/
  ModbusQueueHandle=xQueueCreate( QUEUESIZE, ( unsigned portBASE_TYPE ) sizeof( void * ) );

  result=xTaskCreate( ModbusTask, ( const signed char * ) "Modbus task", ( unsigned short ) 200, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &modbusCreatedTask );
  result=xTaskCreate( AppTask, ( const signed char * ) "App task", ( unsigned short ) 100, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &pvCreatedTask );
  result=xTaskCreate( CooleAndLidTask, (const signed char *) "CooleAndLid task", 100, NULL, ( (unsigned portBASE_TYPE) 3 ) | portPRIVILEGE_BIT, &pvCooleAndLidTask );
  result=xTaskCreate( gdi_task, ( const signed char * ) "Debug task", ( unsigned short ) 200, NULL, ( ( unsigned portBASE_TYPE ) 1 ) | portPRIVILEGE_BIT, &gdiCreatedTask );
  	
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

