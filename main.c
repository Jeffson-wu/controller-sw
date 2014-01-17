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
#include "stm32f10x_tim.h"


#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"

#include "../heater-sw/heater_reg.h"


#define QUEUESIZE 10
//#define GDI_ON_USART3
//also in gdi.c

extern xQueueHandle ModbusQueueHandle;
xQueueHandle TubeSequencerQueueHandle;
xQueueHandle AppQueueHandle;
extern xQueueHandle CooleAndLidQueueHandle;

xSemaphoreHandle xSemaphore = NULL;
xTimerHandle yTimer[2];


void ModbusTask( void * pvParameters );
void CooleAndLidTask( void * pvParameters );

void gdi_task(void *pvParameters);



void TubeSequencerTask( void * pvParameter);
void ErrorOn();

void ErrorOff();


void fn(void)
{
  static int nesting = 0;
  char fill[30];
  nesting++;
  taskYIELD();
  fn();
  nesting--;
}


static void AppTask( void * pvParameters )
{
  ReadModbusRegsRes *preg;
  uint16_t modbus_data, modbus_addr;
  xMessage *msg;
  long TubeId;
  char message[20],i;
  
#ifdef GDI_ON_USART3
  USART_TypeDef *uart = USART3;
#else
  USART_TypeDef *uart = USART1;
#endif

  while(1)
  {
    if( xQueueReceive( AppQueueHandle, &msg, portMAX_DELAY) == pdPASS )
    {
    //DEBUG_PRINTF("TubeSeq[%s]",signals_txt[msg->ucMessageID]);
      switch(msg->ucMessageID)
      {
      case TUBE_TEST_SEQ:
        TubeId = *((long *)(msg->ucData));
        if (TubeId == 1)
        {
          ErrorOn();
        }else
        {
          ErrorOff();
        }
      break;
      case READ_MODBUS_REGS_RES:
        preg=(ReadModbusRegsRes *)msg->ucData;
        //TubeId = preg->slave;
        modbus_addr = preg->addr;
        /*modbus_data =  preg->data[1];*/
        modbus_data =(((u16)(preg->data[0])<<8)|(preg->data[1]));
        if((preg->resultOk == TRUE)&&(modbus_addr == TUBE1_TEMP_REG || TUBE2_TEMP_REG))
        {
          sprintf(message,"%d.%01dC ",dac_2_temp(modbus_data)/10,dac_2_temp(modbus_data)%10);

            for(i=0;i<strlen(message);i++)
            {
                while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
          //      UART_SendMsg(uart, message , strlen(message));
              USART_SendData(uart, *(message+i));
          //      while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
            }


       /*  vTraceConsoleMessage("%d.%02dC-",dac_2_temp(modbus_data)/10,dac_2_temp(modbus_data)%10);*/
        }
      break;
      }
    vPortFree(msg);
    }

  
  //  vTaskDelay(5000);

  }
}


void HeartBeat_ErrorLed_Pinconfig()
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
#if 1
/*ErrorLED PB11 to test*/
/* Enable the GPIO_LED Clock */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

/* Configure the GPIO_LED pin */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure);
GPIO_ResetBits(GPIOB,GPIO_Pin_11);
#endif
/*ErrorLED PB0-1 to test*/
/* Enable the GPIO_LED Clock */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

/* Configure the GPIO_LED pin */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure);
GPIO_ResetBits(GPIOB,GPIO_Pin_0);/*RX LED*/
GPIO_ResetBits(GPIOB,GPIO_Pin_1);/*TX LED*/



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
  
  /*Make system clock 72MHz expecting that oscillator is 16MHz (Heater)((((16MHz /4) * 9)/4)*8))*/
#ifndef STM32F10C_EVAL
  RCC_PLLConfig(RCC_PLLSource_PREDIV1,RCC_PLLMul_8);
  RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2,RCC_PREDIV1_Div4);
  RCC_PLL2Config(RCC_PLL2Mul_9);
  RCC_PREDIV2Config(RCC_PREDIV2_Div4);
#else
/*Make system clock 72MHz expecting that oscillator is 25MHz (Eval board)((((25MHz /5) * 9)/5)*8))*/
RCC_PLLConfig(RCC_PLLSource_PREDIV1,RCC_PLLMul_8);
RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2,RCC_PREDIV1_Div5);
RCC_PLL2Config(RCC_PLL2Mul_9);
RCC_PREDIV2Config(RCC_PREDIV2_Div5);
#endif
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
  
  //RCC_MCOConfig(RCC_MCO_SYSCLK);
  
  //RCC_GetClocksFreq(&CLOCKS);
  //CLOCKS.SYSCLK_Frequency;


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
#ifdef STM32F10C_EVAL
  /*Setup for USART2 - MODBUS*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2Periph_GPIOD;
  AFIO->MAPR |= AFIO_MAPR_USART2_REMAP;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
  RCC->CFGR |= RCC_CFGR_SW_HSE;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; /*TX*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;/*RX*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;/*CTS*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOD,GPIO_Pin_3);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;/*RTS*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOD,GPIO_Pin_4);

 #else
   /*Setup for USART2 - MODBUS*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2Periph_GPIOA;
//  AFIO->MAPR |= AFIO_MAPR_USART2_REMAP;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
  RCC->CFGR |= RCC_CFGR_SW_HSE;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;/*TX*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;/*RX*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;/*CTS*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA,GPIO_Pin_0);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;/*RTS*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA,GPIO_Pin_1);
 #endif
  /*HeartBeatLED PC9 to test*/
  /* Enable the GPIO_LED Clock */
  HeartBeat_ErrorLed_Pinconfig();


#ifdef GDI_ON_USART3
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
#else
  /* Configure USART1 for GDI (debug) interface */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

  /* TIM Configuration */
  PWM_PinConfig();

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  /* FREERTOS CONFIG For simplicity all bits must be defined
		to be pre-emption priority bits.  The following assertion will fail if
		this is not the case (if some bits represent a sub-priority).
			configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );*/

}

void ErrorOn()
{
	if( xTimerStart(yTimer[1], 0 ) != pdPASS );

}

void ErrorOff()
{
	if( xTimerStop( yTimer[1], 0 ) != pdPASS );

}


void vError_LEDToggle(xTimerHandle pxTimer )
{
    GPIOB->ODR ^= GPIO_Pin_11;
  
    xMessage *msg;
    ReadModbusRegsReq *p;
    portBASE_TYPE taskWoken = pdTRUE;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq));
    msg->ucMessageID=READ_MODBUS_REGS;
    p=(ReadModbusRegsReq *)msg->ucData;
    p->slave=1/*+0x02*/;
    p->addr=TUBE1_TEMP_REG;
    p->datasize=1;
    p->reply=AppQueueHandle;
    xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
}

#define DEBUG_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_response(buf);

void vHeartBeat_LEDToggle(xTimerHandle pxTimer )
{
  //char buf[20];
  GPIOC->ODR ^= GPIO_Pin_9;
  //DEBUG_PRINTF("FREE HEAP:%d",xPortGetFreeHeapSize());
}


void ConfigOSTimer ()
{
  int y = /*2*/100;
  int x= 10;
  int i = 0;
  yTimer[0]= xTimerCreate(	  "HeartbeatTimer",		   // Just a text name, not used by the kernel.
					  ( 100 * x ),	   // The timer period in ticks.
   		     		  pdTRUE,		  // The timers will auto-reload themselves when they expire.
  					  ( void * ) 100, 	// Assign each timer a unique id equal to its array index.
  					  vHeartBeat_LEDToggle	 // Each timer calls the same callback when it expires.
  					  );
  yTimer[1]= xTimerCreate(	  "ErrorLedTimer",		   // Just a text name, not used by the kernel.
					  ( 100 * y ),	   // The timer period in ticks.
   		     		  pdTRUE,		  // The timers will auto-reload themselves when they expire.
  					  ( void * ) 101, 	// Assign each timer a unique id equal to its array index.
  					  vError_LEDToggle	 // Each timer calls the same callback when it expires.
  					  );

for (i=0;i<1;i++)/*Only start Heartbeat timer, error timer will be started when needed to flash errorled*/
{
 if( yTimer[i] == NULL )
  {
	  // The timer was not created.
  }
  else
  {
	  // Start the timer.  No block time is specified, and even if one was
	  // it would be ignored because the scheduler has not yet been
	  // started.
	  if( xTimerStart(yTimer[i], 0 ) != pdPASS )
	  {
		  // The timer could not be set into the Active state.
	  }
  }
}
}
#define DEBUG_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_response(buf);

char buf[300]; /*buffer for debug printf*/

#if (SELECTED_PORT == PORT_ARM_CortexM)

#define _PORT_INIT_EXISTS

void port_init(void);

extern void prvSetupHardware(void);

void port_init(void)
{
   // prvSetupHardware();
}
#endif



void vApplicationMallocFailedHook( void );
extern void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName );

void vApplicationMallocFailedHook( void )
{
    vTraceConsoleMessage("\n\rMalloc failed!\n\r");
}

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
    vTraceConsoleMessage("\n\rStack overflow!\n\r");
	taskDISABLE_INTERRUPTS();
    for( ;; );
}

#if 0
void vAssertCalled( void );
void vApplicationTickHook( void );
void vApplicationIdleHook( void );

void vApplicationIdleHook( void )
{

}

void vApplicationTickHook( void )
{
}

void vAssertCalled( void )
{
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
#endif
void init_os_trace()
{
		/* Put the recorder data structure on the heap (malloc), if no 
		static allocation. (See TRACE_DATA_ALLOCATION in trcConfig.h). */
	
#ifdef _PORT_INIT_EXISTS
		port_init();
#endif	
	
		vTraceInitTraceData();
		
		if (! uiTraceStart() )
		{
				vTraceConsoleMessage("Could not start recorder!");
		}else
			{
			vTraceConsoleMessage("OS trace started");
			}

}

/* Defined in main.c. */
void vConfigureTimerForRunTimeStats( void )
{
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
uint16_t TimerPeriod_TIM5 = 0;
uint16_t TIM3_pwm_freq = 1500;


TimerPeriod_TIM5 = (SystemCoreClock / TIM3_pwm_freq ) - 1;

RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);

/* Time Base configuration */
TIM_TimeBaseStructure.TIM_Prescaler = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseStructure.TIM_Period = TimerPeriod_TIM5;
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

}


unsigned long vGetCounter()
{
return TIM_GetCounter(TIM5);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  int result,i;
  xTaskHandle pvCreatedTask;
  xTaskHandle modbusCreatedTask;
  xTaskHandle gdiCreatedTask;
  xTaskHandle systemtestCreatedTask;
  xTaskHandle pvCooleAndLidTask;

  long *p;
  long TubeId;
  xMessage *msg;
  
  set_clock();

  HW_Init();


  PWM_Init(1500,1500);
#ifdef GDI_ON_USART3
  UART_Init(USART3);
#else
  UART_Init(USART1);
#endif
  init_os_trace(); /*GDB CMD:dump binary memory gdb_dump_1.txt 0x20000000 0x20010000  -- http://percepio.com/*/

  Modbus_init(USART2);
  PWM_Set(16384,TopHeaterCtrlPWM);
  PWM_Set(50,FANctrlPWM);
  PWM_Set(8192,PeltierCtrlPWM1);
  PWM_Set(16384,PeltierCtrlPWM2);
  PWM_Set(24576,PeltierCtrlPWM3);
  ConfigOSTimer();

  //HeartBeatLEDTimer();
  xSemaphore = xSemaphoreCreateMutex();
  /*create queue*/
  ModbusQueueHandle=xQueueCreate( 32, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(ModbusQueueHandle,"MODBUS");
    AppQueueHandle=xQueueCreate( 32, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(AppQueueHandle,"APP");
  CooleAndLidQueueHandle=xQueueCreate( QUEUESIZE, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(CooleAndLidQueueHandle,"CooleAndLid");
  TubeSequencerQueueHandle=xQueueCreate( 100, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(TubeSequencerQueueHandle,"TubeSeq");

  heaterIrqInit();

 
  result=xTaskCreate( ModbusTask, ( const signed char * ) "Modbus task", ( unsigned short ) 400, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &modbusCreatedTask );
  result=xTaskCreate( AppTask, ( const signed char * ) "App task", ( unsigned short ) 300, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &pvCreatedTask );
  result=xTaskCreate( gdi_task, ( const signed char * ) "Debug task", ( unsigned short ) 400, NULL, ( ( unsigned portBASE_TYPE ) 1 ) | portPRIVILEGE_BIT, &gdiCreatedTask );
#ifndef GDI_ON_USART3
  //result=xTaskCreate( CooleAndLidTask, (const signed char *) "CooleAndLid task", 300, NULL, ( (unsigned portBASE_TYPE) 4 ) | portPRIVILEGE_BIT, &pvCooleAndLidTask );
#endif
  result=xTaskCreate( TubeSequencerTask, ( const signed char * ) "TubeSeq task", ( unsigned short ) 1000, NULL, ( ( unsigned portBASE_TYPE ) 4 ) | portPRIVILEGE_BIT, &pvCreatedTask );

 


#if 0
  for(i=1;i<17;i++)
  {
    TubeId = i;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
    msg->ucMessageID=TUBE_TEST_SEQ;
    p=(long *)msg->ucData;
    *p=TubeId;
    xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
  }

#endif  
#if 1
  TubeId = 1;
  msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
  msg->ucMessageID=TUBE_TEST_SEQ;
  p=(long *)msg->ucData;
  *p=TubeId;
  xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
#endif

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
	  ErrorOn();/*Turn on error led to show that sequence has ended*/

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

