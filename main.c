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
#include "logtask.h"
#include "swupdatetask.h"

#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"

#include "../heater-sw/heater_reg.h"


#define QUEUESIZE 10
#define REV_2


//#define DEBUG_CLOCK_MSO /*Set mux to output sysclk or other clocks on PA8*/

/* ---------------------------------------------------------------------------*/
/* Task Message Queues -------------------------------------------------------*/
extern xQueueHandle ModbusQueueHandle;
xQueueHandle TubeSequencerQueueHandle;
extern xQueueHandle LogQueueHandle;
xQueueHandle GDIQueueHandle;
extern xQueueHandle CoolAndLidQueueHandle;
extern xQueueHandle SwuQueueHandle;

/* ---------------------------------------------------------------------------*/
xSemaphoreHandle xModbusSemaphore = NULL;

/* ---------------------------------------------------------------------------*/
/* Task Handles ---------------------------------------------------------------*/
xTaskHandle pvTubeSequencerTaskTask;
xTaskHandle modbusCreatedTask;
xTaskHandle gdiCreatedTask;
xTaskHandle pvLogTask;
xTaskHandle pvCooleAndLidTask;
extern xTaskHandle pvSWUpdateTask;

/* ---------------------------------------------------------------------------*/

xTimerHandle yTimer[4];
uint32_t load=0xA5A5 ;

extern gdi_init();
void ModbusTask( void * pvParameters );
void CooleAndLidTask( void * pvParameters );
void gdi_task(void *pvParameters);
void TubeSequencerTask( void * pvParameter);
void LogTask( void * pvParameters );
void ErrorOn();
void ErrorOff();
void LogOn(int log_time);
void LogOff();


void SWupdate_taskkill(void)
{
  //Delete tasks not needed
  vTaskDelete( pvCooleAndLidTask )       PRIVILEGED_FUNCTION;
  vTaskDelete( pvLogTask )               PRIVILEGED_FUNCTION;
  vTaskDelete( pvTubeSequencerTaskTask ) PRIVILEGED_FUNCTION;
  vTaskDelete( modbusCreatedTask )       PRIVILEGED_FUNCTION;

  //Delete a queue - freeing all the memory allocated for storing of items placed on the queue.
  vQueueDelete(CoolAndLidQueueHandle);
  vQueueDelete(LogQueueHandle);
  vQueueDelete(TubeSequencerQueueHandle);
  vQueueDelete(ModbusQueueHandle);
}

void fn(void)
{
  static int nesting = 0;
  char fill[30];
  nesting++;
  taskYIELD();
  fn();
  nesting--;
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
  /*ErrorLED PB11 to test*/
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB,GPIO_Pin_11);

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

  #ifdef REV_2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;/*M0_RESET*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA,GPIO_Pin_0);
  #endif
}


void HW_Init(void)
{
   /*Setup for USART2 - MODBUS*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2Periph_GPIOA;
//  AFIO->MAPR |= AFIO_MAPR_USART2_REMAP;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;/*TX*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;/*RX*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#ifndef REV_2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;/*CTS*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA,GPIO_Pin_0);
#else
   /*Setup for USART3 - MONITOR SEQ*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2Periph_GPIOC;
  AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_PARTIALREMAP;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;/*TX*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;/*RTS*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA,GPIO_Pin_1);

  /*HeartBeatLED PC9 to test*/
  /* Enable the GPIO_LED Clock */
  HeartBeat_ErrorLed_Pinconfig();

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
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;/*CTS*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;/*RTS*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA,GPIO_Pin_12);

  /* Configure pin for lid lock ctrl.(PA8), M0 reset (PA15) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_8); //GPIO_Pin_15 will not go low Deactivate M0 reset

  /* TIM Configuration */
  PWM_PinConfig();

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  /* FREERTOS CONFIG For simplicity all bits must be defined
		to be pre-emption priority bits.  The following assertion will fail if
		this is not the case (if some bits represent a sub-priority).
			configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );*/

//#define DEBUG_CLOCK_MSO
#ifdef DEBUG_CLOCK_MSO
    /*Debug output on PA8 to measure actual clock*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA,GPIO_Pin_8);
    RCC_MCOConfig(RCC_MCO_SYSCLK);/*Set mux to output sysclk on PA8*/
    //RCC_GetClocksFreq(&CLOCKS);
    //CLOCKS.SYSCLK_Frequency;
#endif


}


void ResetHeaters()
{
  if( xTimerStart(yTimer[3], 0 ) != pdPASS );
  GPIO_ResetBits(GPIOA,GPIO_Pin_0);
}

void vHeatersReset(xTimerHandle pxTimer )
{
  GPIO_SetBits(GPIOA,GPIO_Pin_0);
  if( xTimerStop( yTimer[3], 0 ) != pdPASS );
}


void ErrorOn()
{
  if( xTimerStart(yTimer[1], 0 ) != pdPASS );
}

void ErrorOff()
{
if( xTimerStop( yTimer[1], 0 ) != pdPASS );
}
void LogOn(int log_time)/*In secs*/
{
  if( xTimerStart(yTimer[2], 0 ) != pdPASS );
  
  if(xTimerChangePeriod( yTimer[2],10 * log_time,100)!= pdPASS );
}

void LogOff()
{
if( xTimerStop( yTimer[2], 0 ) != pdPASS );
}

void vError_LEDToggle(xTimerHandle pxTimer )
{
  GPIOB->ODR ^= GPIO_Pin_11;
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
  int z = 100;
  int y = 2;
  int x= 5;
  int r= 5; 
  int i = 0;
  yTimer[0]= xTimerCreate("HeartbeatTimer", // Just a text name, not used by the kernel.
              ( 100 * x ),   // The timer period in ticks.
              pdTRUE, // The timers will auto-reload themselves when they expire.
              ( void * ) 100, // Assign each timer a unique id equal to its array index.
              vHeartBeat_LEDToggle // Each timer calls the same callback when it expires.
              );
  yTimer[1]= xTimerCreate("ErrorLedTimer", // Just a text name, not used by the kernel.
              ( 100 * y ),   // The timer period in ticks.
              pdTRUE,  // The timers will auto-reload themselves when they expire.
              ( void * ) 102,// Assign each timer a unique id equal to its array index.
              vError_LEDToggle // Each timer calls the same callback when it expires.
              );
  yTimer[2]= xTimerCreate("LogTimer",       // Just a text name, not used by the kernel.
              ( 100 * z ),     // The timer period in ticks.
              pdTRUE,     // The timers will auto-reload themselves when they expire.
              ( void * ) 103,   // Assign each timer a unique id equal to its array index.
              vReadTubeTemp   // Each timer calls the same callback when it expires.
              );
  yTimer[3]= xTimerCreate("ResetHeaters",       // Just a text name, not used by the kernel.
              ( 100 * r ),     // The timer period in ticks.
              pdTRUE,     // The timers will auto-reload themselves when they expire.
              ( void * ) 104,   // Assign each timer a unique id equal to its array index.
              vHeatersReset   // Each timer calls the same callback when it expires.
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

//####JRJ void port_init(void);

extern void prvSetupHardware(void);

void port_init(void)
{
   // prvSetupHardware();
}
#endif



//####JRJ void vApplicationMallocFailedHook( void );
//####JRJ extern void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName );

void vApplicationMallocFailedHook( void )
{
  ErrorOn();
  vTraceConsoleMessage("\n\rMalloc failed!\n\r");
}

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
  ErrorOn();
  vTraceConsoleMessage("\n\rStack overflow!\n\r");
  taskDISABLE_INTERRUPTS();
  for( ;; );
}

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
    vTraceConsoleMessage("OS trace started ");
    vTraceConsoleMessage("Compiled: %s @ %s",(uint8_t *)__TIME__,(uint8_t *)__DATE__);
    vTraceUserEvent(1);


#define ID_ISR_TIMER1 1       // lowest valid ID is 1
#define PRIO_OF_ISR_TIMER1 3  // the hardware priority of the interrupt

vTraceSetISRProperties(ID_ISR_TIMER1, "ISRTimer1", PRIO_OF_ISR_TIMER1);

  }
}


/* Defined in main.c. */
void vConfigureTimerForRunTimeStats( void )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t TimerPeriod_TIM5 = 0;
  uint16_t TIM5_freq = 1500;
  TimerPeriod_TIM5 = (SystemCoreClock / TIM5_freq ) - 1;
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

  long *p;
  long TubeId;
  xMessage *msg;
  
  HW_Init();
  PWM_Init(24000,24000);

  gdi_init(); /*Setup debug uart*/
 // UART_Init(USART3,NULL);
  //USART_SendData(USART3,'A');
  
  
  UART_SendMsg(USART3, "Monitor Port UP\r\n" , 16);

//  UART_Init(USART1,recieveCMD);
  init_os_trace(); /*GDB CMD:dump binary memory gdb_dump_23.txt 0x20000000 0x20010000  -- http://percepio.com/*/
  Modbus_init(USART2);
  PWM_Set(0,TopHeaterCtrlPWM);
  PWM_Set(0,FANctrlPWM); 
  PWM_Set(0,PeltierCtrlPWM1);
  PWM_Set(0,PeltierCtrlPWM2);
  PWM_Set(0,PeltierCtrlPWM3);
  ConfigOSTimer();

  xModbusSemaphore = xSemaphoreCreateMutex();
  /*create queue*/
  ModbusQueueHandle=xQueueCreate( 32, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(ModbusQueueHandle,"MODBUS");
  LogQueueHandle=xQueueCreate( 64, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(LogQueueHandle,"LOG");
  CoolAndLidQueueHandle=xQueueCreate( QUEUESIZE, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(CoolAndLidQueueHandle,"CooleAndLid");
  TubeSequencerQueueHandle=xQueueCreate( 100, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(TubeSequencerQueueHandle,"GDI");
  SwuQueueHandle=xQueueCreate( QUEUESIZE, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(SwuQueueHandle,"SWUpdate");
  

  heaterIrqInit();
  result=xTaskCreate( ModbusTask, ( const signed char * ) "Modbus task", ( unsigned short ) 400, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &modbusCreatedTask );
  result=xTaskCreate( LogTask, ( const signed char * ) "Log task", ( unsigned short ) 400, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &pvLogTask );
  result=xTaskCreate( gdi_task, ( const signed char * ) "Debug task", ( unsigned short ) 600, NULL, ( ( unsigned portBASE_TYPE ) 1 ) | portPRIVILEGE_BIT, &gdiCreatedTask );
  result=xTaskCreate( CooleAndLidTask, (const signed char *) "CooleAndLid task", 300, NULL, ( (unsigned portBASE_TYPE) 4 ) | portPRIVILEGE_BIT, &pvCooleAndLidTask );
  result=xTaskCreate( TubeSequencerTask, ( const signed char * ) "TubeSeq task", ( unsigned short ) 1000, NULL, ( ( unsigned portBASE_TYPE ) 4 ) | portPRIVILEGE_BIT, &pvTubeSequencerTaskTask );

#if 1
  for(i=1;i<17;i++)
  {
    TubeId = i;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
    if(msg)
    {
      msg->ucMessageID=TUBE_TEST_SEQ;
      p=(long *)msg->ucData;
      *p=TubeId;
      xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
    }
  }
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
    vTraceConsoleMessage("Assertion: %s line: %d", file, line);

  GPIO_SetBits(GPIOB,GPIO_Pin_11); /* Turn on error LED */
  GPIO_ResetBits(GPIOC,GPIO_Pin_9); /* Turn off hartbeat LED */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

void HardFault_Handler(void)
{
  GPIO_SetBits(GPIOB,GPIO_Pin_11); /* Turn on error LED */
  GPIO_ResetBits(GPIOC,GPIO_Pin_9); /* Turn off hartbeat LED */
  
  GPIO_SetBits(GPIOB,GPIO_Pin_0); /* Turn on RX LED */
  GPIO_SetBits(GPIOB,GPIO_Pin_1); /* Turn on TX LED */
  while (1) {}
}


