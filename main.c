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
#include "gdi.h"
#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"
#include "util.h"
#include "serial.h"
#include "adc.h"
#include "../heater-sw/heater_reg.h"
#include "version.h"
#include "debug.h"
#include "sequencer.h"

/* Private feature defines ---------------------------------------------------*/
#define QUEUESIZE 10
//#define USE_DEVELOPMENT_LOGGING

/* Private debug define ------------------------------------------------------*/
//#define DEBUG_CLOCK_MSO /*Set mux to output sysclk or other clocks on PA8*/

#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...)      sprintf(dbgbuf, fmt, ## args);  send_msg_on_monitor(dbgbuf);
#else
#define DEBUG_PRINTF(fmt, args...)      /* Don't do anything in release builds */
#endif

/* ---------------------------------------------------------------------------*/
/* Task Message Queues -------------------------------------------------------*/
xQueueHandle TubeSequencerQueueHandle;
xQueueHandle GDIQueueHandle;

/* ---------------------------------------------------------------------------*/
/* Task Handles ---------------------------------------------------------------*/
xTaskHandle pvTubeSequencerTaskTask;
xTaskHandle modbusCreatedTask;
xTaskHandle gdiCreatedTask;
xTaskHandle pvLogTask;
xTaskHandle pvCooleAndLidTask;
extern xTaskHandle pvSWUpdateTask;

/* ---------------------------------------------------------------------------*/

xTimerHandle heartbeatTimer;
uint32_t load=0xA5A5 ;

void ModbusTask( void * pvParameters );
void Modbus_init(USART_TypeDef *uart);
void CoolAndLidTask( void * pvParameters );
void gdi_task(void *pvParameters);
void TubeSequencerTask( void * pvParameter);
void LogTask( void * pvParameters );

/* ---------------------------------------------------------------------------*/
/* Global variables                                                           */
/* ---------------------------------------------------------------------------*/

extern void stopPeltier(void);
/* ---------------------------------------------------------------------------*/
/* Functions                                                                  */
/* ---------------------------------------------------------------------------*/
void fn(void)
{
  static int nesting = 0;

  nesting++;
  taskYIELD();
  fn();
  nesting--;
}

/* ---------------------------------------------------------------------------*/
void HeartBeat_ErrorLed_Pinconfig()
{
  /*HeartBeatLED PC9 to test*/
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  /* Configure the GPIO_LED pin HeartBeatLED PC9 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_SetBits(GPIOC,GPIO_Pin_9);

  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* Configure the GPIO_LED pin */
  /* Configure the GPIO_LED pins RxTx LEDs PB0,PB1 & ErrorLED PB11 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB,GPIO_Pin_11);/*ErrorLED */
  GPIO_ResetBits(GPIOB,GPIO_Pin_0); /*RX LED*/
  GPIO_ResetBits(GPIOB,GPIO_Pin_1); /*TX LED*/
}

/* ---------------------------------------------------------------------------*/
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
#if 1
   /*Setup for USART3 - MONITOR SEQ*/
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2Periph_GPIOC;
  AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_PARTIALREMAP;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;/*TX*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;/*RTS*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
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
  GPIO_SetBits(GPIOA, GPIO_Pin_15); //GPIO_Pin_15 will not go low Deactivate M0 reset

  /* TIM Configuration */
  PWM_PinConfig();

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

/* ---------------------------------------------------------------------------*/
void NVICInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  /* FREERTOS CONFIG For simplicity all bits must be defined
    to be pre-emption priority bits.  The following assertion will fail if
    this is not the case (if some bits represent a sub-priority).
      configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );*/
}

/* ---------------------------------------------------------------------------*/
void vHeartBeat_LEDToggle(xTimerHandle pxTimer )
{
  static uint8_t toggle = 0;
  GPIOC->ODR ^= GPIO_Pin_9;
  //DEBUG_PRINTF("FREE HEAP:%d",xPortGetFreeHeapSize());
  if(toggle){
    pingAllM0();
    toggle = 0;
  } else {
    toggle = 1;
  }
}

/* ---------------------------------------------------------------------------*/
void ConfigOSTimer ()
{
  heartbeatTimer = xTimerCreate((char *)"HeartbeatTimer", // Just a text name, not used by the kernel.
              ( 100 * 5 ),          // The timer period in ticks.
              pdTRUE,               // The timers will auto-reload themselves when they expire.
              ( void * ) 100,       // Assign each timer a unique id equal to its array index.
              vHeartBeat_LEDToggle  // Each timer calls the same callback when it expires.
              );
  if(NULL != heartbeatTimer ) {
    xTimerStart(heartbeatTimer, 0 );
  } // This is a debug feature - if initi fails it is not handled
}

/* ---------------------------------------------------------------------------*/
void init_os_trace()
{
#define ID_ISR_TIMER1 1       // lowest valid ID is 1
#define PRIO_OF_ISR_TIMER1 3  // the hardware priority of the interrupt
  
  /* Put the recorder data structure on the heap (malloc), if no 
  static allocation. (See TRACE_DATA_ALLOCATION in trcConfig.h). */
  vTraceInitTraceData();
  if (! uiTraceStart() )
  {
    PRINTF("Could not start recorder!");
  }else
  {
    PRINTF("OS trace started ");
    vTraceUserEvent(1);
    vTraceSetISRProperties(ID_ISR_TIMER1, "ISRTimer1", PRIO_OF_ISR_TIMER1);
  }
}

/* ---------------------------------------------------------------------------*/
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

/* ---------------------------------------------------------------------------*/
unsigned long vGetCounter()
{
  return TIM_GetCounter(TIM5);
}

/* ---------------------------------------------------------------------------*/
/* Dummy recieve CB function for the debug UART that does not recieve.        */
void noRecieve(void) 
{
}

/* ---------------------------------------------------------------------------*/
/*  Main program.                                                             */

uint16_t HW_Rev_Id __attribute__ ((section (".buildId_data")));

int main(void)
{
  NVICInit(); // MUST be done prior to adcInit()
  adcInit();
  HW_Rev_Id = readHwRevId(); // Obtain HW Revision ID ASAP
  HW_Init();
  UART_Init(USART3,noRecieve); /*Only for monitoring no RX*/
  PWM_Init(20000,100); // 20kHz/100Hz PWM : (TIM3(Fan, Aux), TIM4(Topheater1, Topheater2))
  PRINTF("Monitor Port UP");
  PRINTF("HW Rev ID = %d", HW_Rev_Id);
  PRINTF("SW build = %s", buildDateStr);
  PRINTF("Git Id = %s", gitCommitIdStr);
  init_os_trace(); /*GDB CMD:dump binary memory gdb_dump_23.txt 0x20000000 0x20010000  -- http://percepio.com/*/
  stopPeltier();
  PWM_Stop();

  ConfigOSTimer();
  initErrorLedTimer();

  /* create queues */
  ModbusQueueHandle=xQueueCreate( 32, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(ModbusQueueHandle,(char *)"MODBUS_Q");
  LogQueueHandle=xQueueCreate( 64, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(LogQueueHandle,(char *)"LOG_Q");
  CoolAndLidQueueHandle=xQueueCreate( QUEUESIZE, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(CoolAndLidQueueHandle,(char *)"CL_Q");
  TubeSequencerQueueHandle=xQueueCreate( 100, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(TubeSequencerQueueHandle,(char *)"Seq_Q");
  GDIQueueHandle=xQueueCreate( 32, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(GDIQueueHandle,(char *)"GDI_Q");
  /* create tasks */
  xTaskCreate( ModbusTask, ( const char * ) "Modbus task", ( unsigned short ) 400, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &modbusCreatedTask );
  xTaskCreate( LogTask, ( const char * ) "Log task", ( unsigned short ) 400, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &pvLogTask );
  xTaskCreate( gdi_task, ( const char * ) "Gdi task", ( unsigned short ) 600, NULL, ( ( unsigned portBASE_TYPE ) 1 ) | portPRIVILEGE_BIT, &gdiCreatedTask );
  xTaskCreate( CoolAndLidTask, (const char *) "Cool Lid task" /*max 16 chars*/, 300, NULL, ( (unsigned portBASE_TYPE) 4 ) | portPRIVILEGE_BIT, &pvCooleAndLidTask );
  xTaskCreate( TubeSequencerTask, ( const char * ) "TubeSeq task", ( unsigned short ) 1000, NULL, ( ( unsigned portBASE_TYPE ) 4 ) | portPRIVILEGE_BIT, &pvTubeSequencerTaskTask );

  { // Synchronize M0 LEDs
    xMessage *msg;
    WriteModbusRegsReq *p;

    msg = pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq));
    if(msg)
    {
      msg->ucMessageID=BROADCAST_MODBUS;
      p=(WriteModbusRegsReq *)msg->ucData;
      p->slave    = 0;    //not used for broadcast
      p->addr     = 0;
      p->datasize = 0;    //datasize;
      p->reply    = NULL; //No reply
      xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);      
    }
  }
  /* Start tasks */
  vTaskStartScheduler();
  return 0;
}


