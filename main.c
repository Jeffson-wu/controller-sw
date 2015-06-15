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

/* Private feature defines ---------------------------------------------------*/
#define QUEUESIZE 10

/* Private debug define ------------------------------------------------------*/
//#define DEBUG_CLOCK_MSO /*Set mux to output sysclk or other clocks on PA8*/

#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_PRINTF(fmt, args...)      /* Don't do anything in release builds */
#endif

/* ---------------------------------------------------------------------------*/
/* Task Message Queues -------------------------------------------------------*/
extern xQueueHandle ModbusQueueHandle;
xQueueHandle TubeSequencerQueueHandle;
extern xQueueHandle LogQueueHandle;
xQueueHandle GDIQueueHandle;
extern xQueueHandle CoolAndLidQueueHandle;

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

void ModbusTask( void * pvParameters );
void Modbus_init(USART_TypeDef *uart);
void CoolAndLidTask( void * pvParameters );
void gdi_task(void *pvParameters);
void TubeSequencerTask( void * pvParameter);
void LogTask( void * pvParameters );
void ErrorOn();
void ErrorOff();
void LogOn(int log_time);
void LogOff();

/* ---------------------------------------------------------------------------*/
/* Global variables                                                           */
/* ---------------------------------------------------------------------------*/
char buf[300]; /*buffer for debug printf*/

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

void printHeap(void) {
  extern size_t xFreeBytesRemaining;
  sprintf(buf, "Heap free bytes: %d", xFreeBytesRemaining);
  gdi_send_msg_on_monitor(buf);
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

  /* Enable the GPIO M0_RESET Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;/*M0_RESET*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA,GPIO_Pin_0);
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
void ResetHeaters()
{
  if( xTimerStart(yTimer[3], 0 ) != pdPASS );
  vTraceConsoleMessage("ResetHeaters!");
  GPIO_ResetBits(GPIOA,GPIO_Pin_0);
}

/* ---------------------------------------------------------------------------*/
void vHeatersReset(xTimerHandle pxTimer )
{
  GPIO_SetBits(GPIOA,GPIO_Pin_0);
  if( xTimerStop( yTimer[3], 0 ) != pdPASS );
}

/* ---------------------------------------------------------------------------*/
void ErrorOn()
{
  if( xTimerStart(yTimer[1], 0 ) != pdPASS );
}

/* ---------------------------------------------------------------------------*/
void ErrorOff()
{
  if( xTimerStop( yTimer[1], 0 ) != pdPASS );
}

/* ---------------------------------------------------------------------------*/
/* --> For USE_DEVELOPMENT_LOGGING feature */
void LogOn(int log_time)/*In secs*/
{
  if( xTimerStart(yTimer[2], 0 ) != pdPASS );
  if(xTimerChangePeriod( yTimer[2],10 * log_time,100)!= pdPASS );
}

/* ---------------------------------------------------------------------------*/
void LogOff()
{
  if( xTimerStop( yTimer[2], 0 ) != pdPASS );
}
/* <-- For USE_DEVELOPMENT_LOGGING feature */
/* ---------------------------------------------------------------------------*/
void vError_LEDToggle(xTimerHandle pxTimer )
{
  GPIOB->ODR ^= GPIO_Pin_11;
}

/* ---------------------------------------------------------------------------*/
void vHeartBeat_LEDToggle(xTimerHandle pxTimer )
{
  //char buf[20];
  GPIOC->ODR ^= GPIO_Pin_9;
  //DEBUG_PRINTF("FREE HEAP:%d",xPortGetFreeHeapSize());
}

/* ---------------------------------------------------------------------------*/
void ConfigOSTimer ()
{
  int z = 100;
  int y = 2;
  int x= 5;
  int r= 5; 
  int i = 0;
  yTimer[0]= xTimerCreate((char *)"HeartbeatTimer", // Just a text name, not used by the kernel.
              ( 100 * x ),          // The timer period in ticks.
              pdTRUE,               // The timers will auto-reload themselves when they expire.
              ( void * ) 100,       // Assign each timer a unique id equal to its array index.
              vHeartBeat_LEDToggle  // Each timer calls the same callback when it expires.
              );
  yTimer[1]= xTimerCreate((char *)"ErrorLedTimer", // Just a text name, not used by the kernel.
              ( 100 * y ),          // The timer period in ticks.
              pdTRUE,               // The timers will auto-reload themselves when they expire.
              ( void * ) 102,       // Assign each timer a unique id equal to its array index.
              vError_LEDToggle      // Each timer calls the same callback when it expires.
              );
/* --> For USE_DEVELOPMENT_LOGGING feature */
  yTimer[2]= xTimerCreate((char *)"LogTimer",       // Just a text name, not used by the kernel.
              ( 100 * z ),          // The timer period in ticks.
              pdTRUE,               // The timers will auto-reload themselves when they expire.
              ( void * ) 103,       // Assign each timer a unique id equal to its array index.
              vReadTubeTemp         // Each timer calls the same callback when it expires.
              );
/* <-- For USE_DEVELOPMENT_LOGGING feature */
  yTimer[3]= xTimerCreate((char *)"ResetHeaters",       // Just a text name, not used by the kernel.
              ( 100 * r ),          // The timer period in ticks.
              pdTRUE,               // The timers will auto-reload themselves when they expire.
              ( void * ) 104,       // Assign each timer a unique id equal to its array index.
              vHeatersReset         // Each timer calls the same callback when it expires.
              );

  for (i = 0; i < 1; i++)/*Only start Heartbeat timer, error timer will be started when needed to flash errorled*/
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

#if (SELECTED_PORT == PORT_ARM_CortexM)

#define _PORT_INIT_EXISTS

extern void prvSetupHardware(void);

/* ---------------------------------------------------------------------------*/
void port_init(void)
{
   // prvSetupHardware();
}
#endif

/* ---------------------------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
  ErrorOn();
  vTraceConsoleMessage("\n\rMalloc failed!\n\r");
}

/* ---------------------------------------------------------------------------*/
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
  ErrorOn();
  vTraceConsoleMessage("\n\rStack overflow!\n\r");
  GPIO_SetBits(GPIOB,GPIO_Pin_0);   /* Turn on RX LED */
  GPIO_SetBits(GPIOB,GPIO_Pin_1);   /* Turn on TX LED */
  taskDISABLE_INTERRUPTS();  
  for( ;; );
}

/* ---------------------------------------------------------------------------*/
void init_os_trace()
{
#define ID_ISR_TIMER1 1       // lowest valid ID is 1
#define PRIO_OF_ISR_TIMER1 3  // the hardware priority of the interrupt
  
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
  PWM_Init(20000,1000); // 20kHz/1kHz PWM : (TIM3(Fan, Aux), TIM4(Topheater1, Topheater2))
  UART_SendMsg(USART3, (u8*)"Monitor Port UP\r\n" , 17);
  sprintf(buf, "HW Rev ID = %d", HW_Rev_Id);
  gdi_send_msg_on_monitor(buf);
  sprintf(buf, "SW build = %s", buildDateStr);
  gdi_send_msg_on_monitor(buf);
  sprintf(buf, "Git Id = %s", gitCommitIdStr);
  gdi_send_msg_on_monitor(buf);
  init_os_trace(); /*GDB CMD:dump binary memory gdb_dump_23.txt 0x20000000 0x20010000  -- http://percepio.com/*/
  PWM_Stop();

  ConfigOSTimer();

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
void assert_failed(unsigned char* file, unsigned int line)
{ extern size_t xFreeBytesRemaining;
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  char buf[100];
  u8 data;
  GPIO_SetBits(GPIOB,GPIO_Pin_11);    /* Turn on error LED */
  GPIO_ResetBits(GPIOC,GPIO_Pin_9);   /* Turn off hartbeat LED */  
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);   /* Turn off RX LED */
  GPIO_ResetBits(GPIOB,GPIO_Pin_1);   /* Turn off TX LED */
  PWM_Stop();
  sprintf(buf, "assert_failed: %s %d", file, line);
  gdi_send_msg_on_monitor(buf);
  sprintf(buf, "Heap free bytes: %d", xFreeBytesRemaining);
  gdi_send_msg_on_monitor(buf);
  
  sprintf(buf, "Task: %s", pcTaskGetTaskName( xTaskGetCurrentTaskHandle()) );
  gdi_send_msg_on_monitor(buf);
  gdi_send_msg_on_monitor(dbgPrintIsr(buf));

  //__asm("BKPT #0\n") ; // Break into the debugger

  /* Infinite loop */
  /* cmd = "bu" -> "OK" to let Linux know that the M3 crashed */
  while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
  data = USART_ReceiveData(USART1);
  if('B' == (data & 0x0DF) )
  { 
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
    data = USART_ReceiveData(USART1);
    if('U' == (data & 0x0DF) )
    { 
      while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
      data = USART_ReceiveData(USART1);
      if('\r' == data)
      { 
        gdi_send_msg_response("OK");
      }
    }
  }
}
#endif

void HardFault_Handler(void)
{
 __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
  static uint32_t hardFaultSP __attribute__ ((section (".regPointer")));
  u8 data;
  /* These are volatile to try and prevent the compiler/linker optimising them
  away as the variables never actually get used.  If the debugger won't show the
  values of the variables, make them global by moving their declaration outside
  of this function. */
  volatile uint32_t r0;
  volatile uint32_t r1;
  volatile uint32_t r2;
  volatile uint32_t r3;
  volatile uint32_t r4;
  volatile uint32_t r5;
  volatile uint32_t r6;
  volatile uint32_t r7;
  volatile uint32_t r8;
  volatile uint32_t r9;
  volatile uint32_t r10;
  volatile uint32_t r11;
  volatile uint32_t r12;
  /*                sp is held in pulFaultStackAddress */
  volatile uint32_t lr; /* Link register. r14 */
  volatile uint32_t pc; /* Program counter. r15 */
  volatile uint32_t psr;/* Program status register. */
  volatile uint32_t _CFSR;
  volatile uint32_t _HFSR;
  volatile uint32_t _DFSR;
  volatile uint32_t _AFSR;
  volatile uint32_t _MMAR;
  volatile uint32_t _BFAR;

  register unsigned int _r4  __asm("r4");
  register unsigned int _r5  __asm("r5");
  register unsigned int _r6  __asm("r6");
  register unsigned int _r7  __asm("r7");
  register unsigned int _r8  __asm("r8");
  register unsigned int _r9  __asm("r9");
  register unsigned int _r10 __asm("r10");
  register unsigned int _r11 __asm("r11");

  register unsigned int _r13 __asm("r13");

  hardFaultSP = _r13; // Save current SP to find variables below in a RAM dump.
  r0 = pulFaultStackAddress[ 0 ];
  r1 = pulFaultStackAddress[ 1 ];
  r2 = pulFaultStackAddress[ 2 ];
  r3 = pulFaultStackAddress[ 3 ];
  r4 = _r4;
  r5 = _r5;
  r6 = _r6;
  r7 = _r7;
  r8 = _r8;
  r9 = _r9;
  r10 = _r10;
  r11 = _r11;
  r12 = pulFaultStackAddress[ 4 ];
  lr = pulFaultStackAddress[ 5 ];
  pc = pulFaultStackAddress[ 6 ];
  psr = pulFaultStackAddress[ 7 ];

  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;   
  
  // Hard Fault Status Register
  _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;
  
  // Debug Fault Status Register
  _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;
  
  // Auxiliary Fault Status Register
  _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;
  
  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
  // Bus Fault Address Register
  _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

  GPIO_SetBits(GPIOB,GPIO_Pin_11);  /* Turn on error LED */
  GPIO_ResetBits(GPIOC,GPIO_Pin_9); /* Turn off hartbeat LED */
  GPIO_SetBits(GPIOB,GPIO_Pin_0);   /* Turn on RX LED */
  GPIO_SetBits(GPIOB,GPIO_Pin_1);   /* Turn on TX LED */
  PWM_Stop();
  gdi_send_msg_on_monitor("\r\n!! HardFault !!"); //Print PC and SP for quick ref.
  //Bus Fault Status Register
  if(_CFSR & 0x00000100) { gdi_send_msg_on_monitor("IBUSRR"); }
  if(_CFSR & 0x00000200) { gdi_send_msg_on_monitor("PRECISERR"); }
  if(_CFSR & 0x00000400) { gdi_send_msg_on_monitor("IMPRECISERR"); }
  if(_CFSR & 0x00000800) { gdi_send_msg_on_monitor("UNSTKERR"); }
  if(_CFSR & 0x00001000) { gdi_send_msg_on_monitor("STKERR"); }
  if(_CFSR & 0x00008000) { 
      sprintf(buf, "BFARVALID  - BFAR: %08X", (unsigned int)_BFAR);
      gdi_send_msg_on_monitor(buf);
    }
  //Usage Fault Status Register
  if(_CFSR & 0x00010000) { gdi_send_msg_on_monitor("UNDEFINSTR"); }
  if(_CFSR & 0x00020000) { gdi_send_msg_on_monitor("INVSTATE"); }
  if(_CFSR & 0x00040000) { gdi_send_msg_on_monitor("INVPC"); }
  if(_CFSR & 0x00080000) { gdi_send_msg_on_monitor("NOCP"); }
  if(_CFSR & 0x00200000) { gdi_send_msg_on_monitor("DIVBYZERO"); }
  if(_CFSR & 0x00100000) { gdi_send_msg_on_monitor("UNALIGNED"); }
  //Memory Manage Fault Status Register
  if(_CFSR & 0x00000080) { gdi_send_msg_on_monitor("MMARVALID"); }
  if(_CFSR & 0x00000010) { gdi_send_msg_on_monitor("MSTKERR"); }
  if(_CFSR & 0x00000008) { gdi_send_msg_on_monitor("MUNSTKERR"); }
  if(_CFSR & 0x00000002) { gdi_send_msg_on_monitor("DACCVIOL"); }
  if(_CFSR & 0x00000001) { gdi_send_msg_on_monitor("IACCVIOL"); }
  sprintf(buf, "Task: %s", pcTaskGetTaskName( xTaskGetCurrentTaskHandle()) );
  gdi_send_msg_on_monitor(buf);
  gdi_send_msg_on_monitor(dbgPrintIsr(buf));
  
  //__asm("BKPT #0\n") ; // Break into the debugger

  /* cmd = "bu" -> "OK" to let Linux know that the M3 crashed */
  while(1)
  {
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
    data = USART_ReceiveData(USART1);
    if('B' == (data & 0x0DF) )
    { 
      while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
      data = USART_ReceiveData(USART1);
      if('U' == (data & 0x0DF) )
      { 
        while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
        data = USART_ReceiveData(USART1);
        if('\r' == data)
        { 
          gdi_send_msg_response("OK");
        }
      }
    }
  }
  r0=r0;
  r1=r1;
  r2=r2;
  r3=r3;
  r4=r4;
  r5=r5;
  r6=r6;
  r7=r7;
  r8=r8;
  r9=r9;
  r10=r10;
  r11=r11;
  r12=r12;
  lr=lr;
  pc=pc;
  psr=psr;
  hardFaultSP=hardFaultSP;
  
  _CFSR=_CFSR;
  _HFSR=_HFSR;
  _DFSR=_DFSR;
  _AFSR=_AFSR;
  _MMAR=_MMAR;
  _BFAR=_BFAR;
}




