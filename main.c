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
#define NUM_OF_TUBES 16+1

//#define DEBUG_CLOCK_MSO /*Set mux to output sysclk or other clocks on PA8*/
//#define GDI_ON_USART3
//also in gdi.c

extern xQueueHandle ModbusQueueHandle;
xQueueHandle TubeSequencerQueueHandle;
xQueueHandle GDIQueueHandle;
xQueueHandle LogQueueHandle;
extern xQueueHandle CooleAndLidQueueHandle;

xSemaphoreHandle xSemaphore = NULL;
xTimerHandle yTimer[3];
uint32_t load=0xA5A5 ;

bool log_tubes[NUM_OF_TUBES]={FALSE};

extern gdi_init();
void ModbusTask( void * pvParameters );
void CooleAndLidTask( void * pvParameters );
void gdi_task(void *pvParameters);
void TubeSequencerTask( void * pvParameter);
void ErrorOn();
void ErrorOff();
void LogOn(int log_time);
void LogOff();


void fn(void)
{
  static int nesting = 0;
  char fill[30];
  nesting++;
  taskYIELD();
  fn();
  nesting--;
}

static void LogTask( void * pvParameters )
{
  ReadModbusRegsRes *preg;
  uint16_t modbus_data, modbus_addr;
  xMessage *msg;
  long TubeId,log_interval;
  char message[20],i=1;
  static long last_tube_to_log;
  
#ifdef GDI_ON_USART3
  USART_TypeDef *uart = USART3;
#else
  USART_TypeDef *uart = USART1;
#endif

  while(1)
  {
    if( xQueueReceive( LogQueueHandle, &msg, portMAX_DELAY) == pdPASS )
    {
      switch(msg->ucMessageID)
      {
      case START_LOG:
        i=1;
        TubeId = *((long *)(msg->ucData));
        while((i < NUM_OF_TUBES)&&(log_tubes[i++]== FALSE));/*Check if this is the first tube to enable log on then enable log timer*/
        last_tube_to_log = i-1;
        if(i >= NUM_OF_TUBES)
        {
          LogOn(1000);
          last_tube_to_log = 0; /*Init to 0 since first tube*/
        }
        log_tubes[TubeId]= TRUE;
        if (TubeId > last_tube_to_log) last_tube_to_log = TubeId;
      break;
      case END_LOG:
       i=1;
       TubeId = *((long *)(msg->ucData));
       log_tubes[TubeId]= FALSE;
       while((i < NUM_OF_TUBES)&&(log_tubes[i++]== FALSE));/*Check if this is last tube to log on, then disable log timer*/
       last_tube_to_log = i-1;
       if(i >= NUM_OF_TUBES)
       {
         LogOff();
       }
      break;
      case READ_MODBUS_REGS_RES:
        preg=(ReadModbusRegsRes *)msg->ucData;
        TubeId = preg->slave;
        modbus_addr = preg->addr;
        modbus_data =(((u16)(preg->data[0])<<8)|(preg->data[1]));
        if((preg->resultOk == NO_ERROR)&&(modbus_addr == TUBE1_TEMP_REG || TUBE2_TEMP_REG))
        {
          sprintf(message,"T%d:%d.%01dC ",TubeId,dac_2_temp(modbus_data)/10,dac_2_temp(modbus_data)%10);

            for(i=0;i<strlen(message);i++)
            {
              while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
              USART_SendData(uart, *(message+i));
            }
          if(last_tube_to_log == TubeId) 
          {
            USART_SendData(uart, '\r');
            while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
            USART_SendData(uart, '\n');
            while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
          }
        }else
        {
           sprintf(message,"####Tube[%d]ERROR MODBUS READ FAILED-log!!! %d",TubeId,preg->resultOk);
           for(i=0;i<strlen(message);i++)
            {
              while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
              USART_SendData(uart, *(message+i));
            }
        }
      break;
      case SET_LOG_INTERVAL:
         log_interval = *((long *)(msg->ucData));
        if(xTimerChangePeriod( yTimer[2],1 * log_interval,100)!= pdPASS );
      break;
      }
    vPortFree(msg);
    }
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
}


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
void vReadTubeTemp(xTimerHandle pxTimer )
{
  xMessage *msg;
  ReadModbusRegsReq *p;
  portBASE_TYPE taskWoken = pdTRUE;
  int tube;

  for(tube=1;tube<17;tube++)
  {
    if(log_tubes[tube]== TRUE)
    {
      msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq));
      msg->ucMessageID=READ_MODBUS_REGS;
      p=(ReadModbusRegsReq *)msg->ucData;
      if((tube%2) == 0)
      {
        p->addr=TUBE2_TEMP_REG;      
      }else
      {
        p->addr=TUBE1_TEMP_REG;
      }
      p->datasize=1;
      p->reply=LogQueueHandle;
      p->slave=tube/*+0x02*/;
      xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
    }
  }
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
  xTaskHandle pvCreatedTask;
  xTaskHandle modbusCreatedTask;
  xTaskHandle gdiCreatedTask;
  xTaskHandle systemtestCreatedTask;
  xTaskHandle pvCooleAndLidTask;

  long *p;
  long TubeId;
  xMessage *msg;
  
  HW_Init();
  PWM_Init(1500,1500);

  gdi_init(); /*Setup debug uart*/

#ifdef GDI_ON_USART3
//  UART_Init(USART3,recieveCMD);
#else
//  UART_Init(USART1,recieveCMD);
#endif
  init_os_trace(); /*GDB CMD:dump binary memory gdb_dump_23.txt 0x20000000 0x20010000  -- http://percepio.com/*/
  Modbus_init(USART2);
  PWM_Set(16384,TopHeaterCtrlPWM);
  PWM_Set(50,FANctrlPWM); 
  PWM_Set(8192,PeltierCtrlPWM1);
  PWM_Set(16384,PeltierCtrlPWM2);
  PWM_Set(24576,PeltierCtrlPWM3);
  ConfigOSTimer();

  xSemaphore = xSemaphoreCreateMutex();
  /*create queue*/
  ModbusQueueHandle=xQueueCreate( 32, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(ModbusQueueHandle,"MODBUS");
  LogQueueHandle=xQueueCreate( 32, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(LogQueueHandle,"LOG");
  CooleAndLidQueueHandle=xQueueCreate( QUEUESIZE, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(CooleAndLidQueueHandle,"CooleAndLid");
  TubeSequencerQueueHandle=xQueueCreate( 100, ( unsigned portBASE_TYPE ) sizeof( void * ) );
  vQueueAddToRegistry(TubeSequencerQueueHandle,"GDI");
  
  

  heaterIrqInit();
  result=xTaskCreate( ModbusTask, ( const signed char * ) "Modbus task", ( unsigned short ) 400, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &modbusCreatedTask );
  result=xTaskCreate( LogTask, ( const signed char * ) "Log task", ( unsigned short ) 300, NULL, ( ( unsigned portBASE_TYPE ) 3 ) | portPRIVILEGE_BIT, &pvCreatedTask );
  result=xTaskCreate( gdi_task, ( const signed char * ) "Debug task", ( unsigned short ) 400, NULL, ( ( unsigned portBASE_TYPE ) 1 ) | portPRIVILEGE_BIT, &gdiCreatedTask );
#ifndef GDI_ON_USART3
  result=xTaskCreate( CooleAndLidTask, (const signed char *) "CooleAndLid task", 300, NULL, ( (unsigned portBASE_TYPE) 4 ) | portPRIVILEGE_BIT, &pvCooleAndLidTask );
#endif
  result=xTaskCreate( TubeSequencerTask, ( const signed char * ) "TubeSeq task", ( unsigned short ) 1000, NULL, ( ( unsigned portBASE_TYPE ) 4 ) | portPRIVILEGE_BIT, &pvCreatedTask );


  for(i=1;i<17;i++)
  {
    TubeId = i;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
    msg->ucMessageID=TUBE_TEST_SEQ;
    p=(long *)msg->ucData;
    *p=TubeId;
    xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
  }

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

