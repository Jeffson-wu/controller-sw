/*
 * serial.c
 *
 *  Created on: Okt 3, 2013
 *      Author: Tommy Kristensen <tfk@xtel.dk>
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
//#include "stm32f10x_misc.h"
#include "stm32f10x_tim.h"
#include "sequencer.h"
#include "ads1148.h"

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "signals.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>




#define STM32F051



//#define ADS_DRDY_PIN GPIO_Pin_7
//#define ADS_DRDY_EXTI EXTI_PinSource7










#define tube1 0x01
#define tube2 0x02

#define SET_TUBE_TEMP 0x01


#define tube_status 0x12

#define NUM_TIMERS 16 /*There should be 1 for each tube*/

 /* An array to hold handles to the created timers. */
 xTimerHandle xTimers[ NUM_TIMERS ];

 /* An array to hold a count of the number of times each timer expires. */
 long lExpireCounters[ NUM_TIMERS ] = { 0 };

/* Private variables ---------------------------------------------------------*/
typedef enum
{
TUBE_INIT,
TUBE_IDLE,
TUBE_WAIT_TEMP, /*Wait until desired temperature are reached*/
TUBE_WAIT_TIME, /*Wait the specified time in the sequence*/
TUBE_NOT_INITIALIZED
}TubeStates;



typedef enum
{
setstage,
title,
tube,
loopstart,
loopend
}CmdNameTypeDef;



typedef enum
{
Melting,
Annealing,
Extension,
Incubation,
LoopStart,
LoopEnd,
End
}TubeStageTypeDef;

typedef struct
{
uint16_t temp; /*Settemp in 0.1 degrees*/
uint16_t time; /*time in secs*/
TubeStageTypeDef stage;    /*Current stage:[M]elting(1), [A]nnealing(2), [E]xtension(3) or [I]ncubation(4) */
}StageCmdTypeDef;

typedef struct
{
uint8_t PINSOURCE; 
uint8_t PORTSOURCE;
uint32_t EXTI_LINE;
IRQn_Type IRQ_CH;
}gpio_extint_t;


typedef enum
{
/*ADS_DRDY,*/
Heater4,
Heater1,
Heater2,
Heater3,
Heater5,
Heater6,
Heater7,
Heater8,
nExtiGpio
}ExtiGpioTypeDef;

gpio_extint_t gpio_EXTI_CNF[nExtiGpio+1]={
/*{ADS_DRDY_PINSOURCE ,ADS_EXTI_PORTSOURCE,ADS_EXTI_LINE, EXTI15_10_IRQn},*//*ADS DRDY*/
{GPIO_PinSource10,GPIO_PortSourceGPIOC,EXTI_Line10,EXTI15_10_IRQn},/*Heater4*/
{GPIO_PinSource3 ,GPIO_PortSourceGPIOC,EXTI_Line3, EXTI3_IRQn},/*Heater1*/
{GPIO_PinSource4 ,GPIO_PortSourceGPIOC,EXTI_Line4, EXTI4_IRQn},/*Heater2*/
{GPIO_PinSource5 ,GPIO_PortSourceGPIOC,EXTI_Line5, EXTI9_5_IRQn},/*Heater3*/
{GPIO_PinSource11,GPIO_PortSourceGPIOC,EXTI_Line11,EXTI15_10_IRQn},/*Heater5*/
{GPIO_PinSource12,GPIO_PortSourceGPIOC,EXTI_Line12,EXTI15_10_IRQn},/*Heater6*/
{GPIO_PinSource2 ,GPIO_PortSourceGPIOD,EXTI_Line2, EXTI2_IRQn},/*Heater7*/
{GPIO_PinSource13,GPIO_PortSourceGPIOC,EXTI_Line13,EXTI15_10_IRQn},/*Heater8*/
{0xFF,0,EXTI0_IRQn}};/*Termination*/


extern xQueueHandle ModbusQueueHandle;
extern xQueueHandle TubeSequencerQueueHandle;



/* Private function prototypes -----------------------------------------------*/

void vTimerCallback( xTimerHandle pxTimer );
void ExtIrqDisable(ExtiGpioTypeDef heater);
void ExtIrqEnable(ExtiGpioTypeDef heater);



/* Private functions ---------------------------------------------------------*/
void StartTubeTimer( long TubeNum, long time  )
{
long x = TubeNum;

 /* Create then start some timers.  Starting the timers before the RTOS scheduler
 has been started means the timers will start running immediately that
 the RTOS scheduler starts. */
     xTimers[ x ] = xTimerCreate
	          (  /* Just a text name, not used by the RTOS kernel. */
                 "TubeTimer",
                 /* The timer period in ticks. */
                 ( 1000 * time ),
                 /* The timers will auto-reload themselves when they expire. */
                 pdTRUE,
                 /* Assign each timer a unique id equal to its array index. */
                 ( void * ) x,
                 /* Each timer calls the same callback when it expires. */
                 vTimerCallback
               );

     if( xTimers[ x ] == NULL )
     {
         /* The timer was not created. */
     }
     else
     {
         /* Start the timer.  No block time is specified, and even if one was
         it would be ignored because the RTOS scheduler has not yet been
         started. */
         if( xTimerStart( xTimers[ x ], 0 ) != pdPASS )
         {
             /* The timer could not be set into the Active state. */
         }
     }
}



StopTubeTimer(long TubeNum)
{

  xTimerStop( xTimers[ TubeNum ], 0 );

}

void heaterIrqInit(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  int i = 0;
  gpio_extint_t *p =  gpio_EXTI_CNF;
  EXTI_StructInit(&EXTI_InitStructure);
  
while(p->PINSOURCE != 0xFF)
{
  EXTI_InitStructure.EXTI_Line = p->EXTI_LINE;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

  GPIO_EXTILineConfig(p->PORTSOURCE , p->PINSOURCE);
  EXTI_Init(&EXTI_InitStructure);
  /* Clear any pending interrupts */
  EXTI_ClearITPendingBit(p->EXTI_LINE);

  NVIC_InitStructure.NVIC_IRQChannel = p->IRQ_CH;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  p++;
}
  
 


}

void ExtIrqDisable(ExtiGpioTypeDef heater)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = gpio_EXTI_CNF[heater].EXTI_LINE;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

  EXTI_Init(&EXTI_InitStructure);
}


void ExtIrqEnable(ExtiGpioTypeDef heater)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = gpio_EXTI_CNF[heater].EXTI_LINE;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

  /* Clear any pending interrupts */
  EXTI_ClearITPendingBit(gpio_EXTI_CNF[heater].EXTI_LINE);

  EXTI_Init(&EXTI_InitStructure);
}


void WriteTubeHeaterReg(u8 tube, u16 reg, u16 *data, u16 datasize)
{
    xMessage *msg;
    WriteModbusRegsReq *p;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+datasize);
#if 0
    msg->ucMessageID=WRITE_MODBUS_REGS;
    p=(WriteModbusRegsReq *)msg->ucData;
    p->slave=tube;
    p->addr=reg;
    memcpy(p->data, data, datasize);
    p->datasize=datasize;
    xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
#else
msg->ucMessageID=READ_MODBUS_REGS_RES;
p=(WriteModbusRegsReq *)msg->ucData;
p->slave=tube;
p->addr=reg;
memcpy(p->data, data, datasize);
p->datasize=datasize;
xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);

#endif
}



void ReadTubeHeaterReg(u8 tube, u16 reg, u16 datasize)
{
#if 0
	{
    xMessage *msg;
    ReadModbusRegsReq *p;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq)+datasize);
    msg->ucMessageID=READ_MODBUS_REGS;
    p=(ReadModbusRegsReq *)msg->ucData;
    p->slave=tube;
    p->addr=reg;
    p->datasize=datasize;
    xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
  }
#else
long lArrayIndex;
xMessage *msg;
long *p;
signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
msg->ucMessageID = DATA_FROM_TUBE;
p = (long *)msg->ucData;
*p = 52;
//xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);


xQueueSendFromISR(TubeSequencerQueueHandle,&msg,&xHigherPriorityTaskWoken);
if( xHigherPriorityTaskWoken )
{
  // Actual macro used here is port specific.
 // taskYIELD_FROM_ISR(); /*MIGHT BE USED TO FIX RESCHEDULING TFK*/
}


#endif
}


/**
  * @brief  Configure the GPIO Pins to PWM using TIM1
  * @param  None
  * @retval None
  */

void Heater_PinConfig(void)
{

/*Setup for Heater Interrupt*/
GPIO_InitTypeDef GPIO_InitStructure;


//RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC;
/* GPIOB-GPIOC Clocks enable */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);


/* GPIOD Configuration: Channel 3 and 4 as alternate function push-pull */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

GPIO_Init(GPIOD, &GPIO_InitStructure);

/* GPIOC Configuration: Channel 1, 2 and 3 as alternate function push-pull */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init(GPIOC, &GPIO_InitStructure);
}



HeaterEventHandler(ExtiGpioTypeDef Heater)
{
  switch (Heater)
  {
  case Heater1:
    ReadTubeHeaterReg(tube1,tube_status,1);
   // ReadTubeHeaterReg(tube2,tube_status,1);
  break;
  case Heater4:
  // ReadTubeHeaterReg(tube1,tube_status,1);
  // ReadTubeHeaterReg(tube2,tube_status,1);
  break;

  default:
  break;
  }
}



void EXTI_Handler(void)
{
ExtiGpioTypeDef ExtiGpio = Heater1; 
  /*ADS DRDY and Heater4_eventline is sharing EXTI_Line10 IRQ, so we need to find the source of IRQ then we read the status of the gpio pin*/
#if 1
  if(SET == EXTI_GetFlagStatus(ADS_EXTI_LINE))
  {
    if(GPIO_ReadInputDataBit(GPIOB,ADS_DRDY_PINSOURCE) == Bit_RESET)
    {
    //ADS_Handler();
    }
    if(GPIO_ReadInputDataBit(GPIOC,gpio_EXTI_CNF[Heater4].PINSOURCE) == Bit_RESET)
    {
    HeaterEventHandler(Heater4);
    }
    EXTI_ClearITPendingBit(ADS_EXTI_LINE);
  }
  #endif
  while(ExtiGpio < nExtiGpio)
  {
    if (SET == EXTI_GetFlagStatus(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE))
    {
		//	 HeaterEventHandler(ExtiGpio);
 	  EXTI_ClearITPendingBit(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE);
	}
  ExtiGpio++;
  }
}




void TubeSequencerTask( void * pvParameter)
{
short usData;
xMessage *msg;
long TimerId,TubeId;
StageCmdTypeDef TubeTemp[]={{50,10,Melting},{95,30,Annealing},{0,0,End}};
long *p;
WriteModbusRegsReq *preg;

TubeStates TubeStage[]={TUBE_NOT_INITIALIZED};
int TubeSeqNum = 0;


//StartTubeTimer(1,10);

while(1)
{
  /*wait for queue msg*/
  if( xQueueReceive( TubeSequencerQueueHandle, &msg, portMAX_DELAY) == pdPASS )
  {
	switch(msg->ucMessageID)
	{
    case START_TUBE_SEQ:
      TubeId = *((long *)(msg->ucData));
	  if(TubeTemp[TubeSeqNum].stage != End)
	  {
        WriteTubeHeaterReg(TubeId,SET_TUBE_TEMP,&TubeTemp[TubeSeqNum].temp,sizeof(TubeTemp[TubeSeqNum].temp));
		TubeStage[TubeId] = TUBE_WAIT_TEMP;
//	  }else if(TubeTemp[TubeSeqNum].stage != LoopStart)
//	  {
//	    TubeLoop[TubeId].LoopSeq = TubeSeqNum;
//        TubeLoop[TubeId].Iterations = TubeTemp[TubeSeqNum].time;
//	    TubeSeqNum++; /*Going to next sequence*/
//        msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
//        msg->ucMessageID=START_TUBE_SEQ;
//        p=(long *)msg->ucData;
//        *p=TubeId;
//        xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
	  }else
	  {
		TubeStage[TubeId] = TUBE_IDLE;
	  }
 	break;
	case READ_MODBUS_REGS_RES:
		preg=(WriteModbusRegsReq *)msg->ucData;
        TubeId = preg->slave;
        /*reg = p->addr;*/
	  if(TubeStage[TubeId] == TUBE_WAIT_TEMP) /*The heater has signalled an IRQ and here the status of the tube is read*/
	  {
 	    StartTubeTimer(TubeId,TubeTemp[TubeSeqNum].time);
	    TubeStage[TubeId] = TUBE_WAIT_TIME;
	  }
	break;
	case TIMER_EXPIRED:                       /*Waiting time for tube ended*/
	  TubeId = *((long *)(msg->ucData));
	  GPIOC->ODR ^= GPIO_Pin_9;
	  TubeSeqNum++; /*Going to next sequence*/
	  msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
	  msg->ucMessageID=START_TUBE_SEQ;
	  p=(long *)msg->ucData;
	  *p=TubeId;
	  xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
	break;
    case WRITE_MODBUS_REGS_RES:
      TubeId = *((long *)(msg->ucData));
	  
	break;
	default:
	break;
	};
	/*dealloc the msg*/
	vPortFree(msg);
  }

  /*call msg handler*/

}
}




void vTimerCallback( xTimerHandle pxTimer )
{
	long lArrayIndex;
	xMessage *msg;
    long *p;

	/* Optionally do something if the pxTimer parameter is NULL. */
	configASSERT( pxTimer );

	/* Which timer expired? */
	lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

	/* Do not use a block time if calling a timer API function from a
	timer callback function, as doing so could cause a deadlock! */
//	xTimerStop( pxTimer, 0 );
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
    msg->ucMessageID=TIMER_EXPIRED;
    p=(long *)msg->ucData;
	*p=lArrayIndex;
    xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
}

/**
  * @}
  */

/*******END OF FILE****/

                                                                                                                                                                                
