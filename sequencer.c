/*
 * Sequencer.c
 *
 *  Created on: Okt 3, 2013
 *      Author: Tommy Kristensen <tfk@xtel.dk>
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "signals.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sequencer.h"
#include "ads1148.h"
#include "../heater-sw/heater_reg.h"

#define STM32F051

//#define SIMULATE_HEATER

#define DEBUG /*General debug shows state changes of tubes (new temp, new time etc.)*/
#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_response(buf);
#else
#define DEBUG_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif

//#define DEBUG_SEQ /*Debug of sequencer, to follow state of sequencer*/
#ifdef DEBUG_SEQ
#define DEBUG_SEQ_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_response(buf);
#else
#define DEBUG_SEQ_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif

//#define DEBUG_IF /*Debug of external interfaces modbus, IRQ and serial */
#ifdef DEBUG_IF
#define DEBUG_IF_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_response(buf);
#else
#define DEBUG_IF_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif


#define tube1 0x01
#define tube2 0x02

#define SET_TUBE_TEMP 0x01
#define SET_TUBE_IDLE 0x02


#define tube_status 0x1

#define nTubes     16+1     /*Number of tubes to use, index 0 is dummy*/
#define NUM_TIMERS 16       /*There should be 1 for each tube*/

 /* An array to hold handles to the created timers. */
 xTimerHandle xTimers[ NUM_TIMERS ];

 /* An array to hold a count of the number of times each timer expires. */
 long lExpireCounters[ NUM_TIMERS ] = { 0 };
 
 char buf[300];                     /*buffer for debug printf*/
 bool Heater4_irq_handled = FALSE;  /* To block for interrupts from ADC since Heater 4 and ADC share same IRQ line*/

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
}TubestateTypeDef;

typedef struct
{
uint16_t temp;          /*Settemp in 0.1 degrees*/
uint32_t time;          /*time in 0.1 secs*/
TubestateTypeDef state; /*Current stage:[M]elting(1), [A]nnealing(2), [E]xtension(3) or [I]ncubation(4) */
}stateCmdTypeDef;

typedef struct
{
TubeStates state;
uint16_t LoopStart;
uint16_t LoopIterations;
int SeqIdx;             /*Current index that sequence are at now*/
stateCmdTypeDef *data;  /*Pointer to the sequence*/
portCHAR ucMessageID;  /*last message processed in statemachine*/
uint16_t event_reg;    /*last event/status register read from tube*/
}Tubeloop_t;

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

#ifdef DEBUG_IF
/* For debug to */
const char *  heater[] = {
/*ADS_DRDY,*/
"Heater4-Tube[7-8]",
"Heater1-Tube[1-2]",
"Heater2-Tube[3-4]",
"Heater3-Tube[5-6]",
"Heater5-Tube[9-10]",
"Heater6-Tube[11-12]",
"Heater7-Tube[13-14]",
"Heater8-Tube[15-16]",
"nExtiGpio"
};
#endif
ExtiGpioTypeDef tube2heater[]={
  0,
  Heater1,
	Heater1,
	Heater2,
	Heater2,
	Heater3,
	Heater3,
	Heater4,
	Heater4,
 	Heater5,
	Heater5,
	Heater6,
	Heater6,
	Heater7,
	Heater7,
	Heater8,
	Heater8
};

typedef struct
{
uint16_t tube_1;
uint16_t tube_2;
}heater_tubes_t;

heater_tubes_t heater2tube[]={
{7,8},/*Heater4*/
{1,2},/*Heater1*/	
{3,4},/*Heater2*/
{5,6},/*Heater3*/
{9,10},/*Heater5*/
{11,12},/*Heater6*/
{13,14},/*Heater7*/
{15,16} /*Heater8*/
};

/*const*/ gpio_extint_t gpio_EXTI_CNF[nExtiGpio+1]={
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

/*Tube state register*/
Tubeloop_t Tube[nTubes]= {{TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF},
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF}};

extern xQueueHandle ModbusQueueHandle;
extern xQueueHandle TubeSequencerQueueHandle;
extern xQueueHandle LogQueueHandle;


/* Private function prototypes -----------------------------------------------*/

void vTimerCallback( xTimerHandle pxTimer );
void ExtIrqDisable(ExtiGpioTypeDef heater);
void ExtIrqEnable(ExtiGpioTypeDef heater);

extern void gdi_send_msg_response(char * response);

/* Private functions ---------------------------------------------------------*/
void InitTubeTimers()
{
  long x;

  /* Create then start some timers.  Starting the timers before the RTOS scheduler
  has been started means the timers will start running immediately that
  the RTOS scheduler starts. */
  for(x=0;x<NUM_TIMERS;x++)
  {
    xTimers[ x ] = xTimerCreate
                  (	/* Just a text name, not used by the RTOS kernel. */
                  "TubeTimer",
                  /* The timer period in ticks. */
                  ( 1000 * 1 ),
                  /* The timers will auto-reload themselves when they expire. */
                  pdTRUE,
                  /* Assign each timer a unique id equal to its array index. */
                  ( void * ) x,
                  /* Each timer calls the same callback when it expires. */
                  vTimerCallback
                  );
  }
}

void StartTubeTimer( long TubeNum, long time  )
{
  long tube_idx = TubeNum -1;
  DEBUG_PRINTF("Tube[%d]StartTubeTimer Time[%d]",TubeNum,time);
  /*time in mSec*/

  if( xTimers[ tube_idx ] == NULL )
  {
    /* The timer was not created. */
  }
  else
  {
    /* Start the timer.  No block time is specified, and even if one was
    it would be ignored because the RTOS scheduler has not yet been
    started. */

    if(xTimerChangePeriod( xTimers[ tube_idx ],100 * time,100)!= pdPASS )
    {
    DEBUG_PRINTF("###ERROR TIMER SET Tube[%d]-Time[%d] ",TubeNum,time);
    }
    if( xTimerStart( xTimers[ tube_idx ], 0 ) != pdPASS )
    {
    /* The timer could not be set into the Active state. */
    
    DEBUG_PRINTF("###ERROR TIMER START Tube[%d]-Time[%d] ",TubeNum,time);
    }
  }
}



StopTubeTimer(long TubeNum)
{
  long tube_idx = TubeNum -1;

  if( xTimers[ tube_idx ] == NULL )
  {
   /* The timer was not created. */
  }
  else
  {
    xTimerStop( xTimers[ tube_idx ], 0 );
  }
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
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;

    GPIO_EXTILineConfig(p->PORTSOURCE , p->PINSOURCE);
    EXTI_Init(&EXTI_InitStructure);
    /* Clear any pending interrupts */
    EXTI_ClearITPendingBit(p->EXTI_LINE);

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_INTERRUPT_PRIORITY + 0x03   /*0x0B - 0x0F */;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; /*dont care*/
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
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&EXTI_InitStructure);
}

void ExtIrqEnable(ExtiGpioTypeDef heater)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = gpio_EXTI_CNF[heater].EXTI_LINE;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;

  /* Clear any pending interrupts */
  EXTI_ClearITPendingBit(gpio_EXTI_CNF[heater].EXTI_LINE);
  EXTI_Init(&EXTI_InitStructure);
}


void WriteTubeHeaterReg(u8 tube, u16 reg, u16 *data, u16 datasize)
{
  xMessage *msg;
  
#if defined (SIMULATE_HEATER)  /*Simulate response from Heater CPU*/
  ReadModbusRegsRes *p;
  msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsRes));
  msg->ucMessageID=READ_MODBUS_REGS_RES;
  p=(ReadModbusRegsRes *)msg->ucData;
  p->slave=tube;
  p->addr=EVENT_REG;
  p->data[0]=SEQUENCE_EVENT_TUBE1|SEQUENCE_EVENT_TUBE2;
  //memcpy(p->data, data, datasize);
  p->datasize=datasize;
  p->resultOk=TRUE;
  xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
  DEBUG_PRINTF("Tube[%d] MODBUS WRITE_REG[%d]data[%d]",tube,p->addr,*data);
#else
  WriteModbusRegsReq *p;
  msg=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+datasize*sizeof(u16));
  *data=((*data&0xFF)<<8)|(*data>>8);
  msg->ucMessageID=WRITE_MODBUS_REGS;
  p=(WriteModbusRegsReq *)msg->ucData;
  p->slave=tube/*+0x02*/;
  p->addr=reg;
  memcpy(p->data, data, datasize*sizeof(u16));
  p->datasize=datasize;
  p->reply=TubeSequencerQueueHandle;
  xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
  DEBUG_IF_PRINTF("Tube[%d]MODBUS WRITE_REG ID[%d] ADR[%d] data[%x]SIZE[%d]",tube, tube,p->addr,(((u16)(p->data[0])<<8)|(p->data[1])),p->datasize);
#endif
}

void ReadTubeHeaterReg(u8 tube, u16 reg, u16 datasize, bool from_isr)
{
#if defined (SIMULATE_HEATER) 
  long lArrayIndex;
  xMessage *msg;
  long *p;
  signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
  msg->ucMessageID = DATA_FROM_TUBE;
  p = (long *)msg->ucData;
  *p = 52;
  DEBUG_IF_PRINTF("Tube[%d]MODBUS READ_REG[tube_status] ",tube);
  xQueueSendFromISR(TubeSequencerQueueHandle,&msg,&xHigherPriorityTaskWoken);
  if( xHigherPriorityTaskWoken )
  {
  // Actual macro used here is port specific.
  // taskYIELD_FROM_ISR(); /*MIGHT BE USED TO FIX RESCHEDULING TFK*/
  }
#else
  xMessage *msg;
  ReadModbusRegsReq *p;
  portBASE_TYPE taskWoken = pdTRUE;
  msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq));
  msg->ucMessageID=READ_MODBUS_REGS;
  p=(ReadModbusRegsReq *)msg->ucData;
  p->slave=tube/*+0x02*/;
  p->addr=reg;
  p->datasize=datasize;
  p->reply=TubeSequencerQueueHandle;
  if (from_isr == TRUE)
  {
   assert_param(xQueueSendFromISR(ModbusQueueHandle,&msg,&taskWoken) == pdPASS);
  }else
  {
    assert_param(xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY)== pdPASS);
  }
  DEBUG_IF_PRINTF("Tube[%d]MODBUS READ_REG ID[%d] ADR[%d] SIZE[%d] CALLER:%s",tube, p->slave,p->addr, p->datasize, (from_isr == TRUE) ?"ISR":"NORM");
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* GPIOC Configuration: Channel 1, 2 and 3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void EXTI_Handler(void)
{
 vTraceStoreISRBegin(3);
  ExtiGpioTypeDef ExtiGpio = Heater1; 
  /*ADS DRDY and Heater4_eventline is sharing EXTI_Line10 IRQ, so we need to find the source of IRQ then we read the status of the gpio pin*/
  if(SET == EXTI_GetFlagStatus(ADS_EXTI_LINE))
  {
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10) == Bit_RESET) /*ADS_DRDY_PINSOURCE*/
    {
      ADS_Handler();
    }
    if((GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)== Bit_SET)&&(Heater4_irq_handled == FALSE))/*gpio_EXTI_CNF[Heater4].PINSOURCE */
    {
      ReadTubeHeaterReg(7,EVENT_REG,5, TRUE);
      Heater4_irq_handled = TRUE;
      DEBUG_IF_PRINTF("INTERRUPT-READ STATUS ON HEATER[%s]",heater[Heater4]);
    }
  EXTI_ClearITPendingBit(ADS_EXTI_LINE);
  }
  while(ExtiGpio < nExtiGpio) /*Always starts at Heater1 since */
  {
    if (SET == EXTI_GetFlagStatus(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE))
    {
      DEBUG_IF_PRINTF("INTERRUPT-READ STATUS ON HEATER[%s]",heater[ExtiGpio]);
      ReadTubeHeaterReg(heater2tube[ExtiGpio].tube_1,EVENT_REG,5, TRUE);
      EXTI_ClearITPendingBit(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE);
    }
    ExtiGpio++;
  }
  vTraceStoreISREnd();
  portEND_SWITCHING_ISR( pdTRUE);

}

uint16_t dac_2_temp(signed short dac)
{
  signed long res;
  res =(((dac*29549)/10000)+ 77175)/100 ;
  return (uint16_t)res;
}

signed short temp_2_dac(int16_t temp)
{
  signed short res;
  res = (10000*(((long)temp*100)-77175))/29549;
  return res;
}

int create_seq(long TubeId,int Nstates )
{
  Tubeloop_t *T = &Tube[TubeId];
  if(NULL != T->data)
  {
    vPortFree(T->data);
  }
  T->data = pvPortMalloc(sizeof(stateCmdTypeDef)*Nstates);
  return (int)T->data;
}

bool insert_state_to_seq(long TubeId,uint32_t time, uint16_t temp )
{
  bool result = TRUE;
  Tubeloop_t *T = &Tube[TubeId];
  stateCmdTypeDef *TSeq = &(T->data[T->SeqIdx]);
  if(Tube[TubeId].state == TUBE_IDLE)/*Check if sequence is already running on tube*/
  {
    if(time != 0)
    {
      TSeq->time = time;
      TSeq->temp = temp;
      TSeq->state = Melting;
      T->SeqIdx++;
    }else
    { /*Last entry in sequence for tube if time is = 0*/
      TSeq->time = 0;
      TSeq->temp = 0;
      TSeq->state = End;
      T->SeqIdx = 0;/*Set seq ready to start*/
    }
  }else
  {
    result = FALSE;
  }
return result;
}

void start_tube_seq( long TubeId)
{
  long *p;
  xMessage *msg;
  msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
  msg->ucMessageID=START_TUBE_SEQ;
  p=(long *)msg->ucData;
  *p=TubeId;
  xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
}

void log_tube_seq(long TubeId,bool enable )
{
  long *p;
  xMessage *new_msg;
  /*Start monitoring temperature on the tube*/
  new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
  if (enable = TRUE)
  {
    new_msg->ucMessageID = START_LOG;
  }else
  {
    new_msg->ucMessageID = END_LOG;
  }
  p=(long *)new_msg->ucData;
  *p=TubeId;
}





void stop_tube_seq( long TubeId)
{
  u16 data;
  long *p;
  xMessage *new_msg;
  if(Tube[TubeId].state == TUBE_WAIT_TIME)
  {
    StopTubeTimer(TubeId);
  }
  /*Set idle mode for tube to stop heating*/
  data = SET_IDLE_MODE;
  WriteTubeHeaterReg(TubeId,TUBE_COMMAND_REG,&data,sizeof(data)/2); 
  Tube[TubeId].SeqIdx = 0;
  Tube[TubeId].state = TUBE_IDLE;
  new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
  new_msg->ucMessageID=END_LOG;
  p=(long *)new_msg->ucData;
  *p=TubeId;
  xQueueSend(LogQueueHandle, &new_msg, portMAX_DELAY);
}

 void set_log_interval( long Log_Interval)
 {
  u16 data;
  long *p;
  xMessage *new_msg;
  new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
  new_msg->ucMessageID=SET_LOG_INTERVAL;
  p=(long *)new_msg->ucData;
  *p=Log_Interval;
  xQueueSend(LogQueueHandle, &new_msg, portMAX_DELAY);
}


void get_tube_state(long TubeId)
{
  char tube_state[10]="IDLE";
  for(TubeId = 1; TubeId < 17; TubeId++)
  {
    if(Tube[TubeId].data != NULL)
    {
      if((Tube[TubeId].state==TUBE_WAIT_TEMP)||((Tube[TubeId].state==TUBE_WAIT_TIME)))
      {
        strncpy(tube_state,"RUNNING",strlen("RUNNING"));
      }
      DEBUG_PRINTF("Tube[%d] %s @Step %d state:%s CURR temp:%d time:%d LAST MSG:%s EVENT:%x",TubeId,tube_state,Tube[TubeId].SeqIdx+1,tube_states[Tube[TubeId].state],Tube[TubeId].data[Tube[TubeId].SeqIdx].temp,Tube[TubeId].data[Tube[TubeId].SeqIdx].time,signals_txt[Tube[TubeId].ucMessageID],Tube[TubeId].event_reg);
    }else
    {
      DEBUG_PRINTF("Tube[%d]NO VALID SEQ FOUND State for Tube SEQ_ID:%d state:%s LAST MSG:%s EVENT:%x",TubeId,Tube[TubeId].SeqIdx,tube_states[Tube[TubeId].state],signals_txt[Tube[TubeId].ucMessageID],Tube[TubeId].event_reg);
    }
  }
}

void TubeStateHandler(long TubeId,xMessage *msg)
{
  xMessage *new_msg;
  long *p;
  u16 data;
  int a = 0;
  uint16_t i = 0;
  Tubeloop_t *T = &Tube[TubeId];

  if (T->data != NULL)
  {
    stateCmdTypeDef *TSeq = &(T->data[T->SeqIdx]);
    switch (TSeq->state)
    {
      case LoopStart:/*{3,0,LoopStart}*/
        T->LoopStart = T->SeqIdx+1;/*Loop from next entry*/
        T->LoopIterations = TSeq->time;
        DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] *** LOOPSTART @StarID[%d] Iterations[%d] ***",TubeId,tube_states[T->state],signals_txt[msg->ucMessageID],T->SeqIdx,T->LoopIterations);
        T->SeqIdx++; /*Going to next step*/
        new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
        new_msg->ucMessageID=NEXT_TUBE_STATE;
        p=(long *)new_msg->ucData;
        *p=TubeId;
        assert_param(pdPASS == xQueueSend(TubeSequencerQueueHandle, &new_msg, portMAX_DELAY));
      break;
      case LoopEnd: 
        T->LoopIterations--;
        if(T->LoopIterations > 0) /*Finished looping ?*/
        {
          T->SeqIdx=T->LoopStart; /*Jump back to start of loop*/
          DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s]*** LOOP @Iterations left[%d]",TubeId,tube_states[T->state],signals_txt[msg->ucMessageID],T->LoopIterations);
        }else
        {
          T->SeqIdx++;/*Finished looping go to next sequence*/
          DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s]***LOOP END***",TubeId,tube_states[T->state],signals_txt[msg->ucMessageID]);
        }
        new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
        new_msg->ucMessageID=NEXT_TUBE_STATE;
        p=(long *)new_msg->ucData;
        *p=TubeId;
        xQueueSend(TubeSequencerQueueHandle, &new_msg, portMAX_DELAY);
      break;
      case End:
        T->state = TUBE_IDLE;
        DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] END OF SEQUENCE FOR TUBE",TubeId,tube_states[T->state],signals_txt[msg->ucMessageID]);
        /*Set idle mode for tube to stop heating*/
        data = SET_IDLE_MODE;
        WriteTubeHeaterReg(TubeId,TUBE_COMMAND_REG,&data,sizeof(data)/2); 
        T->SeqIdx = 0;

        /*Stop monitoring temperature on the tube*/
        new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
        new_msg->ucMessageID=END_LOG;
        p=(long *)new_msg->ucData;
        *p=TubeId;
        xQueueSend(LogQueueHandle, &new_msg, portMAX_DELAY);
      break;
      case Melting:
      case Annealing:
      case Extension:
      case Incubation:
        if(T->SeqIdx == 0) 
        {
          /*Set heater to automatic mode if a new sequence is started*/
          data = SET_AUTOMATIC_MODE;
          WriteTubeHeaterReg(TubeId,TUBE_COMMAND_REG,&data,sizeof(data)/2);
          while(T->data[a].state != End)
          {
            a++;
          }
          DEBUG_PRINTF("\n\rTube[%d] Step %d of %d New temp %d.%01dC",TubeId,T->SeqIdx+1,a,TSeq->temp/10,TSeq->temp%10);/*ASCII 155 norm 167*/
        }else
        {
          DEBUG_PRINTF("\n\rTube[%d] Step %d Time reached New temp %d.%01dC",TubeId,T->SeqIdx+1,TSeq->temp/10,TSeq->temp%10);/*ASCII 155 norm 167*/
        }
        data = temp_2_dac(TSeq->temp);
        WriteTubeHeaterReg(TubeId,SETPOINT_REG,&data,sizeof(data)/2);
        T->state = TUBE_WAIT_TEMP;
        /*Start monitoring temperature on the tube*/
        new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
        new_msg->ucMessageID=START_LOG;
        p=(long *)new_msg->ucData;
        *p=TubeId;
        xQueueSend(LogQueueHandle, &new_msg, portMAX_DELAY);
      break;
      default:
        DEBUG_PRINTF("ERROR STATE NOT HADLED Tube[%d]@%s-%d TubeSeq[%s] ",TubeId,tube_states[T->state],T->state,signals_txt[msg->ucMessageID]);
      break;
    }
  }else
  {
  DEBUG_PRINTF("ERROR NO SEQ FOR Tube[%d]",TubeId);
  }
}

void TubeSequencerTask( void * pvParameter)
{
  short usData;
  xMessage *msg,*new_msg;
  long TimerId,TubeId;
  u16 modbus_addr,modbus_id;
  u16 modbus_data[5];
  bool modbus_result;
  signed portBASE_TYPE xEntryTimeSet = pdFALSE;
  long *p,i=0;
  ReadModbusRegsRes *preg;
  WriteModbusRegsRes *wres;
  ExtiGpioTypeDef heater;
  Tubeloop_t *T; 
  stateCmdTypeDef *TSeq; 

  InitTubeTimers();
  //vTaskDelay(2000);/*Wait for heaters to boot*/

  while(1)
  {
    /*wait for queue msg*/
    if( xQueueReceive( TubeSequencerQueueHandle, &msg, portMAX_DELAY) == pdPASS )
    {
      //DEBUG_PRINTF("TubeSeq[%s]",signals_txt[msg->ucMessageID]);
      switch(msg->ucMessageID)
      {
        case TUBE_TEST_SEQ:
          TubeId = *((long *)(msg->ucData));
          Tube[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          modbus_data[0]=1;
          DEBUG_PRINTF("TUBE[%d] TUBE_TEST_SEQ",TubeId);
          ReadTubeHeaterReg(TubeId,EVENT_REG,1, FALSE); /*Read & Clear pending events on heater*/
#if 0
          new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
          new_msg->ucMessageID=START_LOG;
          p=(long *)new_msg->ucData;
          *p=TubeId;
          xQueueSend(LogQueueHandle, &new_msg, portMAX_DELAY);
#endif
          while(heater != nExtiGpio)
          {
            heater++;
          }
          // json_test();
        break;
        case START_TUBE_SEQ:
          TubeId = *((long *)(msg->ucData));
          Tube[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          if((Tube[TubeId].state == TUBE_IDLE) && (Tube[TubeId].SeqIdx==0))
          {
            TubeStateHandler(TubeId,msg);
            DEBUG_PRINTF("Tube[%d]@%s START TUBE SEQUENCE ",TubeId,tube_states[Tube[TubeId].state]);
          }else
          {
            DEBUG_PRINTF("Tube[%d]@%s FAILED TO START TUBE SEQUENCE - SEQ Already running use AT@GDI:SEQ_CMD(%d,stop) to stop ",TubeId,tube_states[Tube[TubeId].state],TubeId);
          }
        break;
        case NEXT_TUBE_STATE:
          TubeId = *((long *)(msg->ucData));
          Tube[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          TubeStateHandler(TubeId,msg);
        break;
        case READ_MODBUS_REGS_RES:
          preg=(ReadModbusRegsRes *)msg->ucData;
          TubeId = preg->slave;
          Tube[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          modbus_addr = preg->addr;
          /*modbus_data =  preg->data[1];*/
          modbus_result = preg->resultOk;
          if(modbus_result != NO_ERROR)
          {
            DEBUG_PRINTF("####Tube[%d]ERROR MODBUS READ FAILED!!! %d",TubeId,modbus_result);
          }
          for(i=0;i<(preg->datasize);i++)
          {
            modbus_data[i] =(((u16)(preg->data[i*2])<<8)|(preg->data[(i*2)+1]));
          }
          DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s]ADDR[%d]size[%d]data[%x]STATUS[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],preg->addr,preg->datasize,/*(((u16)(preg->data[0])<<8)|(preg->data[1]))*/modbus_data[0],(preg->resultOk==NO_ERROR)?"PASS":"FAIL");
          if(preg->addr == SETPOINT_REG)
          {
            DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s] SetPoint[%d]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],dac_2_temp(modbus_data[0])/10);
          }
          if(preg->addr == EVENT_REG)
          {
            if(preg->resultOk==NO_ERROR)
            {
              if (TubeId == 7)
              {
                Heater4_irq_handled == FALSE; /*Now we have read the content of the event register and are ready to recieve a new IRQ from Heater4*/
              }
              if((Tube[TubeId].state == TUBE_WAIT_TEMP)&&((modbus_data[0] & SEQUENCE_EVENT_TUBE1)||(modbus_data[0] & SEQUENCE_EVENT_TUBE2)))
              {
                Tubeloop_t *T = &Tube[TubeId];
                stateCmdTypeDef *TSeq = &(T->data[T->SeqIdx]);
                modbus_id = TubeId%2;
                if(modbus_id == 0)modbus_id=2;
                DEBUG_PRINTF("\n\rTube[%d] Step %d Temp reached:%d.%01dC Start timer %d.%01d Sec",TubeId,T->SeqIdx+1,dac_2_temp(modbus_data[modbus_id])/10,dac_2_temp(modbus_data[modbus_id])%10,TSeq->time/10,TSeq->time%10);/*ASCII 155 norm 167*/
                StartTubeTimer(TubeId,TSeq->time);
                Tube[TubeId].state = TUBE_WAIT_TIME;
              }else
              {
                DEBUG_PRINTF("Tube[%d]EVENT[%x]:%s-%s-%s-%s-%s-%s-%s-%s",TubeId,modbus_data[0],((modbus_data[0] & SEQUENCE_EVENT_TUBE1)?"S_EV_T1 ":" "),((modbus_data[0] & SEQUENCE_EVENT_TUBE1)?"S_EV_T2 ":" "),((modbus_data[0] & INIT_HW_ERROR_TUBE1)?"HW_ERR_T1 ":" "),((modbus_data[0] & INIT_HW_ERROR_TUBE2)?"HW_ERR_T2 ":" "),((modbus_data[0] & TUBE1_NOT_PRESENT)?"NO_T1 ":" "),((modbus_data[0] & TUBE2_NOT_PRESENT)?"NO_T2 ":" "),((modbus_data[0] & EVENT6)?"EV6 ":" "),((modbus_data[0] & EVENT7)?"EV7 ":" "));
              }
              Tube[TubeId].event_reg = modbus_data[0];
              if(Tube[TubeId].state == TUBE_NOT_INITIALIZED)/*No errors on current tube, set it to IDLE so its ready for use*/
              {
                Tube[TubeId].state = TUBE_IDLE;
                modbus_data[0]=1;
                ExtIrqEnable(tube2heater[TubeId]);
                DEBUG_SEQ_PRINTF("ENABLE IRQ ON HEATER[%d]",tube2heater[TubeId]);
                WriteTubeHeaterReg(TubeId,EVENT_REG,&modbus_data[0], 1); /*******TEST to force the heater to create an event*/
              }
            }
            //   SEQUENCE_EVENT_TUBE1	 /* New temperature reached on tube 1             */	
            //   SEQUENCE_EVENT_TUBE2 /* New temperature reached on tube 2             */
            //   INIT_HW_ERROR_TUBE1 /* HW failure detected during start up on tube 1 */
            //   INIT_HW_ERROR_TUBE2  /* HW failure detected during start up on tube 2 */
            //   TUBE1_NOT_PRESENT /* Tube1 Not Present    */
            //   TUBE2_NOT_PRESENT  /* Tube1 Not Present      */
            //   EVENT6 /* EVENT6     */
            //   EVENT7 /* EVENT7     */
            //   EVENT8 /* EVENT8     */
          }
        break;
        case TIMER_EXPIRED:                       /*Waiting time for tube ended*/
          TubeId = *((long *)(msg->ucData));
          Tube[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID]);
          Tube[TubeId].SeqIdx++; /*Going to next sequence*/
          new_msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
          new_msg->ucMessageID = NEXT_TUBE_STATE;
          p=(long *)new_msg->ucData;
          *p=TubeId;
          xQueueSend(TubeSequencerQueueHandle, &new_msg, portMAX_DELAY);
        break;
        case WRITE_MODBUS_REGS_RES:
          wres = ((WriteModbusRegsRes *)(msg->ucData));
          TubeId = wres->slave;
          Tube[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          modbus_addr = wres->addr;
          modbus_result = wres->resultOk;
          if(modbus_result != NO_ERROR)
          {
            DEBUG_PRINTF("####Tube[%d]ERROR MODBUS WRITE FAILED!!! %d",TubeId,modbus_result);
          }
          DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s] Tube[%d]ADDR[%d]size[%d]STATUS[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],wres->slave,wres->addr,wres->datasize,(wres->resultOk==NO_ERROR)?"PASS":"FAIL");
        break;
        default:
          DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s]State[%s] ***UNHANDLED STATE***",0xFF,tube_states[Tube[0].state],signals_txt[msg->ucMessageID]);
        break;
      };
      vPortFree(msg);      /*dealloc the msg*/
    }
    /*call msg handler*/
  }
}


void vTimerCallback( xTimerHandle pxTimer )
{
  long lArrayIndex;
  xMessage *msg;
  long *p;
  portBASE_TYPE taskWoken = pdFALSE;

  /* Optionally do something if the pxTimer parameter is NULL. */
  //	configASSERT( pxTimer == NULL );

  /* Which timer expired? */
  lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

  /* Do not use a block time if calling a timer API function from a
  timer callback function, as doing so could cause a deadlock! */
  //xTimerStopFromISR(pxTimer,&taskWoken);
  //if( taskWoken != pdFALSE )
  //{
  // Call the interrupt safe yield function here (actual function
  // depends on the FreeRTOS port being used.
  //}
  xTimerStop( pxTimer, 0 );
  msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
  msg->ucMessageID=TIMER_EXPIRED;
  p=(long *)msg->ucData;
  *p=lArrayIndex+1;

  DEBUG_IF_PRINTF("Tube[%d] TIMER_EXPIRED ",lArrayIndex+1);
  xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
}
