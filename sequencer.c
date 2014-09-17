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

//#define USE_IN_SEQUENCE_LOOPING
//#define SIMULATE_HEATER /*Disable communication to M0 CPU's return temperature reached when temp is requested*/
#define USE_DEVELOPMENT_LOGGING

extern xQueueHandle CoolAndLidQueueHandle;


#define USE_STATICALLY_ALLOCATED_SEQUENCES
#define USE_PAUSE_FEATURE
#define MAIN_IF_REV2
#define REV_2


//#define DEBUG /*General debug shows state changes of tubes (new temp, new time etc.)*/
#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif

//#define DEBUG_SEQ /*Debug of sequencer, to follow state of sequencer*/
#ifdef DEBUG_SEQ
#define DEBUG_SEQ_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_SEQ_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif

//#define DEBUG_IF /*Debug of external interfaces modbus, IRQ and serial */
#ifdef DEBUG_IF
#define DEBUG_IF_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_IF_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif


#define tube1 0x01
#define tube2 0x02

#define SET_TUBE_TEMP 0x01
#define SET_TUBE_IDLE 0x02

/*STATUS_REG Bit definitions */
#define SEQUENCE_EVENT    ((uint16_t)0x01)  /* New temperature reached on tube              */
#define INIT_HW_ERROR     ((uint16_t)0x02)  /* HW failure detected during start up on tube  */
#define NOT_PRESENT       ((uint16_t)0x04)  /* Tube not present                             */
#define LOGGING_READY     ((uint16_t)0x08)  /* Logging data buffer ready to be read         */
#define EVENT_DATA_SIZE 5

#define tube_status 0x1

#define nTubes     16+1     /*Number of tubes to use, index 0 is dummy*/
#define NUM_TIMERS 16       /*There should be 1 for each tube*/

//#define STAGES_PER_SEQ ((35*3)+1)   /* 35 iterations over 3 stages + 1 end indicator stage */

#define STAGES_PER_SEQ (4)   /* 4 iterations over 3 stages + 1 end indicator stage */


 /* An array to hold handles to the created timers. */
 xTimerHandle xTimers[ NUM_TIMERS ];

 /* An array to hold a count of the number of times each timer expires. */
 long lExpireCounters[ NUM_TIMERS ] = { 0 };
 
 /* An array to hold a count of the number of times each timer starts. */
 long lStartTickCounters[ NUM_TIMERS ] = { 0 };

 char buf[300];                     /*buffer for debug printf*/
 bool Heater4_irq_handled = FALSE;  /* To block for interrupts from ADC since Heater 4 and ADC share same IRQ line*/

/* Private variables ---------------------------------------------------------*/
#if 0
const char *  signals_txt[] = {
  "FIRST_MSG",
  "WRITE_MODBUS_REGS",
  "WRITE_MODBUS_REGS_RES",
  "TIMER_EXPIRED",
  "START_TUBE_SEQ",
  "TUBE_TEST_SEQ",
  "NEXT_TUBE_STAGE",
  "READ_MODBUS_REGS",
  "READ_MODBUS_REGS_RES",
  "START_TUBE",
  "DATA_FROM_TUBE",
  "SET_FAN",
  "SET_FAN_RES",
  "SET_COOLE_TEMP",
  "SET_COOLE_TEMP_RES",
  "SET_LID_TEMP",
  "SET_LID_TEMP_RES",
  "START_LOG",
  "END_LOG",
  "LAST_MSG"
};
#endif

const char *  signals_txt[] = {
 "FIRST_MSG",
 "WRITE_MODBUS_REGS",
 "WRITE_MODBUS_REGS_RES",
 "TIMER_EXPIRED",
 "START_TUBE_SEQ",
 "TUBE_TEST_SEQ",
 "NEXT_TUBE_STAGE",
 "READ_MODBUS_REGS",
 "READ_MODBUS_REGS_RES",
 "START_TUBE",
 "DATA_FROM_TUBE",
 "SET_COOLE_AND_LID",
 "SET_COOLE_AND_LID_RES",
 "SET_FAN_SPEED",
 "SET_FAN_RES",
 "SET_COOL_TEMP",
 "SET_COOLE_TEMP_RES",
 "SET_LID_TEMP",
 "SET_LID_TEMP_RES",
 "SET_LID_LOCK",
 "SET_LID_LOCK_RES",
 "START_LOG",
 "END_LOG",
 "SET_LOG_INTERVAL",
 "GDI_NEW_CMD",
 "BROADCAST_MODBUS",
 "START_SWU",
 "LAST_MSG",
 "START_LID_HEATING",
 "STOP_LID_HEATING",
 "SET_LID_PWM",
 "INIT_TUBE_SEQ"
};




const char *  tube_sta[] = {
"Melting",
"Annealing",
"Extension",
"Incubation",
"Pause",
"LoopStart",
"LoopEnd",
"End",
};



typedef enum
{
  TUBE_INIT,
  TUBE_IDLE,
  TUBE_WAIT_TEMP,   /*Wait until desired temperature is reached*/
  TUBE_WAIT_TIME,   /*Wait the specified time in the sequence  */
  TUBE_WAIT_P_TEMP, /*Wait until pause temperature is reached  */
  TUBE_PAUSED,      /*Wait until continiue is requested        */
  TUBE_OUT_OF_DATA, /*No more data in que and no end tag is found*/
  TUBE_NOT_INITIALIZED
}TubeStates;

const char *  tube_states[] = {
"TUBE_INIT",
"TUBE_IDLE",
"TUBE_WAIT_TEMP",   /*Wait until desired temperature is reached*/
"TUBE_WAIT_TIME",   /*Wait the time specified in the sequence  */
"TUBE_WAIT_P_TEMP", /*Wait until pause temperature is reached  */
"TUBE_PAUSED",
"TUBE_OUT_OF_DATA", /*No more data in que and no end tag is found*/
"TUBE_NOT_INITIALIZED"
};

/* ---------------------------------------------------------------------------*/
/* Pause is requested by setting pausePendingState = TUBE_P_PAUSE_REQUESTED   */
/* pausePendingState remains TUBE_P_PAUSE_REQUESTED untill tube state is set  */
/* to TUBE_PAUSED (i.e while tube state is TUBE_WAIT_P_TEMP), or a continue   */
/* cmd is received.                                                           */
typedef enum TUBE_PAUSE_FLAGS {
  TUBE_P_NORM,
  TUBE_P_PAUSE_REQUESTED
} TubePauseStates_t;

typedef enum
{
setstage,
title,
tube,
loopstart,
loopend
}CmdNameTypeDef;





#ifdef USE_STATICALLY_ALLOCATED_SEQUENCES
typedef struct
{
  TubeStates state;
  TubePauseStates_t pausePendingState;
  portCHAR ucMessageID;   /* last message processed in state machine  */
  uint16_t LoopStart;
  uint16_t LoopIterations;
  uint16_t pauseTemp;     /* Temperature to maintain during pause     */
  uint16_t event_reg;     /* last event/status register read from tube*/
  int SeqIdx;             /* Current index that sequence are at now   */
  int tail;              /*Read Pointer*/ 
  int  head;              /*Write Pointer */
  stageCmd_t *data;       /* Pointer to the sequence*/
  stageCmd_t seq[STAGES_PER_SEQ];   /* The sequence */
  stageCmd_t curr;
}Tubeloop_t;
#else
typedef struct
{
  TubeStates state;
  TubePauseStates_t pausePendingState;
  uint16_t LoopStart;
  uint16_t LoopIterations;
  uint16_t pauseTemp;     /*Temperature to maintain during pause*/
  int SeqIdx;             /*Current index that sequence are at now*/
  stageCmd_t *data;       /*Pointer to the sequence*/
  portCHAR ucMessageID;  /*last message processed in state machine*/
  uint16_t event_reg;    /*last event/status register read from tube*/
}Tubeloop_t;
#endif

typedef struct
{
uint8_t PINSOURCE; 
uint8_t PORTSOURCE;
uint32_t EXTI_LINE;
IRQn_Type IRQ_CH;
}gpio_extint_t;

#ifdef REV_2
typedef enum
{
Heater1,
Heater2,
Heater3,
Heater4,
nExtiGpio
}ExtiGpioTypeDef;
#else
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
#endif
const char *  stageToChar[] = {"Melting","Anealing","Extention","Incubation","Pause","LoopStart","LoopEnd","End"};

#ifdef DEBUG_IF
/* For debug to */
const char *  heater[] = {
/*ADS_DRDY,*/
"M0.1-Tube[1-4]",
"M0.2-Tube[5-8]",
"M0.3-Tube[9-12]",
"M0.4-Tube[13-16]",
"nExtiGpio"
};
#endif
#ifdef REV_2

ExtiGpioTypeDef tube2heater[]={
  0,
  Heater1,
	Heater1,
	Heater1,
	Heater1,
	Heater2,
	Heater2,
	Heater2,
	Heater2,
 	Heater3,
	Heater3,
	Heater3,
	Heater3,
	Heater4,
	Heater4,
	Heater4,
	Heater4
};

typedef struct
{
uint16_t tube_1;
uint16_t tube_2;
uint16_t tube_3;
uint16_t tube_4;
}heater_tubes_t;

heater_tubes_t heater2tube[]={
{1,2,3,4},/*Heater1*/	
{5,6,7,8},/*Heater2*/
{9,10,11,12},/*Heater3*/
{13,14,15,16},/*Heater4*/
};

/*const*/ gpio_extint_t gpio_EXTI_CNF[nExtiGpio+1]={
{GPIO_PinSource3 ,GPIO_PortSourceGPIOC,EXTI_Line3, EXTI3_IRQn    },/*Heater1*/
{GPIO_PinSource4 ,GPIO_PortSourceGPIOC,EXTI_Line4, EXTI4_IRQn    },/*Heater2*/
{GPIO_PinSource11,GPIO_PortSourceGPIOC,EXTI_Line11,EXTI15_10_IRQn},/*Heater3*/
{GPIO_PinSource12,GPIO_PortSourceGPIOC,EXTI_Line12,EXTI15_10_IRQn},/*Heater4*/
{0xFF,0,EXTI0_IRQn}};/*Termination*/
#else
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
{11,12},/*Heater6*/
{9,10},/*Heater5*/
{13,14},/*Heater7*/
{15,16} /*Heater8*/
};


/*const*/ gpio_extint_t gpio_EXTI_CNF[nExtiGpio+1]={
/*{ADS_DRDY_PINSOURCE ,ADS_EXTI_PORTSOURCE,ADS_EXTI_LINE, EXTI15_10_IRQn},*//*ADS DRDY*/
{GPIO_PinSource10,GPIO_PortSourceGPIOC,EXTI_Line10,EXTI15_10_IRQn},/*Heater4*/
{GPIO_PinSource3 ,GPIO_PortSourceGPIOC,EXTI_Line3, EXTI3_IRQn    },/*Heater1*/
{GPIO_PinSource4 ,GPIO_PortSourceGPIOC,EXTI_Line4, EXTI4_IRQn    },/*Heater2*/
{GPIO_PinSource5 ,GPIO_PortSourceGPIOC,EXTI_Line5, EXTI9_5_IRQn  },/*Heater3*/
{GPIO_PinSource11,GPIO_PortSourceGPIOC,EXTI_Line11,EXTI15_10_IRQn},/*Heater5*/
{GPIO_PinSource12,GPIO_PortSourceGPIOC,EXTI_Line12,EXTI15_10_IRQn},/*Heater6*/
{GPIO_PinSource2 ,GPIO_PortSourceGPIOD,EXTI_Line2, EXTI2_IRQn    },/*Heater7*/
{GPIO_PinSource13,GPIO_PortSourceGPIOC,EXTI_Line13,EXTI15_10_IRQn},/*Heater8*/
{0xFF,0,EXTI0_IRQn}};/*Termination*/
#endif
/*Tube state register*/
#ifdef USE_STATICALLY_ALLOCATED_SEQUENCES
/* state, pausePendingState, ucMessageID, LoopStart, LoopIterations, pauseTemp, event_reg, SeqIdx, *data, seq[]*/
/* Save code space - initialize this in code instead - this is 19kB initialized data in FLASH */
Tubeloop_t Tubeloop[nTubes]= {
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0xFFFF,0,0,0,NULL,{},0,End,0}};

#else
/* state, pausePendingState, LoopStart, LoopIterations, pauseTemp, SeqIdx, *data, ucMessageID, event_reg */
Tubeloop_t Tubeloop[nTubes]= {{TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF},
                              {TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,NULL,0,0xFFFF}};
#endif
extern xQueueHandle ModbusQueueHandle;
extern xQueueHandle TubeSequencerQueueHandle;
extern xQueueHandle LogQueueHandle;


/* Private function prototypes -----------------------------------------------*/

void vTimerCallback( xTimerHandle pxTimer );
void ExtIrqDisable(ExtiGpioTypeDef heater);
void ExtIrqEnable(ExtiGpioTypeDef heater);
bool getseq(u8 tubeId,stageCmd_t * data);


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
  
  /*time in mSec*/
  if( xTimers[ tube_idx ] == NULL )
  {
    /* The timer was not created. */
  }
  else
  {
   lStartTickCounters[ tube_idx ]= xTaskGetTickCount();
   DEBUG_PRINTF("Tube[%d]StartTubeTimer Time[%d]Tick[%d]",TubeNum,time,lStartTickCounters[ tube_idx ]);

  
    /* Start the timer.  No block time is specified, and even if one was
    it would be ignored because the RTOS scheduler has not yet been
    started. */
if(time > 1)
{
    if(xTimerChangePeriod( xTimers[ tube_idx ],100 * time,100)!= pdPASS )
    {
    DEBUG_PRINTF("###ERROR TIMER SET Tube[%d]-Time[%d] ",TubeNum,time);
    }
    if( xTimerStart( xTimers[ tube_idx ], 0 ) != pdPASS )
    {
    /* The timer could not be set into the Active state. */
    
    DEBUG_PRINTF("###ERROR TIMER START Tube[%d]-Time[%d] ",TubeNum,time);
    }
}else
{
  DEBUG_PRINTF("###ERROR TIMER START Tube[%d]-Time[%d] CANT BE '0'!! ",TubeNum,time);
}
  }
}


long GetTubeTimeLeft(long TubeNum)
{
  long tube_idx = TubeNum -1;
  long time_consumed;
  long time_left;
  Tubeloop_t *T;
  T = &Tubeloop[TubeNum];
  time_consumed = xTaskGetTickCount() - lStartTickCounters[ tube_idx ];
   DEBUG_PRINTF("TUBE[%d] GetTubeTimeLeft VAL[%d] EST LEFT[%d] PROGRESS[%d]% TICK[%d]-[%d]",TubeNum, T->curr.time,time_consumed/100,((time_consumed)/T->curr.time),xTaskGetTickCount(),lStartTickCounters[ tube_idx ] );
   if( (T->curr.time == 0)||(T->curr.stage==Pause)||(0==lStartTickCounters[ tube_idx ]))/*Ended state - Tube Paused - Waiting for heat up no timer running*/
    {
    time_left = 0;
    }else
    {
      time_left = ((time_consumed)/T->curr.time); 
    }
return time_left;
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
    lStartTickCounters[ tube_idx ] = 0;
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
    //if(GPIO_PinSource11 != p->PINSOURCE){
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
    //}
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
u16 *data_p ;
u16 event = (tube%4)==1?SEQUENCE_EVENT_TUBE1:0|(tube%4)==2?SEQUENCE_EVENT_TUBE2:0|(tube%4)==3?SEQUENCE_EVENT_TUBE3:0|(tube%4)==0?SEQUENCE_EVENT_TUBE4:0;

if (reg == SETPOINT_REG) /*Make virtual response if setpoint is set*/
{
  msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsRes)+3*sizeof(u16));
  msg->ucMessageID=READ_MODBUS_REGS_RES;
  p=(ReadModbusRegsRes *)msg->ucData;
  data_p = (u16*)p->data;
  p->slave=(tube/4)*3+1;
  p->addr=EVENT_REG;
  *(data_p) = event<<8;
  *(data_p+1) = ((*data&0xFF)<<8)|(*data>>8);/*Tube 1 on heater*/
  *(data_p+2) = ((*data&0xFF)<<8)|(*data>>8);/*Tube 2 on heater*/
  //memcpy((p->data+1), data, datasize*sizeof(u16));

  //memcpy(p->data, data, datasize);
  p->datasize=3;
  p->resultOk=NO_ERROR;
  xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
  DEBUG_IF_PRINTF("Tube[%d] MODBUS WRITE_REG[%d]data[%d][%d][%d]",tube,p->addr,*(data_p),*(data_p+1),*(data_p+2));
}
#else
  WriteModbusRegsReq *p;
  msg=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+datasize*sizeof(u16));
  if(NULL == msg)
  {
    DEBUG_IF_PRINTF("Malloc failed! WriteTubeHeaterReg Tube[%d]MODBUS WRITE_REG ADR[%d]",tube,reg);
  }
  else
  {
    *data=((*data&0xFF)<<8)|(*data>>8);//*************OBS only first element is transformed *********/
    msg->ucMessageID=WRITE_MODBUS_REGS;
    p=(WriteModbusRegsReq *)msg->ucData;
    p->slave=tube/*+0x02*/;
    p->addr=reg;
    memcpy(p->data, data, datasize*sizeof(u16));
    p->datasize=datasize;
    p->reply=TubeSequencerQueueHandle;
    xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
    DEBUG_IF_PRINTF("Tube[%d]MODBUS WRITE_REG ID[%d] ADR[%d] data[%x]SIZE[%d]",tube, tube,p->addr,(((u16)(p->data[0])<<8)|(p->data[1])),p->datasize);
  }
#endif
}

void ReadTubeHeaterReg(u8 tube, u16 reg, u16 datasize, xQueueHandle xQueue, bool from_isr)
{
#if defined (SIMULATE_HEATER) 
  long lArrayIndex;
  xMessage *msg;
  long *p;
  ReadModbusRegsRes *preg;
  u16 *data_p ;
  u16 event = (tube%4)==1?SEQUENCE_EVENT_TUBE1:0|(tube%4)==2?SEQUENCE_EVENT_TUBE2:0|(tube%4)==3?SEQUENCE_EVENT_TUBE3:0|(tube%4)==0?SEQUENCE_EVENT_TUBE4:0;
  signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  msg = pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsRes));
  msg->ucMessageID = READ_MODBUS_REGS_RES;
  preg = (ReadModbusRegsRes *)msg->ucData;
  preg->slave = (tube/4)*3+1;;
  preg->addr = reg;
  preg->datasize = 1;
  data_p = (u16*)preg->data;
  *(data_p) = event<<8;
  preg->resultOk =  NO_ERROR;
 // Tubeloop[tube].curr.stage = End;
  Tubeloop[tube].state = TUBE_IDLE;/*Tube ready for operation*/
  DEBUG_IF_PRINTF("Tube[%d]MODBUS READ_REG[%d] ",tube,reg);
  xQueueSend(TubeSequencerQueueHandle,&msg,portMAX_DELAY);
  if( xHigherPriorityTaskWoken )
  {
  // Actual macro used here is port specific.
  // taskYIELD_FROM_ISR(); /*MIGHT BE USED TO FIX RESCHEDULING TFK*/
  }
  
  DEBUG_IF_PRINTF("Tube[%d]MODBUS READ_REG ID[%d] ADR[%d] SIZE[%d] CALLER:%s",tube, preg->slave,preg->addr, preg->datasize, (from_isr == TRUE) ?"ISR":"NORM");
#else
  xMessage *msg;
  ReadModbusRegsReq *p;
  portBASE_TYPE taskWoken = pdTRUE;
  msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq));
  if(NULL == msg)
  {
    DEBUG_IF_PRINTF("Malloc failed! ReadTubeHeaterReg Tube[%d]MODBUS WRITE_REG ADR[%d]",tube,reg);
  }
  else
  {
    msg->ucMessageID=READ_MODBUS_REGS;
    p=(ReadModbusRegsReq *)msg->ucData;
    p->slave=tube/*+0x02*/;
    p->addr=reg;
    p->datasize=datasize;
    p->reply=xQueue;
    if (from_isr == TRUE)
    {
     assert_param(xQueueSendFromISR(ModbusQueueHandle,&msg,&taskWoken) == pdPASS);
    }else
    {
      assert_param(xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY)== pdPASS);
    }
    DEBUG_IF_PRINTF("Tube[%d]MODBUS READ_REG ID[%d] ADR[%d] SIZE[%d] CALLER:%s",tube, p->slave,p->addr, p->datasize, (from_isr == TRUE) ?"ISR":"NORM");
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
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

  /* GPIOD Configuration: Channel 3 and 4 as alternate function push-pull */
 // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
 // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
 // GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* GPIOC Configuration: Channel 1, 2 and 3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4/* | GPIO_Pin_5 | GPIO_Pin_10*/ | GPIO_Pin_11 | GPIO_Pin_12 /*| GPIO_Pin_13*/;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}


#ifdef REV_2
void EXTI_Handler(void)
{
 vTraceStoreISRBegin(3);
  ExtiGpioTypeDef ExtiGpio = Heater1; 
  while(ExtiGpio < nExtiGpio) /*Alwlays starts at Heater1 since */
  {
    if (SET == EXTI_GetFlagStatus(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE))
    {
      DEBUG_IF_PRINTF("INTERRUPT-READ STATUS ON HEATER[%s]",heater[ExtiGpio]);
      ReadTubeHeaterReg(heater2tube[ExtiGpio].tube_1,EVENT_REG,5,TubeSequencerQueueHandle,TRUE);			//ToDo: Also check tube_1!!!
      EXTI_ClearITPendingBit(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE);
    }
    ExtiGpio++;
  }
  vTraceStoreISREnd();
  portEND_SWITCHING_ISR( pdTRUE);
}

#else
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
      ReadTubeHeaterReg(7,EVENT_REG,5, TubeSequencerQueueHandle, TRUE);
      Heater4_irq_handled = TRUE;
      DEBUG_IF_PRINTF("INTERRUPT-READ STATUS ON HEATER[%s]",heater[Heater4]);
    }
  EXTI_ClearITPendingBit(ADS_EXTI_LINE);
  }
  while(ExtiGpio < nExtiGpio) /*Alwlays starts at Heater1 since */
  {
    if (SET == EXTI_GetFlagStatus(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE))
    {
      DEBUG_IF_PRINTF("INTERRUPT-READ STATUS ON HEATER[%s]",heater[ExtiGpio]);
      ReadTubeHeaterReg(heater2tube[ExtiGpio].tube_2,EVENT_REG,5,TubeSequencerQueueHandle,TRUE);			//ToDo: Also check tube_1!!!
      EXTI_ClearITPendingBit(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE);
    }
    ExtiGpio++;
  }
  vTraceStoreISREnd();
  portEND_SWITCHING_ISR( pdTRUE);

}
#endif
/* ---------------------------------------------------------------------------*/
int create_seq(long TubeId, uint16_t temp, int Nstages )
{
  Tubeloop_t *pTubeloop = &Tubeloop[TubeId];
#ifndef USE_STATICALLY_ALLOCATED_SEQUENCES
  if(NULL != pTubeloop->data)
  {
    vPortFree(pTubeloop->data);
  }
  pTubeloop->data = pvPortMalloc(sizeof(stageCmd_t)*Nstages);
#else
  pTubeloop->data = &Tubeloop[TubeId].seq[0];
#endif
  pTubeloop->pauseTemp = temp;
  pTubeloop->pausePendingState = TUBE_P_NORM;
  //DEBUG_PRINTF("###Size of Tubeloop_t %d ; Size of stageCmd_t %d ; Nof stages %d", sizeof(Tubeloop_t),sizeof(stageCmd_t),Nstages);
  return (int)pTubeloop->data;
}

/* ---------------------------------------------------------------------------*/
bool insert_state_to_seq(long TubeId, char stageChar, uint32_t time, uint16_t temp )
{
  bool result = TRUE;
  Tubeloop_t *pTubeloop = &Tubeloop[TubeId];
  stageCmd_t *TSeq = &(pTubeloop->data[pTubeloop->SeqIdx]);
  pTubeloop->pauseTemp = 300;/*hardcoded pausetemp to 30 C*/
  pTubeloop->pausePendingState = TUBE_P_NORM;
  if(Tubeloop[TubeId].state == TUBE_IDLE)/*Check if sequence is already running on tube*/
  {
    if(time != 0)
    {
      TSeq->time = time;
      TSeq->temp = temp;
      // gdi_parse_command() converts all characters to lowercase
      if('m' == stageChar)      { TSeq->stage = Melting;   }
      else if('a' == stageChar) { TSeq->stage = Annealing; }
      else if('e' == stageChar) { TSeq->stage = Extension; }
      else if('i' == stageChar) { TSeq->stage = Incubation;}
      pTubeloop->SeqIdx++;
    } else { /*Last entry in sequence for tube if time is = 0*/
      TSeq->time = 0;
      TSeq->temp = 0;
      TSeq->stage = End;
      pTubeloop->SeqIdx = 0;/*Set seq ready to start*/
    }
  }else
  {
    result = FALSE;
  }
return result;
}


void start_lid_heating()
{
	xMessage *msg;
	long *p;
	msg=pvPortMalloc(sizeof(xMessage));
	if(msg)
	{
		msg->ucMessageID=START_LID_HEATING;
		p=(long *)msg->ucData;
		xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
	}
}

void stop_lid_heating()
{
	xMessage *msg;
	long *p;
	msg=pvPortMalloc(sizeof(xMessage));
	if(msg)
	{
		msg->ucMessageID=STOP_LID_HEATING;
		p=(long *)msg->ucData;
		xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
	}
}

/* ---------------------------------------------------------------------------*/
bool start_tube_seq( long TubeId)
{  
  bool result = TRUE;
  Tubeloop_t *pTubeloop = &Tubeloop[TubeId];
  pTubeloop->data = &Tubeloop[TubeId].seq[0];
  //DEBUG_PRINTF("start_tube_seq");
  //Tubeloop[TubeId].state = TUBE_IDLE;
  Tubeloop[TubeId].SeqIdx = 0;
  if( (Tubeloop[TubeId].state == TUBE_PAUSED) || Tubeloop[TubeId].state == TUBE_IDLE )
  {
    long *p;
    xMessage *msg;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
    
    //  DEBUG_PRINTF("start_tube_seq-data valid");
    if(msg)
    {
      msg->ucMessageID=START_TUBE_SEQ;/*INIT_TUBE_SEQ;*/
      p=(long *)msg->ucData;
      *p=TubeId;
      xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
      //  DEBUG_PRINTF("start_tube_seq-send msg");
    }
  }else if(Tubeloop[TubeId].pausePendingState == TUBE_P_PAUSE_REQUESTED) /*waiting for pause but did regret it*/
  {
    Tubeloop[TubeId].pausePendingState = TUBE_P_NORM;
    DEBUG_PRINTF("Tube[%d] pause aborted REQ[%d]",TubeId,Tubeloop[TubeId].pausePendingState);
  }
  else 
  {
    result = FALSE;
  }
  return result;
}


/* ---------------------------------------------------------------------------*/
bool stop_tube_seq( long TubeId)
{
  u16 data;
  long *p;
  xMessage *new_msg;
  bool result = TRUE;
  stageCmd_t STAGE;
   stageCmd_t *TSeq = &STAGE;

  stop_lid_heating();
  if((TUBE_INIT == Tubeloop[TubeId].state) || (TUBE_IDLE == Tubeloop[TubeId].state))
  {
    result = FALSE;
  }
  else
  {
  	if(Tubeloop[TubeId].state == TUBE_WAIT_TIME)
    {
      StopTubeTimer(TubeId);
    }
    /*Set idle mode for tube to stop heating*/
    data = SET_IDLE_MODE;
    WriteTubeHeaterReg(TubeId,TUBE_COMMAND_REG,&data,sizeof(data)/2); 
    DEBUG_SEQ_PRINTF("Tube[%d]@%s END OF SEQUENCE FOR TUBE",
    TubeId,Tubeloop[TubeId].state);
    Tubeloop[TubeId].SeqIdx = 0;
    Tubeloop[TubeId].state = TUBE_IDLE;
    Tubeloop[TubeId].pausePendingState = TUBE_P_NORM;

/*Set idle mode for tube to stop heating*/
  Tubeloop[TubeId].SeqIdx = 0;

while( (getseq(TubeId,  TSeq) == TRUE));/*empty buffer*/
 emptyLog(TubeId);
 Tubeloop[TubeId].curr.seq_num = 0;
#ifdef USE_DEVELOPMENT_LOGGING
    new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
    if(new_msg)
    {
      new_msg->ucMessageID=END_LOG;
      p=(long *)new_msg->ucData;
      *p=TubeId;
      xQueueSend(LogQueueHandle, &new_msg, portMAX_DELAY);
    }
#endif
  }
  return result;
}

/* ---------------------------------------------------------------------------*/
#ifdef USE_PAUSE_FEATURE
void pause_tube_seq(void)
{
  int TubeId;
  for(TubeId=1; TubeId<17; TubeId++)
  {
    Tubeloop[TubeId].pausePendingState = TUBE_P_PAUSE_REQUESTED;
  }
}
void pause_tube_state(long TubeId)
{
  Tubeloop[TubeId].pausePendingState = TUBE_P_PAUSE_REQUESTED;
  DEBUG_PRINTF("Tube[%d] - Pause REQ[%d]",TubeId,Tubeloop[TubeId].pausePendingState);
}

/* ---------------------------------------------------------------------------*/
/* Send a pause ended message to all tubes paused */
void continue_tube_seq(void)
{
  long *pucData;
  long TubeId;
  xMessage *msg;
  for(TubeId=1; TubeId<17; TubeId++)
  {    
    Tubeloop[TubeId].pausePendingState = TUBE_P_NORM; // Pause no longer requested
    if((TUBE_PAUSED == Tubeloop[TubeId].state) || (TUBE_WAIT_P_TEMP == Tubeloop[TubeId].state))
    { // Tubes in pause needs to move on. SeqIdx is normally incremented on timer expiry but not when entering states.
      Tubeloop[TubeId].SeqIdx++; /*Going to next stage in sequence*/
      msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
      if(msg)
      {
        msg->ucMessageID = NEXT_TUBE_STAGE; //Hvordan siger man at pausen er slut??
        pucData=(long *)msg->ucData;
        *pucData=TubeId;
        xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);      
        Tubeloop[TubeId].pausePendingState = TUBE_P_NORM;
      }
    }
  }
}

#endif
/* ---------------------------------------------------------------------------*/
void log_tube_seq(long TubeId,bool enable )
 {
   long *p;
   xMessage *new_msg;
   /*Start monitoring temperature on the tube*/
   new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
   if(new_msg)
   {
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
 }
 
 
/* ---------------------------------------------------------------------------*/
void set_log_interval( long Log_Interval)
 {
  u16 data;
  long *p;
  xMessage *new_msg;
  new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
  if(new_msg)
  {
    new_msg->ucMessageID=SET_LOG_INTERVAL;
    p=(long *)new_msg->ucData;
    *p=Log_Interval;
    xQueueSend(LogQueueHandle, &new_msg, portMAX_DELAY);
  }
}

/* ---------------------------------------------------------------------------*/
char *get_system_state(char *poutText)
{
  int TubeId;
  int pausePending = 0;
  int paused = 0;
  int running = 0;
  int idle = 0;
  int incubating = 0;

  for(TubeId = 1; TubeId < 17; TubeId++)
  {
    if (Incubation == Tubeloop[TubeId].seq[Tubeloop[TubeId].SeqIdx].stage)
    {
      incubating++;
    }
    else
    {
      if((TUBE_WAIT_TEMP == Tubeloop[TubeId].state) || (TUBE_WAIT_TIME==Tubeloop[TubeId].state) || (TUBE_WAIT_P_TEMP==Tubeloop[TubeId].state))
        { running++; }
      else if(TUBE_PAUSED == Tubeloop[TubeId].state)
        { paused++; }
    }
    if(TUBE_P_PAUSE_REQUESTED == Tubeloop[TubeId].pausePendingState)
    {
      pausePending++;
    }
  }
  DEBUG_SEQ_PRINTF("I%d R%d P%d PP%d i%d", idle,running,paused,pausePending,incubating);  
//#### What do do with incubating tubes here???
  if(0 < running) {
    if(0 == pausePending) {
      strcpy(poutText,"Running");
    } else {
      strcpy(poutText,"PausePending");
    }
  }
  else
  { // No tubes are running
    if(0 < paused) {
      strcpy(poutText,"Paused");
    } else {
      strcpy(poutText,"Stopped");
    }
  }  
  return poutText;
}

/* ---------------------------------------------------------------------------*/
char * get_tube_state(long TubeId, char *poutText)
{
#ifdef DEBUG
  char tube_state[10]="IDLE";
#endif
    char state[20]="idle";
    char progress[5]="50";
    char stage_nr[5]="3";
    char log[50]="2070,18";
    

    *poutText = 0;

    if(Tubeloop[TubeId].data != NULL)
    {
      if(Tubeloop[TubeId].state==TUBE_OUT_OF_DATA)
      {
        strcpy(state,"waiting for data");
        //sprintf(progress,"0");
        strcpy(progress, "0");
      }
      else if((Tubeloop[TubeId].state==TUBE_WAIT_TEMP)||(Tubeloop[TubeId].state==TUBE_WAIT_TIME)||(Tubeloop[TubeId].state==TUBE_WAIT_P_TEMP))
      {
        // Tube is considered running untill the pause temperature is actually reached.
        //sprintf(progress,"%d",GetTubeTimeLeft(TubeId));
        Itoa(GetTubeTimeLeft(TubeId), progress);
        if(tubequefree(TubeId) > 0)
          {
            strcpy(state,"running waiting for data");
            
          }
          else
          {
            strcpy(state,"running");
            
          }
      }
      else if(Tubeloop[TubeId].state == TUBE_PAUSED)
      {
        strcpy(state,"paused");
#ifdef DEBUG
        strcpy(tube_state,"PAUSED");
#endif
      }
      else if(Tubeloop[TubeId].state == TUBE_IDLE)
      {
        strcpy(state,"idle");
#ifdef DEBUG
        strcpy(tube_state,"IDLE");
#endif
      }
      //sprintf(stage_nr,"%d",Tubeloop[TubeId].curr.seq_num);
      Itoa(Tubeloop[TubeId].curr.seq_num, stage_nr);
      strcpy(poutText,"OK<state=\"");
      strcat(poutText,state);
      if(Tubeloop[TubeId].state != TUBE_IDLE)
      {
        strcat(poutText,"\";progress=");
        strcat(poutText,progress);
        strcat(poutText,";stage_number=");
        strcat(poutText,stage_nr);
        //strncat(poutText,";log={",strlen(";log={"));
        getLog(poutText,TubeId);
      }
      strcat(poutText,">");
   
      DEBUG_PRINTF("T%d: %s @Step %d state:%s Stage:%s PState:%d CURR temp:%d time:%d LAST MSG:%s EVENT:%x",
        TubeId, tube_state, Tubeloop[TubeId].SeqIdx+1, tube_states[Tubeloop[TubeId].state], 
        stageToChar[Tubeloop[TubeId].curr.stage], Tubeloop[TubeId].pausePendingState,
        Tubeloop[TubeId].data[Tubeloop[TubeId].SeqIdx].temp, Tubeloop[TubeId].data[Tubeloop[TubeId].SeqIdx].time,
        signals_txt[Tubeloop[TubeId].ucMessageID], Tubeloop[TubeId].event_reg);
    }
    else
    {
      strcpy(poutText,"OK<state=\"idle\";>");
      DEBUG_PRINTF("T%d:NO VALID SEQ FOUND State for Tube SEQ_ID:%d state:%s LAST MSG:%s EVENT:%x",
        TubeId,Tubeloop[TubeId].SeqIdx, tube_states[Tubeloop[TubeId].state], signals_txt[Tubeloop[TubeId].ucMessageID],
        Tubeloop[TubeId].event_reg);
    }

  return poutText;
}
#ifdef MAIN_IF_REV2  


int RDIdx;              /*Read Pointer*/ 
int WRIdx;              /*Write Pointer */
stageCmd_t *data;       /* Pointer to the sequence*/
stageCmd_t seq[STAGES_PER_SEQ];   /* The sequence */


/* ---------------------------------------------------------------------------*/
void tubeinitQueue()
{
  int i;
  //Tubeloop_t *pTubeloop = &Tubeloop[TubeId];
  for(i=0; nTubes > i; i++)
  {
   Tubeloop_t *pTubeloop = &Tubeloop[i];
    pTubeloop->tail= 0; 
    pTubeloop->head= 0;
    // ####Fill in debug padding
    //logDataQueue[i].padding = 0xA5A5A5A5;
  }
}

/* ---------------------------------------------------------------------------*/

// Enqueue element. Return pointer to element so it can be written to.
stageCmd_t * tubeenqueue(Tubeloop_t * pQueue)
{
  if((pQueue->tail - STAGES_PER_SEQ) == pQueue->head) { return NULL; } // Return null if queue is full
  pQueue->tail++;
//  DEBUG_PRINTF("tubeenqueue: t:%d h:%d FREE:%d",pQueue->tail% STAGES_PER_SEQ,pQueue->head% STAGES_PER_SEQ,STAGES_PER_SEQ-(pQueue->tail -  pQueue->head));
  return &pQueue->seq[pQueue->tail % STAGES_PER_SEQ];
}

/* ---------------------------------------------------------------------------*/
// Dequeue element. Return pointer to element so it can be read from.
stageCmd_t * tubedequeue(Tubeloop_t * pQueue)
{
  if(pQueue->tail == pQueue->head) {return NULL;}  // Return null if queue is empty
  else
  {
    pQueue->head++;
 //   DEBUG_PRINTF("tubedequeue: t:%d h:%d FREE:%d",pQueue->tail% STAGES_PER_SEQ,pQueue->head% STAGES_PER_SEQ, STAGES_PER_SEQ-(pQueue->tail - pQueue->head));
    return &pQueue->seq[pQueue->head % STAGES_PER_SEQ];
  }
}
/* ---------------------------------------------------------------------------*/
// Dumpqueue element. Return pointer to element so it can be read from.
void tubedumpqueue(Tubeloop_t * pQueue)
{
int i;
  if(pQueue->tail == pQueue->head) {return ;}  // Return null if queue is empty
  else
  {
 // DEBUG_PRINTF("QUE tubedump head:%d tail:%d FREE:%d",pQueue->head%STAGES_PER_SEQ,pQueue->tail%STAGES_PER_SEQ,STAGES_PER_SEQ-(pQueue->tail - pQueue->head));
    for(i=pQueue->head+1;i<=pQueue->tail;i++)
    {
      stageCmd_t stage =pQueue->seq[i%STAGES_PER_SEQ];
    
       // DEBUG_PRINTF("QUE tubedump:%d temp:%d time:%d stage:%d ",i% STAGES_PER_SEQ,stage.temp,stage.time,stage.stage, STAGES_PER_SEQ-(pQueue->tail - pQueue->head));
    }
  }
}
int tubequefree(long tubeid)
{
  Tubeloop_t * pQueue = &Tubeloop[tubeid-1];

return (int)STAGES_PER_SEQ-(pQueue->tail - pQueue->head);
}
/* ---------------------------------------------------------------------------*/
bool tubedataQueueAdd(u8 tubeId,u16 seq_num, char state, stageCmd_t *data)
{
bool result = FALSE;
  int i;
  stageCmd_t * poutData;
    Tubeloop_t *pTubeloop = &Tubeloop[tubeId];
      long TubeId;
  xMessage *msg;
  long *pucData;
//  SERIAL_String("tubedataQueueAdd:()");
//   DEBUG_PRINTF("tubedataQueueAdd: %d",tubeId);

  taskENTER_CRITICAL(); //push irq state
           //   DEBUG_PRINTF("ENQUE:Tube:%d:TEMP %d.%02dC @ TIME %d.%02dsecs STATE:%d ",tubeId,data->temp/10,data->temp%10,data->time/10,data->time%10,data->stage);  
  if(NULL != (poutData = tubeenqueue(&Tubeloop[tubeId-1]))) //tubeId=[1..16],idx=[0..15]
  {
DEBUG_PRINTF("T%d:@QUE ADD:TEMP %d.%02dC @ TIME %d.%02dsecs SEQ_ID:%d STATE:%c FREE:%d CURR T ST:%d ",tubeId,data->temp/10,data->temp%10,data->time/10,data->time%10,seq_num,state,tubequefree(tubeId),pTubeloop->state);   
   poutData->temp  = data->temp;
   poutData->time  = data->time;
   poutData->seq_num = seq_num;
   if('m' ==  (char)state)      { poutData->stage = Melting;   }
      else if('a' ==  (char)state) { poutData->stage = Annealing; }
      else if('e' ==  (char)state) { poutData->stage = Extension; }
      else if('i' ==  (char)state) { poutData->stage = Incubation;}
      if((0 ==  data->time)&(0 ==  data->temp)) { poutData->stage = End;}
  // poutData->stage = data->stage;
// DEBUG_PRINTF("%s-%d-%c",tube_sta[poutData->stage],poutData->stage,state);
   result = TRUE;
  }else
  {
   result = FALSE;
   DEBUG_PRINTF("T%d:-QUE ADD FAILED");
  }


  taskEXIT_CRITICAL();
  /*Debug*/
  tubedumpqueue(&Tubeloop[tubeId-1]);
  if(pTubeloop->state == TUBE_OUT_OF_DATA)/*Auto restart running sequence after added more data*/
  {
   msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
      if(msg)
      {
        msg->ucMessageID = NEXT_TUBE_STAGE; //Spring til den n�ste stage der lige er lagt p� k�??
        pucData=(long *)msg->ucData;
        *pucData=tubeId;
        xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);      
        }
  
  }

return result;
}

/* ---------------------------------------------------------------------------*/
/* at@gdi:seq_cmd(getlog)\n                                                   */
/* Response:                                                                  */
/* LOG:(tube1:<tempX>,<tempX>,<tempX>,<tempX>,<tempX>,<tempX>,<tempX>;        */
/*      tube2:<tempX>,<tempX>,<tempX>,<tempX>;tube12:<tempX>,<tempX>;)       */
/*                                                                            */
/* Where <tempX> is the logged temperature in deci C and printed in HEX       */ 
/* ---------------------------------------------------------------------------*/
bool  getseq(u8 tubeId,  stageCmd_t *data)
{
bool res = FALSE;
 stageCmd_t *quedata;
  int i = 0;
    Tubeloop_t *pTubeloop = &Tubeloop[tubeId];
  //DEBUG_PRINTF("DEQUE %d",tubeId);
    taskENTER_CRITICAL(); //push irq state #### Kan critical section laves mindre!
    // Send all available log elements for each tube
    if(NULL != (quedata = tubedequeue(&Tubeloop[tubeId-1])) ) //idx=[0..15]
    {
    memcpy(data,quedata,sizeof(stageCmd_t));
    res = TRUE;
    DEBUG_PRINTF("Tube[%d] SEQ ID[%d]QUE GET:TEMP %d.%02dC @ TIME %d.%02dsecs STATE:%d ",tubeId,pTubeloop->SeqIdx,data->temp/10,data->temp%10,data->time/10,data->time%10,data->stage);   
    }else
      {
     res = FALSE;
      }
    taskEXIT_CRITICAL();
 return res; 
}


#endif

/* ---------------------------------------------------------------------------*/
/* Handle event for one tube */
/* ---------------------------------------------------------------------------*/
void TubeEventHandler (long TubeId, int event, xMessage *msg)
{
  ReadModbusRegsRes *preg;
  u16 modbus_addr,modbus_id;
  u16 modbus_data[EVENT_DATA_SIZE];
  bool modbus_result;
  long i=0;

  preg=(ReadModbusRegsRes *)msg->ucData;

  modbus_addr = preg->addr;
  /*modbus_data =  preg->data[1];*/
  modbus_result = preg->resultOk;

  Tubeloop_t *pTubeloop = &Tubeloop[TubeId];
  for(i=0;i<(preg->datasize);i++)
  {
    modbus_data[i] =(((u16)(preg->data[i*2])<<8)|(preg->data[(i*2)+1]));
  }
#ifdef DEBUG
  // If not logging event output debug data
  if(((event & LOGGING_READY)==FALSE))
  {
    DEBUG_PRINTF("Tube[%d][%x]-EVENT %s - ST:%s",TubeId,modbus_data[0] ,(((event & SEQUENCE_EVENT)==TRUE)?"SEQ_EVENT":" "),tube_states[pTubeloop->state]);
  }
#endif
#ifndef REV_2
  if (TubeId == 7)
  {
    Heater4_irq_handled == FALSE; /*Now we have read the content of the event register and are ready to recieve a new IRQ from Heater4*/
  }
#endif

  /* Handle "Temperature reached" event */
  if( ((pTubeloop->state == TUBE_WAIT_TEMP) || (pTubeloop->state == TUBE_WAIT_P_TEMP)) && 
      (event & SEQUENCE_EVENT) )
  {
    modbus_id = TubeId%4;
    if(modbus_id == 0) { modbus_id = 4; }
#ifdef USE_PAUSE_FEATURE
    if(TUBE_WAIT_P_TEMP == Tubeloop[TubeId].state) {
      Tubeloop[TubeId].state = TUBE_PAUSED;
      pTubeloop->pausePendingState = TUBE_P_NORM; //Pause is not pending anymore
      //do we check if all tubes are in pause and notify Linuxbox
      DEBUG_SEQ_PRINTF("Tube[%d] Pause temp reached:%d.%01dC - paused!", TubeId, modbus_data[modbus_id]/10, modbus_data[modbus_id]%10);
    }
    else
#endif
    {
      DEBUG_SEQ_PRINTF("\n\rTube[%d] Step %d Temp reached:%d.%01dC Start timer %d.%01d Sec, stage:%s\n\r", 
                   TubeId, pTubeloop->SeqIdx+1, modbus_data[modbus_id]/10, modbus_data[modbus_id]%10, 
                   pTubeloop->curr.time/10, pTubeloop->curr.time%10, stageToChar[pTubeloop->curr.stage]);/*ASCII 155 norm 167*/
      StartTubeTimer(TubeId,pTubeloop->curr.time);
      pTubeloop->state = TUBE_WAIT_TIME;
    }
  }
  if(Tubeloop[TubeId].state == TUBE_NOT_INITIALIZED)/*No errors on current tube, set it to IDLE so its ready for use*/
  {
    Tubeloop[TubeId].state = TUBE_INIT;
    modbus_data[0]=1;
    #if defined(SIMULATE_HEATER)
    ReadTubeHeaterReg(TubeId,EVENT_REG,1,TubeSequencerQueueHandle, FALSE);
    #else
//    if(tube2heater[TubeId] != Heater3){
      ExtIrqEnable(tube2heater[TubeId]);
  //  }
    DEBUG_PRINTF("ENABLE IRQ ON HEATER[%d]",tube2heater[TubeId]);
    WriteTubeHeaterReg(TubeId,EVENT_REG,&modbus_data[0], 1); /*******TEST to force the heater to create an event*/
    #endif
  }
      
  if((LOGGING_READY) & event)
  { //Request a read of the logged data from tube. The result is sent to logging task 
    ReadTubeHeaterReg(TubeId, DATA_LOG, DATA_LOG_SIZE+1, LogQueueHandle, FALSE);
  }
  
}

/* ---------------------------------------------------------------------------*/
/* Read event register and sort out events per tube and handle the events     */
/* for each tube seperately
/* ---------------------------------------------------------------------------*/
void HeaterEventHandler (ReadModbusRegsRes *preg, xMessage *msg)
{
  int i;
  int n;
  int event;
  u16 modbus_data[EVENT_DATA_SIZE];
  long TubeId = preg->slave;
  n = preg->datasize <=EVENT_DATA_SIZE ? preg->datasize : EVENT_DATA_SIZE;
  
  for(i=0;i<(n);i++)
  {
    modbus_data[i] =(((u16)(preg->data[i*2])<<8)|(preg->data[(i*2)+1]));
  }
  Tubeloop[TubeId].event_reg = modbus_data[0];
#if 1 // event debug out
  DEBUG_SEQ_PRINTF("T%d:EVENT[%04x]:%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s",
        TubeId,modbus_data[0],
        ((modbus_data[0] & SEQUENCE_EVENT_TUBE1)? "S_EV_T1 "  : " "),
        ((modbus_data[0] & SEQUENCE_EVENT_TUBE2)? "S_EV_T2 "  : " "),
        ((modbus_data[0] & SEQUENCE_EVENT_TUBE3)? "S_EV_T3 "  : " "),
        ((modbus_data[0] & SEQUENCE_EVENT_TUBE4)? "S_EV_T4 "  : " "),
        ((modbus_data[0] & INIT_HW_ERROR_TUBE1) ? "HW_ERR_T1 ": " "),
        ((modbus_data[0] & INIT_HW_ERROR_TUBE2) ? "HW_ERR_T2 ": " "),
        ((modbus_data[0] & INIT_HW_ERROR_TUBE3) ? "HW_ERR_T3 ": " "),
        ((modbus_data[0] & INIT_HW_ERROR_TUBE4) ? "HW_ERR_T4 ": " "),
        ((modbus_data[0] & TUBE1_NOT_PRESENT)   ? "NO_T1 "    : " "),
        ((modbus_data[0] & TUBE2_NOT_PRESENT)   ? "NO_T2 "    : " "),
        ((modbus_data[0] & TUBE3_NOT_PRESENT)   ? "NO_T3 "    : " "),
        ((modbus_data[0] & TUBE4_NOT_PRESENT)   ? "NO_T4 "    : " "),
        ((modbus_data[0] & TUBE1_LOGGING_READY) ? "LogT1 "    : " "),
        ((modbus_data[0] & TUBE2_LOGGING_READY) ? "LogT2 "    : " "),
        ((modbus_data[0] & TUBE3_LOGGING_READY) ? "LogT3 "    : " "),
        ((modbus_data[0] & TUBE4_LOGGING_READY) ? "LogT4 "    : " "));
#endif
  event = 0;
  event |= (modbus_data[0] & SEQUENCE_EVENT_TUBE1) ? SEQUENCE_EVENT : 0; 
  event |= (modbus_data[0] & INIT_HW_ERROR_TUBE1)  ? INIT_HW_ERROR  : 0; 
  event |= (modbus_data[0] & TUBE1_NOT_PRESENT)    ? NOT_PRESENT    : 0; 
  event |= (modbus_data[0] & TUBE1_LOGGING_READY)  ? LOGGING_READY  : 0; 
  if(event) TubeEventHandler (TubeId, event, msg);
  event = 0;
  event |= (modbus_data[0] & SEQUENCE_EVENT_TUBE2) ? SEQUENCE_EVENT : 0; 
  event |= (modbus_data[0] & INIT_HW_ERROR_TUBE2)  ? INIT_HW_ERROR  : 0; 
  event |= (modbus_data[0] & TUBE2_NOT_PRESENT)    ? NOT_PRESENT    : 0; 
  event |= (modbus_data[0] & TUBE2_LOGGING_READY)  ? LOGGING_READY  : 0; 
  if(event) TubeEventHandler (TubeId + 1, event, msg);
  event = 0;
  event |= (modbus_data[0] & SEQUENCE_EVENT_TUBE3) ? SEQUENCE_EVENT : 0; 
  event |= (modbus_data[0] & INIT_HW_ERROR_TUBE3)  ? INIT_HW_ERROR  : 0; 
  event |= (modbus_data[0] & TUBE3_NOT_PRESENT)    ? NOT_PRESENT    : 0; 
  event |= (modbus_data[0] & TUBE3_LOGGING_READY)  ? LOGGING_READY  : 0; 
  if(event) TubeEventHandler (TubeId + 2, event, msg);
  event = 0;
  event |= (modbus_data[0] & SEQUENCE_EVENT_TUBE4) ? SEQUENCE_EVENT : 0; 
  event |= (modbus_data[0] & INIT_HW_ERROR_TUBE4)  ? INIT_HW_ERROR  : 0; 
  event |= (modbus_data[0] & TUBE4_NOT_PRESENT)    ? NOT_PRESENT    : 0; 
  event |= (modbus_data[0] & TUBE4_LOGGING_READY)  ? LOGGING_READY  : 0; 
  if(event) TubeEventHandler (TubeId + 3, event, msg);

  if(Tubeloop[TubeId].state == TUBE_NOT_INITIALIZED){
    TubeEventHandler (TubeId, event, msg);

    }

}

/* ---------------------------------------------------------------------------*/
/* Handle any register read other than event register */
/* ---------------------------------------------------------------------------*/

void TubeMessageHandler (long TubeId, xMessage *msg)
{
  ReadModbusRegsRes *preg;
  if(preg->addr == SETPOINT_REG)
  {
    //DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] SetPoint[%d]", TubeId, tube_states[Tubeloop[TubeId].state],
    //              signals_txt[msg->ucMessageID], modbus_data[0]/10);
  }
  
  // DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s]ADDR[%d]size[%d]data[%x]STATUS[%s]",
  //   TubeId, tube_states[Tubeloop[TubeId].state], signals_txt[msg->ucMessageID],
  //   preg->addr, preg->datasize,/*(((u16)(preg->data[0])<<8)|(preg->data[1]))*/modbus_data[0],
  //   (preg->resultOk==NO_ERROR)?"PASS":"FAIL");
}

/* ---------------------------------------------------------------------------*/
/* TubeStateHandler sets up the next stage in a sequence, temperature and time*/
/* TubeStateHandler is called on reception of START_TUBE_SEQ and              */
/* NEXT_TUBE_STAGE messages  */
void TubeStateHandler(long TubeId, xMessage *msg)
{
  xMessage *new_msg;
  long *p;
  u16 data;
  int a = 0;
  uint16_t i = 0;
  Tubeloop_t *pTubeloop = &Tubeloop[TubeId];
  
#ifdef MAIN_IF_REV2 
   stageCmd_t STAGE;
   stageCmd_t *TSeq = &STAGE;
 //  DEBUG_PRINTF("Tube[%d]Get next stage:%d",TubeId,pTubeloop->SeqIdx);

   if (getseq(TubeId,  TSeq) == TRUE)
  {
 // DEBUG_PRINTF("FOUND next stage %d",TSeq->stage);
#else
  if (pTubeloop->data != NULL)
  {
    stageCmd_t *TSeq = &(pTubeloop->data[pTubeloop->SeqIdx]);
#endif
    switch (TSeq->stage)
    {
#ifdef USE_IN_SEQUENCE_LOOPING
      case LoopStart:/*{3,0,LoopStart}*/
        pTubeloop->LoopStart = pTubeloop->SeqIdx+1;/*Loop from next entry*/
        pTubeloop->LoopIterations = TSeq->time;
        DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] *** LOOPSTART @StarID[%d] Iterations[%d] ***",
          TubeId,tube_states[pTubeloop->state],signals_txt[msg->ucMessageID],pTubeloop->SeqIdx,pTubeloop->LoopIterations);
        pTubeloop->SeqIdx++; /*Going to next step*/
        new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
        if(new_msg)
        {
          new_msg->ucMessageID=NEXT_TUBE_STAGE;
          p=(long *)new_msg->ucData;
          *p=TubeId;
          assert_param(pdPASS == xQueueSend(TubeSequencerQueueHandle, &new_msg, portMAX_DELAY));
        }
      break;
      case LoopEnd: 
        pTubeloop->LoopIterations--;
        if(pTubeloop->LoopIterations > 0) /*Finished looping ?*/
        {
          pTubeloop->SeqIdx=pTubeloop->LoopStart; /*Jump back to start of loop*/
          DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s]*** LOOP @Iterations left[%d]",
            TubeId,tube_states[pTubeloop->state],signals_txt[msg->ucMessageID],pTubeloop->LoopIterations);
        }else
        {
          pTubeloop->SeqIdx++;/*Finished looping go to next sequence*/
          DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s]***LOOP END***",
            TubeId,tube_states[pTubeloop->state],signals_txt[msg->ucMessageID]);
        }
        new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
        if(new_msg)
        {
          new_msg->ucMessageID=NEXT_TUBE_STAGE;
          p=(long *)new_msg->ucData;
          *p=TubeId;
          xQueueSend(TubeSequencerQueueHandle, &new_msg, portMAX_DELAY);
        }
      break;
#endif
      //All sequences has got an End stage as last entry
      case End:
        pTubeloop->state = TUBE_IDLE;
        DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s] END OF SEQUENCE FOR TUBE",
          TubeId,tube_states[pTubeloop->state],signals_txt[msg->ucMessageID]);
        /*Set idle mode for tube to stop heating*/
        data = SET_IDLE_MODE;
        WriteTubeHeaterReg(TubeId,TUBE_COMMAND_REG,&data,sizeof(data)/2); 
        pTubeloop->SeqIdx = 0;
        stop_lid_heating();
#ifdef USE_DEVELOPMENT_LOGGING
        /*Stop monitoring temperature on the tube*/
        new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
        if(new_msg)
        {
          new_msg->ucMessageID=END_LOG;
          p=(long *)new_msg->ucData;
          *p=TubeId;
          xQueueSend(LogQueueHandle, &new_msg, portMAX_DELAY);
        }
#endif
      break;
      //Normal entries are one of these stages - they are handled alike, except for pause.
      case Melting:
      case Annealing:
      case Extension:
      case Incubation:
     //   DEBUG_PRINTF("STart sequence");
        data = TSeq->temp;
        WriteTubeHeaterReg(TubeId,SETPOINT_REG,&data,sizeof(data)/2);
        data = TSeq->seq_num;
        WriteTubeHeaterReg(TubeId,STAGE_NUM_REG,&data,sizeof(data)/2);
        pTubeloop->state = TUBE_WAIT_TEMP;
        pTubeloop->curr.temp = TSeq->temp;
        pTubeloop->curr.time = TSeq->time;
        pTubeloop->curr.stage = TSeq->stage;
        pTubeloop->curr.seq_num = TSeq->seq_num;
        if(pTubeloop->SeqIdx == 0) 
        {
          /*Set heater to automatic mode if a new sequence is started*/
          data = SET_AUTOMATIC_MODE;
          WriteTubeHeaterReg(TubeId,TUBE_COMMAND_REG,&data,sizeof(data)/2);
#ifndef MAIN_IF_REV2 /*Cant see end of sequence in this mode*/
          while(pTubeloop->data[a].stage != End)
          { // find nof stages in sequence
            a++;
          }
          DEBUG_SEQ_PRINTF("\n\rTube[%d] Step %d of %d START Seq. New stage:%s, temp %d.%01dC",
            TubeId, pTubeloop->SeqIdx+1, a, stageToChar[TSeq->stage], TSeq->temp/10, TSeq->temp%10);
#endif          
        }else
        {
          DEBUG_SEQ_PRINTF("\n\rTube[%d] Step %d Time reached. New stage:%s, temp %d.%01dC",
            TubeId, TSeq->seq_num, stageToChar[TSeq->stage], TSeq->temp/10, TSeq->temp%10);
        }
#ifdef USE_DEVELOPMENT_LOGGING
        /*Start monitoring temperature on the tube*/
        new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
        if(new_msg)
        {
          new_msg->ucMessageID=START_LOG;
          p=(long *)new_msg->ucData;
          *p=TubeId;
          xQueueSend(LogQueueHandle, &new_msg, portMAX_DELAY);
        }
#endif
      break;
      default:
        DEBUG_PRINTF("ERROR STATE NOT HADLED Tube[%d]@%s-%d TubeSeq[%s] ",
          TubeId,tube_states[pTubeloop->state],pTubeloop->state,signals_txt[msg->ucMessageID]);
      break;
    }
  }
  else
  {
  DEBUG_SEQ_PRINTF("ERROR NO SEQ FOR Tube[%d]",TubeId);
   pTubeloop->curr.temp = 0;
   pTubeloop->curr.time = 0;
   pTubeloop->state = TUBE_OUT_OF_DATA;
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

  Heater_PinConfig();
  heaterIrqInit();
 //  stageCmd_t *TSeq; 

  InitTubeTimers();
  vTaskDelay(1000);/*Wait for heaters to boot*/
#ifdef SIMULATE_HEATER 
  gdi_send_msg_on_monitor("System is using SIMULATE_HEATER!!");
#endif
  while(1)
  {
  	/*wait for queue msg*/
    if( xQueueReceive( TubeSequencerQueueHandle, &msg, portMAX_DELAY) == pdPASS )
    {
      //UART_SendMsg(USART3, "OK\r" , 6);

     // DEBUG_PRINTF("TubeSeq[%s]",signals_txt[msg->ucMessageID]);
    //  USART_SendData(USART3,'T');

      switch(msg->ucMessageID)
      {
      case TUBE_TEST_SEQ:
          TubeId = *((long *)(msg->ucData));
          Tubeloop[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          modbus_data[0]=1;
          DEBUG_PRINTF("TUBE[%d] TUBE_TEST_SEQ-ST:[%s]#3",TubeId,tube_states[Tubeloop[TubeId].state]);
          ReadTubeHeaterReg(TubeId,EVENT_REG,1, TubeSequencerQueueHandle, FALSE); /*Read & Clear pending events on heater*/
#ifdef USE_DEVELOPMENT_LOGGING
#if 0
          new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
          if(new_msg)
          {
            new_msg->ucMessageID=START_LOG;
            p=(long *)new_msg->ucData;
            *p=TubeId;
            xQueueSend(LogQueueHandle, &new_msg, portMAX_DELAY);
          }
#endif
#endif
        break;
        case INIT_TUBE_SEQ:
        {
          TubeId = *((long *)(msg->ucData));
          Tubeloop[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          start_lid_heating();
        }
        break;
        case START_TUBE_SEQ:
           TubeId = *((long *)(msg->ucData));
         
          #if 1
          
         // if((Tubeloop[TubeId].state == TUBE_IDLE) && (Tubeloop[TubeId].SeqIdx==0))
        //  {
            DEBUG_PRINTF("Tube[%d]@%s START TUBE SEQUENCE ",TubeId,tube_states[Tubeloop[TubeId].state]);

            TubeStateHandler(TubeId,msg);
          
         // }else
         // {
        //    DEBUG_PRINTF("Tube[%d]@%s FAILED TO START TUBE SEQUENCE - SEQ Already running use AT@GDI:SEQ_CMD(%d,stop) to stop ",TubeId,tube_states[Tubeloop[TubeId].state],TubeId);
        //  }
          #endif
        break;
        case NEXT_TUBE_STAGE:
          TubeId = *((long *)(msg->ucData));
          Tubeloop[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          TubeStateHandler(TubeId,msg);
        break;
        case READ_MODBUS_REGS_RES:
          preg=(ReadModbusRegsRes *)msg->ucData;
          TubeId = preg->slave;
          Tubeloop[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          if(preg->resultOk == NO_ERROR)
          {
            if(preg->addr == EVENT_REG) {
            //  DEBUG_PRINTF("Tube[%d]Event[%x]",TubeId,*preg->data);
              HeaterEventHandler(preg, msg);
            } else {
              TubeMessageHandler(preg->slave, msg);
            }
          } else {
            DEBUG_PRINTF("####Tube[%d]ERROR MODBUS TELEGRAM READ FAILED!!! %d",TubeId,modbus_result);
          }
        break;
        case TIMER_EXPIRED:                       /*Waiting time for tube ended*/
          TubeId = *((long *)(msg->ucData));
          T = &Tubeloop[TubeId];
          T->ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] Stage:%s, Pause:%d", TubeId, tube_states[T->curr.stage], 
                           signals_txt[msg->ucMessageID], stageToChar[T->curr.stage],
                           T->pausePendingState);
#ifdef USE_PAUSE_FEATURE
          if( (Extension == T->curr.stage)
              && (TUBE_P_PAUSE_REQUESTED == T->pausePendingState) )
          {
            u16 data;
            data = T->pauseTemp;
            WriteTubeHeaterReg(TubeId, SETPOINT_REG, &data, sizeof(data)/2);
            Tubeloop[TubeId].state = TUBE_WAIT_P_TEMP;
            DEBUG_SEQ_PRINTF("Tube[%d] Step %d Time reached. waiting to reach pause temperature!\n\r", TubeId, Tubeloop[TubeId].SeqIdx+1);
            /* As we are pausing we do not goto next stage in sequence, this we do on "continue" command*/
          }
          else
#endif
          {
            Tubeloop[TubeId].SeqIdx++; /*Going to next stage in sequence*/
            new_msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
            new_msg->ucMessageID = NEXT_TUBE_STAGE;
            p=(long *)new_msg->ucData;
            *p=TubeId;
            xQueueSend(TubeSequencerQueueHandle, &new_msg, portMAX_DELAY);
          }
        break;
        case WRITE_MODBUS_REGS_RES:
          wres = ((WriteModbusRegsRes *)(msg->ucData));
          TubeId = wres->slave;
          Tubeloop[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
          modbus_addr = wres->addr;
          modbus_result = wres->resultOk;
          if(modbus_result != NO_ERROR)
          {
            DEBUG_PRINTF("####Tube[%d]ERROR MODBUS WRITE FAILED!!! %d",TubeId,modbus_result);
          }
          DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] Tube[%d]ADDR[%d]size[%d]STATUS[%s]", 
            TubeId, tube_states[Tubeloop[TubeId].state], signals_txt[msg->ucMessageID], wres->slave, wres->addr, 
            wres->datasize, (wres->resultOk==NO_ERROR)?"PASS":"FAIL");

           if(Tubeloop[TubeId].state == TUBE_INIT)/*No errors on current tube, set it to IDLE so its ready for use*/
            {
              Tubeloop[TubeId].state = TUBE_IDLE;
              bool Initialized = TRUE;
              i=1;
              while(i<nTubes)
              {
                if(Tubeloop[i].state != TUBE_IDLE)
                {
                  Initialized = FALSE;
                  break;
                }
                i++;
              }
              if (Initialized == TRUE)
              {
                gdi_send_msg_response("READY FOR OPERATION");
              }
            }        
        break;
        default:
          DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s]State[%s] ***UNHANDLED STATE***", 
            0xFF, tube_states[Tubeloop[0].state], signals_txt[msg->ucMessageID]);
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

