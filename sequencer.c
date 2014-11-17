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
#include "logtask.h"
#include "sequencer.h"
#include "ads1148.h"
#include "../heater-sw/heater_reg.h"
#include "gdi.h"
#include "serial.h"
#include "util.h"

//#define SIMULATE_HEATER /*Disable communication to M0 CPU's return temperature reached when temp is requested*/
//#define USE_DEVELOPMENT_LOGGING
#define USE_SYNCHRONOUS_PROTOCOL /* All tubes are synced at temp hold timeout */

extern xQueueHandle CoolAndLidQueueHandle;

#define USE_PAUSE_FEATURE
#define DEBUG_BUFFER_SIZE 600

#define DEBUG /*General debug shows state changes of tubes (new temp, new time etc.)*/
#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...)      snprintf(buf, DEBUG_BUFFER_SIZE, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_PRINTF(fmt, args...)                          /* Don't do anything in release builds */
#endif

//#define DEBUG_SEQ /*Debug of sequencer, to follow state of sequencer*/
#ifdef DEBUG_SEQ
#define DEBUG_SEQ_PRINTF(fmt, args...)      snprintf(buf, DEBUG_BUFFER_SIZE, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_SEQ_PRINTF(fmt, args...)                      /* Don't do anything in release builds */
#endif

//#define DEBUG_IF /*Debug of external interfaces modbus, IRQ and serial */
#ifdef DEBUG_IF
#define DEBUG_IF_PRINTF(fmt, args...)      snprintf(buf, DEBUG_BUFFER_SIZE, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_IF_PRINTF(fmt, args...)                       /* Don't do anything in release builds */
#endif

//#define DEBUG_QUEUE /*Debug of stage queue */
#ifdef DEBUG_QUEUE
#define DEBUG_QUEUE_PRINTF(fmt, args...)      snprintf(buf, DEBUG_BUFFER_SIZE, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_QUEUE_PRINTF(fmt, args...)                       /* Don't do anything in release builds */
#endif

/*STATUS_REG Bit definitions */
#define SEQUENCE_EVENT    ((uint16_t)0x01)                  /* New temperature reached on tube              */
#define INIT_HW_ERROR     ((uint16_t)0x02)                  /* HW failure detected during start up on tube  */
#define NOT_PRESENT       ((uint16_t)0x04)                  /* Tube not present                             */
#define LOGGING_READY     ((uint16_t)0x08)                  /* Logging data buffer ready to be read         */
#define EVENT_DATA_SIZE 5

#define NTUBES     16                                       /*Number of tubes to use, index 0 is dummy*/
#define NUM_TIMERS 16                                       /*There should be 1 for each tube*/

#define STAGES_QUEUE_SIZE (4)                               /* 4 iterations over 3 stages + 1 end indicator stage */

/* An array to hold handles to the created timers. */
xTimerHandle xTimers[ NUM_TIMERS ];

/* An array to hold the tick count of times each timer started - used for progress. */
long lStartTickCounters[ NUM_TIMERS ] = { 0 };

char buf[DEBUG_BUFFER_SIZE];                                /* buffer for debug printf*/

/* Private variables ---------------------------------------------------------*/
const char *  signals_txt[] =
{
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
  "SET_COOL_AND_LID",
  "SET_COOL_AND_LID_RES",
  "SET_FAN_SPEED",
  "SET_FAN_RES",
  "SET_COOL_TEMP",
  "SET_COOL_TEMP_RES",
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
};

typedef enum
{
  TUBE_INIT,                                                /*Ready                                    */
  TUBE_IDLE,
  TUBE_WAIT_TEMP,                                           /*Wait until desired temperature is reached*/
  TUBE_WAIT_TIME,                                           /*Wait the specified time in the sequence  */
  TUBE_WAIT_P_TEMP,                                         /*Wait until pause temperature is reached  */
  TUBE_PAUSED,                                              /*Wait until continiue is requested        */
  TUBE_OUT_OF_DATA,                                         /*No more data in que and no end tag is found*/
  TUBE_NOT_INITIALIZED
}TubeStates;

const char *  tube_states[] =
{
  "TUBE_INIT",
  "TUBE_IDLE",
  "TUBE_WAIT_TEMP",                                         /*Wait until desired temperature is reached*/
  "TUBE_WAIT_TIME",                                         /*Wait the time specified in the sequence  */
  "TUBE_WAIT_P_TEMP",                                       /*Wait until pause temperature is reached  */
  "TUBE_PAUSED",
  "TUBE_OUT_OF_DATA",                                       /*No more data in que and no end tag is found*/
  "TUBE_NOT_INITIALIZED"
};

/* ---------------------------------------------------------------------------*/
/* Pause is requested by setting pausePendingState = TUBE_P_PAUSE_REQUESTED   */
/* pausePendingState remains TUBE_P_PAUSE_REQUESTED untill tube state is set  */
/* to TUBE_PAUSED (i.e while tube state is TUBE_WAIT_P_TEMP), or a continue   */
/* cmd is received.                                                           */
typedef enum TUBE_PAUSE_FLAGS
{
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

typedef struct
{
  TubeStates state;
  TubePauseStates_t pausePendingState;
  portCHAR ucMessageID;                                     /* last message processed in state machine  */
  uint16_t pauseTemp;                                       /* Temperature to maintain during pause     */
  uint16_t event_reg;                                       /* last event/status register read from tube*/
  int tail;                                                 /* Read Pointer                             */
  int head;                                                 /* Write Pointer                            */
  stageCmd_t seq[STAGES_QUEUE_SIZE];                        /* The sequence                             */
  stageCmd_t curr;
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
  Heater1,
  Heater2,
  Heater3,
  Heater4,
  nExtiGpio
}ExtiGpioTypeDef;

const char *  stageToChar[] = {"Melting","Anealing","Extention","Incubation","Pause","End"};

#ifdef DEBUG_IF
/* For debug to */
const char *  heater[] =
{
  /*ADS_DRDY,*/
  "M0.1-Tube[1-4]",
  "M0.2-Tube[5-8]",
  "M0.3-Tube[9-12]",
  "M0.4-Tube[13-16]",
  "nExtiGpio"
};
#endif

ExtiGpioTypeDef tube2heater[]=
{
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

heater_tubes_t heater2tube[]= {
  { 1,2,3,4 },                                              /*Heater1*/
  { 5,6,7,8 },                                              /*Heater2*/
  { 9,10,11,12 },                                           /*Heater3*/
  { 13,14,15,16 },                                          /*Heater4*/
};

/*const*/ gpio_extint_t gpio_EXTI_CNF[nExtiGpio+1]={
  { GPIO_PinSource3 ,GPIO_PortSourceGPIOC,EXTI_Line3, EXTI3_IRQn },     /*Heater1*/
  { GPIO_PinSource4 ,GPIO_PortSourceGPIOC,EXTI_Line4, EXTI4_IRQn },     /*Heater2*/
  { GPIO_PinSource11,GPIO_PortSourceGPIOC,EXTI_Line11,EXTI15_10_IRQn }, /*Heater3*/
  { GPIO_PinSource12,GPIO_PortSourceGPIOC,EXTI_Line12,EXTI15_10_IRQn }, /*Heater4*/
  { 0xFF,0,EXTI0_IRQn } /*Termination*/
};

/*Tube state register*/
Tubeloop_t Tubeloop[NTUBES] = {
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
    
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
    
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
    
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} },
  { TUBE_NOT_INITIALIZED,TUBE_P_NORM,0,0,0,0,0, {}, {0,0,End,0} }
};

extern xQueueHandle ModbusQueueHandle;
extern xQueueHandle TubeSequencerQueueHandle;
extern xQueueHandle LogQueueHandle;

/* Private function prototypes -----------------------------------------------*/
void vTimerCallback( xTimerHandle pxTimer );
void ExtIrqDisable(ExtiGpioTypeDef heater);
void ExtIrqEnable(ExtiGpioTypeDef heater);
bool getseq(u8 tubeId,stageCmd_t * data);
static int tubequefree(long tubeid);
void tubeinitQueue();
void stop_all_tube_LED();

/* Private functions ---------------------------------------------------------*/
void InitTubeTimers()
{
  long x;

  /* Create then start some timers.  Starting the timers before the RTOS scheduler
  has been started means the timers will start running immediately that
  the RTOS scheduler starts. */
  for(x = 0; x < NUM_TIMERS; x++)
  {
    xTimers[ x ] = xTimerCreate
      (                                                     /* Just a text name, not used by the RTOS kernel. */
      (char *) "TubeTimer",
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

/* ---------------------------------------------------------------------------*/
void StartTubeTimer( long TubeNum, long time  )
{
  long tube_idx = TubeNum -1;

  /*time in mSec*/
  configASSERT(NULL != xTimers[ tube_idx ]);    /* The timer was not created. */
  
  lStartTickCounters[ tube_idx ]= xTaskGetTickCount();
  DEBUG_PRINTF("Tube[%ld]StartTubeTimer Time[%ld]Tick[%ld]",TubeNum,time,lStartTickCounters[ tube_idx ]);

  /* Start the timer.  No block time is specified, and even if one was
  it would be ignored because the RTOS scheduler has not yet been
  started. */
  if(time > 1)
  {
    if(xTimerChangePeriod( xTimers[ tube_idx ],100 * time,100)!= pdPASS )
    {
      DEBUG_PRINTF("###ERROR TIMER SET Tube[%ld]-Time[%ld] ",TubeNum,time);
      configASSERT(pdFALSE); // This is a fatal error 
    }
    if( xTimerStart( xTimers[ tube_idx ], 0 ) != pdPASS )
    {
      /* The timer could not be set into the Active state. */
      DEBUG_PRINTF("###ERROR TIMER START Tube[%ld]-Time[%ld] ",TubeNum,time);
      configASSERT(pdFALSE); // This is a fatal error 
    }
  }
  else
  {
    DEBUG_PRINTF("###ERROR TIMER START Tube[%ld]-Time[%ld] CANT BE '0'!! ",TubeNum,time);
  }
}

/* ---------------------------------------------------------------------------*/
long GetTubeTimeLeft(long TubeNum)
{
  long tube_idx = TubeNum -1;
  long time_consumed;
  long time_left;
  Tubeloop_t *T;
  T = &Tubeloop[tube_idx];
  time_consumed = xTaskGetTickCount() - lStartTickCounters[ tube_idx ];
  DEBUG_SEQ_PRINTF("TUBE[%ld] GetTubeTimeLeft VAL[%ld] EST LEFT[%ld] PROGRESS[%ld] TICK[%ld]-[%ld]",
               TubeNum, T->curr.time, time_consumed/100,((time_consumed)/T->curr.time), xTaskGetTickCount(),
               lStartTickCounters[ tube_idx ] );
      /*Ended state - Tube Paused - Waiting for heat up no timer running*/
  if( (T->curr.time == 0)||(T->curr.stage==Pause)||(0==lStartTickCounters[ tube_idx ]))
  {
    time_left = 0;
  }else
  {
    time_left = ((time_consumed)/T->curr.time);
  }
  return time_left;
}

/* ---------------------------------------------------------------------------*/
void StopTubeTimer(long TubeNum)
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

/* ---------------------------------------------------------------------------*/
void heaterIrqInit(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

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
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;   /*dont care*/
    NVIC_InitStructure.NVIC_IRQChannel = p->IRQ_CH;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    p++;
  }
}

/* ---------------------------------------------------------------------------*/
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

/* ---------------------------------------------------------------------------*/
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

/* ---------------------------------------------------------------------------*/
/*  data size is number of registers (2 bytes) */
/* ---------------------------------------------------------------------------*/
void WriteTubeHeaterReg(u8 tube, u16 reg, u16 *data, u16 datasize)
{
  xMessage *msg;

  #if defined (SIMULATE_HEATER)                             /*Simulate response from Heater CPU*/
  ReadModbusRegsRes *p;
  u16 *data_p ;
  u16 event = (tube%4)==1?SEQUENCE_EVENT_TUBE1:0|(tube%4)==2?SEQUENCE_EVENT_TUBE2:0|(tube%4)==3?SEQUENCE_EVENT_TUBE3:0|(tube%4)==0?SEQUENCE_EVENT_TUBE4:0;

  if (reg == SETPOINT_REG)                                  /*Make virtual response if setpoint is set*/
  {
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsRes)+5*sizeof(u16));
    msg->ucMessageID=READ_MODBUS_REGS_RES;
    p=(ReadModbusRegsRes *)msg->ucData;
    data_p = (u16*)p->data;
    p->slave=(tube/4)*3+1;
    p->addr=EVENT_REG;
    *(data_p) = event<<8;
    *(data_p+1) = ((*data&0xFF)<<8)|(*data>>8);             /*Tube 1 on heater*/
    *(data_p+2) = ((*data&0xFF)<<8)|(*data>>8);             /*Tube 2 on heater*/
    *(data_p+3) = ((*data&0xFF)<<8)|(*data>>8);             /*Tube 3 on heater*/
    *(data_p+4) = ((*data&0xFF)<<8)|(*data>>8);             /*Tube 4 on heater*/
    //memcpy((p->data+1), data, datasize*sizeof(u16));

    //memcpy(p->data, data, datasize);
    p->datasize=5;
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
    configASSERT(pdFALSE); // This is a fatal error
  }
  else
  {
    int i;
    for(i = 0; i < datasize; i++)
    {
      *(data+i)=((*(data+i)&0xFF)<<8)|(*(data+i)>>8);
    }
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

/* ---------------------------------------------------------------------------*/
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
  // Tubeloop[tube-1].curr.stage = End;
  Tubeloop[tube-1].state = TUBE_IDLE;                         /*Tube ready for operation*/
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
    configASSERT(pdFALSE); // This is a fatal error
  }
  else
  {
    msg->ucMessageID=READ_MODBUS_REGS;
    p=(ReadModbusRegsReq *)msg->ucData;
    p->slave=tube;
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

/* ---------------------------------------------------------------------------*/
/* Configure the GPIO Pins to PWM using TIM1 */
void Heater_PinConfig(void)
{
  /*Setup for Heater Interrupt*/
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOC Clocks enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  /* GPIOC Configuration: Channel 1, 2 and 3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* ---------------------------------------------------------------------------*/
void ReadTubeHeaterRegFromISR( void *pvParameter1, uint32_t ulParameter2 )
{
  u8 tube = (u8)ulParameter2;
  xMessage *msg;
  ReadModbusRegsReq *p;
  portBASE_TYPE taskWoken = pdTRUE;
  msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq));
  if(NULL == msg)
  {
    configASSERT(pdFALSE); // This is a fatal error
  }
  else
  {
    msg->ucMessageID=READ_MODBUS_REGS;
    p=(ReadModbusRegsReq *)msg->ucData;
    p->slave=tube;
    p->addr=EVENT_REG;
    p->datasize=5;
    p->reply=TubeSequencerQueueHandle;
    configASSERT(xQueueSendFromISR(ModbusQueueHandle,&msg,&taskWoken) == pdPASS);
  }
  return;
}

/* ---------------------------------------------------------------------------*/
void EXTI_Handler(void)
{
//  vTraceStoreISRBegin(3);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  ExtiGpioTypeDef ExtiGpio = Heater1;
  while(ExtiGpio < nExtiGpio)                               /*Alwlays starts at Heater1 */
  {
    if (SET == EXTI_GetFlagStatus(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE))
    {
      DEBUG_IF_PRINTF("INTERRUPT-READ STATUS ON HEATER[%s]",heater[ExtiGpio]);
      xTimerPendFunctionCallFromISR( ReadTubeHeaterRegFromISR,
                               NULL,
                               ( uint32_t ) heater2tube[ExtiGpio].tube_1,
                               &xHigherPriorityTaskWoken );
      EXTI_ClearITPendingBit(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE);
    }
    ExtiGpio++;
  }
//  vTraceStoreISREnd();
  //portEND_SWITCHING_ISR( pdTRUE);
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* ---------------------------------------------------------------------------*/
void start_lid_heating()
{
  xMessage *msg;

  msg = pvPortMalloc(sizeof(xMessage));
  if(msg)
  {
    msg->ucMessageID = START_LID_HEATING;
    xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
  }
}

/* ---------------------------------------------------------------------------*/
void stop_lid_heating()
{
  xMessage *msg;

  msg=pvPortMalloc(sizeof(xMessage));
  if(msg)
  {
    msg->ucMessageID=STOP_LID_HEATING;
    xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
  }
}

/* ---------------------------------------------------------------------------*/
void stop_all_tube_LED()
{
  long TubeId;
  for(TubeId = 1; TubeId < 17; TubeId++)
  {
    send_led_cmd(SET_LED_OFF, TubeId);
  }
}

/* ---------------------------------------------------------------------------*/
bool start_tube_seq( long TubeId)
{
  bool result = TRUE;

  if( (Tubeloop[TubeId-1].state == TUBE_PAUSED) || Tubeloop[TubeId-1].state == TUBE_IDLE )
  {
    long *p;
    xMessage *msg;
    msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));

    //  DEBUG_PRINTF("start_tube_seq-data valid");
    if(msg)
    {
      msg->ucMessageID = START_TUBE_SEQ;
      p = (long *)msg->ucData;
      *p = TubeId;
      xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
      //  DEBUG_PRINTF("start_tube_seq-send msg");
    }
    /* waiting for pause but did regret it */
  }
  else if(Tubeloop[TubeId-1].pausePendingState == TUBE_P_PAUSE_REQUESTED)
  {
    Tubeloop[TubeId-1].pausePendingState = TUBE_P_NORM;
    DEBUG_PRINTF("Tube[%ld] pause aborted REQ[%d]",TubeId,Tubeloop[TubeId-1].pausePendingState);
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
  #ifdef USE_DEVELOPMENT_LOGGING
  long *p;
  xMessage *new_msg;
  #endif
  bool result = TRUE;
  //stageCmd_t STAGE;
  //stageCmd_t *TSeq = &STAGE;

  if((TUBE_NOT_INITIALIZED == Tubeloop[TubeId-1].state) || (TUBE_INIT == Tubeloop[TubeId-1].state) || (TUBE_IDLE == Tubeloop[TubeId-1].state))
  {
    result = FALSE;
  }
  else
  {
    if(Tubeloop[TubeId-1].state == TUBE_WAIT_TIME)
    {
      StopTubeTimer(TubeId);
    }
    /*Set idle mode for tube to stop heating*/
    data = SET_IDLE_MODE;
    WriteTubeHeaterReg(TubeId, TUBE_COMMAND_REG, &data, sizeof(data)/2);
    DEBUG_SEQ_PRINTF("Tube[%ld]@%s END OF SEQUENCE FOR TUBE",
                      TubeId, tube_states[Tubeloop[TubeId].state]);
    Tubeloop[TubeId-1].state = TUBE_IDLE;
    Tubeloop[TubeId-1].pausePendingState = TUBE_P_NORM;
    { // If no tubes are running switch off light and top heater
      int sequences_done = TRUE;
      int i;
      for(i = 0; i < 16; i++)
      {
        //TUBE_INIT, TUBE_IDLE, TUBE_WAIT_TEMP, TUBE_WAIT_TIME, TUBE_WAIT_P_TEMP, TUBE_PAUSED, TUBE_OUT_OF_DATA, TUBE_NOT_INITIALIZED
        if(! ( (TUBE_INIT == Tubeloop[i].state) || (TUBE_IDLE  == Tubeloop[i].state) || (TUBE_NOT_INITIALIZED == Tubeloop[i].state) ))
        {
          sequences_done = FALSE;             
          DEBUG_PRINTF("sequence NOT done Tube:%d", i);
        }
      }
      if(sequences_done)
      {
        DEBUG_PRINTF("sequences done");
        //#### We cannot switch on yet!!  stop_lid_heating();
        stop_all_tube_LED();
      }
    }

    //while( (getseq(TubeId, TSeq) == TRUE));                 /*empty buffer*/
    tubeinitQueue();
    emptyLog(TubeId);
    Tubeloop[TubeId-1].curr.seq_num = 0;
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
bool pause_tube_state(long TubeId)
{
  Tubeloop[TubeId-1].pausePendingState = TUBE_P_PAUSE_REQUESTED;
  DEBUG_PRINTF("Tube[%ld] - Pause REQ[%d]", TubeId, Tubeloop[TubeId-1].pausePendingState);
  return TRUE;
}
#endif

/* ---------------------------------------------------------------------------*/
char * get_tube_state(long TubeId, char *poutText)
{
  char state[20]="idle";
  char progress[5]="50";
  char stage_nr[5]="3";

  *poutText = 0;

  if(Tubeloop[TubeId-1].state == TUBE_OUT_OF_DATA)
  {
    strcpy(state,"waiting for data");
    strcpy(progress, "0");
  }
  else if((Tubeloop[TubeId-1].state == TUBE_WAIT_TEMP)||(Tubeloop[TubeId-1].state == TUBE_WAIT_TIME)||(Tubeloop[TubeId-1].state == TUBE_WAIT_P_TEMP))
  {
    // Tube is considered running untill the pause temperature is actually reached.
    Itoa(GetTubeTimeLeft(TubeId), progress);
    if(tubequefree(TubeId) > 0)
    {
      strcpy(state, "running waiting for data");
    }
    else
    {
      strcpy(state, "running");
    }
  }
  else if(Tubeloop[TubeId-1].state == TUBE_PAUSED)
  {
    strcpy(state, "paused");
  }
  else if(Tubeloop[TubeId-1].state == TUBE_IDLE)
  {
    strcpy(state, "idle");
  }
  Itoa(Tubeloop[TubeId-1].curr.seq_num, stage_nr);
  strcpy(poutText, "OK<state=\"");
  strcat(poutText, state);
  if(Tubeloop[TubeId-1].state != TUBE_IDLE)
  {
    strcat(poutText, "\";progress=");
    strcat(poutText, progress);
    strcat(poutText, ";stage_number=");
    strcat(poutText, stage_nr);
    getLog(poutText, TubeId);
  }
  strcat(poutText, ">");
  return poutText;
}

/* ---------------------------------------------------------------------------*/
void tubeinitQueue()
{
  int i;
  //Tubeloop_t *pTubeloop = &Tubeloop[TubeId-1];
  for(i=0; NTUBES > i; i++)
  {
    Tubeloop_t *pTubeloop = &Tubeloop[i-1];
    pTubeloop->tail = -1;
    pTubeloop->head = -1;
  }
}

/* ---------------------------------------------------------------------------*/
// Enqueue element. Return pointer to element so it can be written to.
stageCmd_t * tubeenqueue(Tubeloop_t * pQueue)
{
  if((pQueue->tail - STAGES_QUEUE_SIZE) == pQueue->head)    // Return null if queue is full
  {
    return NULL;
  }
  pQueue->tail++;
  DEBUG_QUEUE_PRINTF("tube-enqueue: t:%d h:%d FREE:%d", pQueue->tail, pQueue->head, (pQueue->tail - pQueue->head));
  return &pQueue->seq[pQueue->tail % STAGES_QUEUE_SIZE];
}

/* ---------------------------------------------------------------------------*/
// Dequeue element. Return pointer to element so it can be read from.
stageCmd_t * tubedequeue(Tubeloop_t * pQueue)
{
  if(pQueue->tail == pQueue->head)                          // Return null if queue is empty
  {
    return NULL;
  }
  else
  {
    pQueue->head++;
    DEBUG_QUEUE_PRINTF("tube-dequeue: t:%d h:%d FREE:%d : %ld", pQueue->tail ,pQueue->head, (pQueue->tail - pQueue->head), (long int)&pQueue->head);
    return &pQueue->seq[pQueue->head % STAGES_QUEUE_SIZE];
  }
}

/* ---------------------------------------------------------------------------*/
int tubequefree(long tubeid)
{
  Tubeloop_t * pQueue = &Tubeloop[tubeid-1];
  DEBUG_QUEUE_PRINTF("tubequefree: t:%d h:%d", pQueue->tail ,pQueue->head);
  return (int)STAGES_QUEUE_SIZE-(pQueue->tail - pQueue->head);
}

/* ---------------------------------------------------------------------------*/
bool tubedataQueueAdd(u8 tubeId,u16 seq_num, char state, stageCmd_t *data)
{
  bool result = FALSE;
  stageCmd_t * poutData;
  Tubeloop_t *pTubeloop = &Tubeloop[tubeId-1];
  xMessage *msg;
  long *pucData;
  // DEBUG_PRINTF("tubedataQueueAdd: %ld",tubeId);

  taskENTER_CRITICAL();                                     //push irq state
  // DEBUG_PRINTF("ENQUE:Tube:%ld:TEMP %d.%02dC @ TIME %d.%02dsecs STATE:%d ",tubeId,data->temp/10,data->temp%10,data->time/10,data->time%10,data->stage);
  if(NULL != (poutData = tubeenqueue(&Tubeloop[tubeId-1]))) //tubeId=[1..16],idx=[0..15]
  {
    DEBUG_QUEUE_PRINTF("T%d:@QUE ADD:TEMP %d.%02dC @ TIME %ld.%02ldsecs SEQ_ID:%d STATE:%c FREE:%d CURR T ST:%d ", tubeId, data->temp/10, data->temp%10, data->time/10, data->time%10, seq_num,state, tubequefree(tubeId), pTubeloop->state);
    poutData->temp  = data->temp;
    poutData->time  = data->time;
    poutData->seq_num = seq_num;
    if('m' ==  (char)state)      { poutData->stage = Melting;   }
    else if('a' ==  (char)state) { poutData->stage = Annealing; }
    else if('e' ==  (char)state) { poutData->stage = Extension; }
    else if('i' ==  (char)state) { poutData->stage = Incubation;}
    if((0 ==  data->time)&(0 ==  data->temp)) { poutData->stage = End;}
    // DEBUG_PRINTF("%s-%d-%c",stageToChar[poutData->stage], poutData->stage, state);
    result = TRUE;
  }else
  {
    result = FALSE;
  }

  taskEXIT_CRITICAL();
  if(pTubeloop->state == TUBE_OUT_OF_DATA)                  /*Auto restart running sequence after added more data*/
  {
    msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
    if(msg)
    {
      msg->ucMessageID = NEXT_TUBE_STAGE;                   //Jump to next stage on queue
      pucData = (long *)msg->ucData;
      *pucData = tubeId;
      xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
    }
    else
    {
      configASSERT(pdFALSE); //If this happens the tube sequence is stuck
    }
  }
  return result;
}

/* ---------------------------------------------------------------------------*/
/* dequene next stage                                                         */
/* ---------------------------------------------------------------------------*/
bool getseq(u8 tubeId, stageCmd_t *data)
{
  bool res = FALSE;
  stageCmd_t *quedata;
  DEBUG_PRINTF("DEQUE %d", tubeId);
  taskENTER_CRITICAL();  //push irq state #### Kan critical section laves mindre!
  // Send all available log elements for each tube
  if(NULL != (quedata = tubedequeue(&Tubeloop[tubeId-1])) ) //idx=[0..15]
  {
    memcpy(data,quedata,sizeof(stageCmd_t));
    res = TRUE;
  }
  else
  {
    res = FALSE;
  }
  taskEXIT_CRITICAL();
  return res;
}

/* ---------------------------------------------------------------------------*/
/* Handle HW Status                                                           */
/* ---------------------------------------------------------------------------*/
void HW_EventHandler(ReadModbusRegsRes *preg, xMessage *msg){
  // Change tube state to some out of order state
}

/* ---------------------------------------------------------------------------*/
/* Handle event for one tube */
/* ---------------------------------------------------------------------------*/
void TubeEventHandler (long TubeId, int event, xMessage *msg)
{
  ReadModbusRegsRes *preg;
  u16 modbus_id;
  u16 modbus_data[EVENT_DATA_SIZE];
  long i=0;

  preg = (ReadModbusRegsRes *)msg->ucData;
  Tubeloop_t *pTubeloop = &Tubeloop[TubeId-1];
  for(i = 0; i < (preg->datasize); i++)
  {
    modbus_data[i] =(((u16)(preg->data[i*2])<<8) | (preg->data[(i*2)+1]));
  }
  #ifdef DEBUG
  // If not logging event output debug data
  if(((event & LOGGING_READY)==FALSE))
  {
    DEBUG_PRINTF("Tube[%ld][%x]-EVENT %s - ST:%s",TubeId,modbus_data[0] ,(((event & SEQUENCE_EVENT)==TRUE)?"SEQ_EVENT":" "),tube_states[pTubeloop->state]);
  }
  #endif

  /* Handle "Temperature reached" event */
  if( ((pTubeloop->state == TUBE_WAIT_TEMP) || (pTubeloop->state == TUBE_WAIT_P_TEMP)) &&
    (event & SEQUENCE_EVENT) )
  {
    modbus_id = TubeId%4;
    if(modbus_id == 0) { modbus_id = 4; }
    #ifdef USE_PAUSE_FEATURE
    if(TUBE_WAIT_P_TEMP == Tubeloop[TubeId-1].state)
    {
      Tubeloop[TubeId-1].state = TUBE_PAUSED;
      pTubeloop->pausePendingState = TUBE_P_NORM;           //Pause is not pending anymore
      //do we check if all tubes are in pause and notify Linuxbox
      DEBUG_SEQ_PRINTF("Tube[%ld] Pause temp reached:%d.%01dC - paused!", TubeId, modbus_data[modbus_id]/10, modbus_data[modbus_id]%10);
    }
    else
    #endif
    {
      StartTubeTimer(TubeId,pTubeloop->curr.time);
      pTubeloop->state = TUBE_WAIT_TIME;
    }
  }
  /* Handle "log data ready" event */
  if((LOGGING_READY) & event)
  { //Request a read of the logged data from tube. The result is sent to logging task
    ReadTubeHeaterReg(TubeId, DATA_LOG, DATA_LOG_SIZE+1, LogQueueHandle, FALSE);
  }
  /*  */
  if(Tubeloop[TubeId-1].state == TUBE_NOT_INITIALIZED)        /*No errors on current tube, set it to IDLE so its ready for use*/
  {
    Tubeloop[TubeId-1].state = TUBE_INIT;    
    tubeinitQueue();
    modbus_data[0] = 1;
    #if defined(SIMULATE_HEATER)
    ReadTubeHeaterReg(TubeId, EVENT_REG, 1, TubeSequencerQueueHandle, FALSE);
    #else
    ExtIrqEnable(tube2heater[TubeId]);
    DEBUG_PRINTF("ENABLE IRQ ON HEATER[%d]",tube2heater[TubeId]);
    WriteTubeHeaterReg(TubeId, EVENT_REG, &modbus_data[0], 1); /* TEST to force the heater to create an event*/
    #endif
  }
}

/* ---------------------------------------------------------------------------*/
/* Read event register and sort out events per tube and handle the events     */
/* for each tube seperately                                                   */
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
  Tubeloop[TubeId-1].event_reg = modbus_data[0];
  #if 1                                                     // event debug out
  DEBUG_SEQ_PRINTF("T%ld:EVENT[%04x]:%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s-%s",
    TubeId,modbus_data[0],
    ((modbus_data[0] & SEQUENCE_EVENT_TUBE1)? "S_EV_T1 "  : " "),
    ((modbus_data[0] & SEQUENCE_EVENT_TUBE2)? "S_EV_T2 "  : " "),
    ((modbus_data[0] & SEQUENCE_EVENT_TUBE3)? "S_EV_T3 "  : " "),
    ((modbus_data[0] & SEQUENCE_EVENT_TUBE4)? "S_EV_T4 "  : " "),
    ((modbus_data[0] & M0_BOOT)             ? "M0_Boot "  : " "),
    ((modbus_data[0] & M0_PING)             ? "M0_Ping "  : " "),
    ((modbus_data[0] & HW_FAULT)            ? "HW_Fault  ": " "),
    ((modbus_data[0] & ((uint16_t)0x0080))  ? "Bit_7 "    : " "),
    ((modbus_data[0] & ((uint16_t)0x0100))  ? "Bit_8 "    : " "),
    ((modbus_data[0] & ((uint16_t)0x0200))  ? "Bit_9 "    : " "),
    ((modbus_data[0] & ((uint16_t)0x0400))  ? "Bit_10 "   : " "),
    ((modbus_data[0] & ((uint16_t)0x0800))  ? "Bit_11 "   : " "),
    ((modbus_data[0] & TUBE1_LOGGING_READY) ? "LogT1 "    : " "),
    ((modbus_data[0] & TUBE2_LOGGING_READY) ? "LogT2 "    : " "),
    ((modbus_data[0] & TUBE3_LOGGING_READY) ? "LogT3 "    : " "),
    ((modbus_data[0] & TUBE4_LOGGING_READY) ? "LogT4 "    : " "));
  #endif
  event = 0;
  event |= (modbus_data[0] & SEQUENCE_EVENT_TUBE1) ? SEQUENCE_EVENT : 0;
  event |= (modbus_data[0] & TUBE1_LOGGING_READY)  ? LOGGING_READY  : 0;
  if(event) TubeEventHandler (TubeId + 0, event, msg);
  event = 0;
  event |= (modbus_data[0] & SEQUENCE_EVENT_TUBE2) ? SEQUENCE_EVENT : 0;
  event |= (modbus_data[0] & TUBE2_LOGGING_READY)  ? LOGGING_READY  : 0;
  if(event) TubeEventHandler (TubeId + 1, event, msg);
  event = 0;
  event |= (modbus_data[0] & SEQUENCE_EVENT_TUBE3) ? SEQUENCE_EVENT : 0;
  event |= (modbus_data[0] & TUBE3_LOGGING_READY)  ? LOGGING_READY  : 0;
  if(event) TubeEventHandler (TubeId + 2, event, msg);
  event = 0;
  event |= (modbus_data[0] & SEQUENCE_EVENT_TUBE4) ? SEQUENCE_EVENT : 0;
  event |= (modbus_data[0] & TUBE4_LOGGING_READY)  ? LOGGING_READY  : 0;
  if(event) TubeEventHandler (TubeId + 3, event, msg);

  if(Tubeloop[TubeId-1].state == TUBE_NOT_INITIALIZED)
  {  // This is only to get tube state from TUBE_NOT_INITIALIZED to TUBE_INIT
    TubeEventHandler (TubeId, event, msg);
  }

  if(modbus_data[0] & HW_FAULT)
  { // Read HW_STATUS_REG
    ReadTubeHeaterReg(TubeId, HW_STATUS_REG, 1, TubeSequencerQueueHandle, FALSE);
  }
  
  if(modbus_data[0] & M0_BOOT)
  { // M0 booted 
    
  }

  if(modbus_data[0] & M0_PING)
  { //Answer to a ping
    
  }
}

/* ---------------------------------------------------------------------------*/
/* Handle any register read other than event register */
/* ---------------------------------------------------------------------------*/
void TubeMessageHandler (long TubeId, xMessage *msg)
{
  #if 0
  ReadModbusRegsRes *preg;
  preg = ??
    if(preg->addr == SETPOINT_REG)
  {
    //DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] SetPoint[%d]", TubeId, tube_states[Tubeloop[TubeId-1].state],
    //              signals_txt[msg->ucMessageID], modbus_data[0]/10);
  }

  // DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s]ADDR[%d]size[%d]data[%x]STATUS[%s]",
  //   TubeId, tube_states[Tubeloop[TubeId-1].state], signals_txt[msg->ucMessageID],
  //   preg->addr, preg->datasize,/*(((u16)(preg->data[0])<<8)|(preg->data[1]))*/modbus_data[0],
  //   (preg->resultOk==NO_ERROR)?"PASS":"FAIL");
  #endif
}

/* ---------------------------------------------------------------------------*/
/* TubeStageHandler sets up the next stage in a sequence, temperature and time*/
/* TubeStageHandler is called on reception of START_TUBE_SEQ and              */
/* NEXT_TUBE_STAGE messages  */
void TubeStageHandler(long TubeId, xMessage *msg)
{
  #ifdef USE_DEVELOPMENT_LOGGING
  xMessage *new_msg;
  long *p;
  #endif
  u16 data;
  u16 data_element[2];
  Tubeloop_t *pTubeloop = &Tubeloop[TubeId-1];
  stageCmd_t *TSeq = &pTubeloop->curr;

  if(getseq(TubeId, TSeq) == TRUE) //Get next stage in sequence
  {
    // DEBUG_PRINTF("FOUND next stage %d",TSeq->stage);
    switch (TSeq->stage)
    {
      //All sequences has got an End stage as last entry
      case End:
        pTubeloop->state = TUBE_IDLE;
        DEBUG_SEQ_PRINTF("Tube[%ld]@%s TubeSeq[%s] END OF SEQUENCE FOR TUBE",
          TubeId,tube_states[pTubeloop->state],signals_txt[(unsigned char)msg->ucMessageID]);
        /*Set idle mode for tube to stop heating*/
        data = SET_IDLE_MODE;
        WriteTubeHeaterReg(TubeId, TUBE_COMMAND_REG, &data, sizeof(data)/2);
        { // If no tubes are running switch off light and top heater
          int sequences_done = TRUE;
          int i;
          for(i = 0; i < 16; i++)
          {
            //TUBE_INIT, TUBE_IDLE, TUBE_WAIT_TEMP, TUBE_WAIT_TIME, TUBE_WAIT_P_TEMP, TUBE_PAUSED, TUBE_OUT_OF_DATA, TUBE_NOT_INITIALIZED
            if(! ( (TUBE_INIT == Tubeloop[i].state) || (TUBE_IDLE  == Tubeloop[i].state) || (TUBE_NOT_INITIALIZED == Tubeloop[i].state) ))
            {
              sequences_done = FALSE;             
              DEBUG_PRINTF("sequence NOT done Tube:%d", i);
            }
          }
          if(sequences_done)
          {
            DEBUG_PRINTF("sequences done");
            //#### We cannot switch on yet!!  stop_lid_heating();
            stop_all_tube_LED();
          }
        }
      #ifdef USE_DEVELOPMENT_LOGGING
        /*Stop monitoring temperature on the tube*/
        new_msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
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
        // DEBUG_PRINTF("STart sequence");
        data_element[0] = TSeq->temp;
        data_element[1] = TSeq->seq_num;
        // Write both SETPOINT_REG and STAGE_NUM_REG at the same time to make sure the log stage and measurements are in sync.
        WriteTubeHeaterReg(TubeId, SETPOINT_REG, data_element, sizeof(data_element)/2);
        pTubeloop->state = TUBE_WAIT_TEMP;
        DEBUG_QUEUE_PRINTF("### head=%d @ %08lX", pTubeloop->head, (long int)&pTubeloop->head);
        if(pTubeloop->head == 0) // First element in sequence
        {
          DEBUG_QUEUE_PRINTF("### head==0 @ %08lX", (long int)&pTubeloop->head);
          /*Set heater to automatic mode if a new sequence is started*/
          data = SET_AUTOMATIC_MODE;
          WriteTubeHeaterReg(TubeId, TUBE_COMMAND_REG, &data, sizeof(data)/2);
        }
        else
        {
          DEBUG_SEQ_PRINTF("###\n\rTube[%ld] Step %d Time reached. New stage:%s, temp %d.%01dC",
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
        DEBUG_PRINTF("ERROR STATE NOT HADLED Tube[%ld]@%s-%d TubeSeq[%s] ",
          TubeId,tube_states[pTubeloop->state],pTubeloop->state,signals_txt[(unsigned char)msg->ucMessageID]);
        break;
    }
  }
  else
  {
    DEBUG_SEQ_PRINTF("ERROR NO SEQ FOR Tube[%ld]",TubeId);
    pTubeloop->curr.temp = 0;
    pTubeloop->curr.time = 0;
    pTubeloop->state = TUBE_OUT_OF_DATA;
  }
}

/* ---------------------------------------------------------------------------*/
void TubeSequencerTask( void * pvParameter)
{
  xMessage *msg,*new_msg;
  long TubeId;
  USART_ERROR modbus_result;
  long *p,i=0;
  ReadModbusRegsRes *preg;
  WriteModbusRegsRes *wres;
  Tubeloop_t *T;

  Heater_PinConfig();
  heaterIrqInit();

  InitTubeTimers();
  vTaskDelay(1000);                                       /*Wait for heaters to boot*/
  #ifdef SIMULATE_HEATER
  gdi_send_msg_on_monitor("System is using SIMULATE_HEATER!!");
  #endif
  while(1)
  {
    /*wait for queue msg*/
    if( xQueueReceive( TubeSequencerQueueHandle, &msg, portMAX_DELAY) == pdPASS )
    {
      switch(msg->ucMessageID)
      {
        case TUBE_TEST_SEQ:
          TubeId = *((long *)(msg->ucData));
          Tubeloop[TubeId-1].ucMessageID = msg->ucMessageID;

          DEBUG_PRINTF("TUBE[%ld] TUBE_TEST_SEQ-ST:[%s]#3", TubeId, tube_states[Tubeloop[TubeId-1].state]);
                                                          /*Read & Clear pending events on heater*/
          ReadTubeHeaterReg(TubeId, EVENT_REG, 1, TubeSequencerQueueHandle, FALSE);
          break;
        case START_TUBE_SEQ:
          TubeId = *((long *)(msg->ucData));
          DEBUG_PRINTF("Tube[%ld]@%s START TUBE SEQUENCE ", TubeId, tube_states[Tubeloop[TubeId-1].state]);
          TubeStageHandler(TubeId,msg);
          break;
        case NEXT_TUBE_STAGE:
          TubeId = *((long *)(msg->ucData));
          Tubeloop[TubeId-1].ucMessageID = msg->ucMessageID;
          TubeStageHandler(TubeId,msg);
          break;
        case READ_MODBUS_REGS_RES:
          preg = (ReadModbusRegsRes *)msg->ucData;
          TubeId = preg->slave;
          Tubeloop[TubeId-1].ucMessageID = msg->ucMessageID;
          if(preg->resultOk == NO_ERROR)
          {
            if(preg->addr == EVENT_REG)
            {
              //  DEBUG_PRINTF("Tube[%d]Event[%x]",TubeId,*preg->data);
              HeaterEventHandler(preg, msg);
            }
            else if(preg->addr == HW_STATUS_REG)
            {
              //  DEBUG_PRINTF("Tube[%d] HW Status:%x",TubeId,*preg->data);
              HW_EventHandler(preg, msg);
            }            
            else
            {
              TubeMessageHandler(preg->slave, msg);
            }
          }
          else
          {
            DEBUG_PRINTF("####Tube[%ld]ERROR MODBUS TELEGRAM READ FAILED!!! %d",TubeId,modbus_result);
            configASSERT(pdFALSE); //This is a fatal as the sequence will halt with no recovery
          }
          break;
        case TIMER_EXPIRED:                               /*Waiting time for tube ended*/
          TubeId = *((long *)(msg->ucData));
          T = &Tubeloop[TubeId-1];
          T->ucMessageID = msg->ucMessageID;
          DEBUG_PRINTF("Tube[%ld]@%s TubeSeq[%s] Stage:%s, Pause:%d", TubeId, tube_states[T->curr.stage],
            signals_txt[(unsigned char)msg->ucMessageID], stageToChar[(unsigned char)T->curr.stage],
            T->pausePendingState);
      #ifdef USE_PAUSE_FEATURE
          if( (Extension == T->curr.stage) && (TUBE_P_PAUSE_REQUESTED == T->pausePendingState) )
          {
            u16 data;
            data = T->pauseTemp;
            WriteTubeHeaterReg(TubeId, SETPOINT_REG, &data, sizeof(data)/2);
            Tubeloop[TubeId-1].state = TUBE_WAIT_P_TEMP;
            /* As we are pausing we do not goto next stage in sequence, this we do on "continue" command*/
          }
          else
      #endif
      #ifdef USE_SYNCHRONOUS_PROTOCOL
          {
            int time_done = TRUE;
            int i;
            
            static int well_sync[16] = {FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};
            well_sync[TubeId - 1] = TRUE;
            DEBUG_PRINTF("Tube %ld Timeout",TubeId);

            // If all running tubes have timed out then proceed
            for(i = 0; i < 16; i++)
            {
              //TUBE_INIT, TUBE_IDLE, TUBE_WAIT_TEMP, TUBE_WAIT_TIME, TUBE_WAIT_P_TEMP, TUBE_PAUSED, TUBE_OUT_OF_DATA, TUBE_NOT_INITIALIZED
              if( (TUBE_WAIT_TIME   == Tubeloop[i].state) || (TUBE_PAUSED      == Tubeloop[i].state) || 
                  (TUBE_OUT_OF_DATA == Tubeloop[i].state) || (TUBE_WAIT_P_TEMP == Tubeloop[i].state))
              { // All running tubes
                if(FALSE == well_sync[i])
                time_done = FALSE;             
              }
            }
            if(TRUE == time_done)
            { //Start next stage on all tubes running
              DEBUG_PRINTF("Last tube timeout");
              for(i = 0; i < 16; i++)
              {
                if(TRUE == well_sync[i])
                {
                  new_msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
                  new_msg->ucMessageID = NEXT_TUBE_STAGE;
                  p=(long *)new_msg->ucData;
                  *p=i+1; // i to TubeId
                  xQueueSend(TubeSequencerQueueHandle, &new_msg, portMAX_DELAY);
                }
                well_sync[i] = FALSE; //Make sure all wells are in the not synched state
              }
            }
          }
      #else
          {
            new_msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
            new_msg->ucMessageID = NEXT_TUBE_STAGE;
            p=(long *)new_msg->ucData;
            *p=TubeId;
            xQueueSend(TubeSequencerQueueHandle, &new_msg, portMAX_DELAY);
          }
      #endif
          break;
        case WRITE_MODBUS_REGS_RES:
          wres = ((WriteModbusRegsRes *)(msg->ucData));
          TubeId = wres->slave;
          Tubeloop[TubeId-1].ucMessageID = msg->ucMessageID;

          modbus_result = wres->resultOk;
          if(modbus_result != NO_ERROR)
          {
            DEBUG_PRINTF("####Tube[%ld]ERROR MODBUS WRITE FAILED!!! %d",TubeId,modbus_result);
            configASSERT(pdFALSE); //This is a fatal as the sequence will halt with no recovery
          }
          DEBUG_PRINTF("Tube[%ld]@%s TubeSeq[%s] Tube[%d]ADDR[%d]size[%d]STATUS[%s]",
            TubeId, tube_states[Tubeloop[TubeId-1].state], signals_txt[(unsigned char)msg->ucMessageID], wres->slave, wres->addr,
            wres->datasize, (wres->resultOk==NO_ERROR)?"PASS":"FAIL");

          if(Tubeloop[TubeId-1].state == TUBE_INIT)         /*No errors on current tube, set it to IDLE so its ready for use*/
          {
            Tubeloop[TubeId-1].state = TUBE_IDLE;
            bool Initialized = TRUE;
            i = 0;
            while(i<NTUBES)
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
          DEBUG_SEQ_PRINTF("***UNHANDLED MESSAGE*** %s",
            signals_txt[(unsigned char)msg->ucMessageID]);
          break;
      };
      vPortFree(msg);                                     /*dealloc the msg*/
    }
  }
}

void vTimerCallback( xTimerHandle pxTimer )
{
  long lArrayIndex;
  xMessage *msg;
  long *p;

  if(NULL != pxTimer)
  {
    /* Which timer expired? */
    lArrayIndex = (long) pvTimerGetTimerID( pxTimer );

    xTimerStop( pxTimer, 0 );
    msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
    msg->ucMessageID = TIMER_EXPIRED;
    p = (long *)msg->ucData;
    *p = lArrayIndex+1;

    DEBUG_IF_PRINTF("Tube[%ld] TIMER_EXPIRED ",lArrayIndex+1);
    xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
  }
}
