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


#if 0
const char *  signals_txt[] = {
  "FIRST_MSG",
  "WRITE_MODBUS_REGS",
  "WRITE_MODBUS_REGS_RES",
  "TIMER_EXPIRED",
  "START_TUBE_SEQ",
  "TUBE_TEST_SEQ",
  "NEXT_TUBE_STATE",
  "READ_MODBUS_REGS",
  "READ_MODBUS_REGS_RES",
  "START_TUBE",
  "DATA_FROM_TUBE",
  "LAST_MSG"
};
#endif




#define STM32F051



//#define ADS_DRDY_PIN GPIO_Pin_7
//#define ADS_DRDY_EXTI EXTI_PinSource7



#define DEBUG
#ifdef DEBUG
/*#define DEBUG_PRINT(fmt, args...)    fprintf(stderr, fmt, ## args)*/
#define DEBUG_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_response(buf);
#else
#define DEBUG_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif

//#define DEBUG_SEQ
#ifdef DEBUG_SEQ
/*#define DEBUG_PRINT(fmt, args...)    fprintf(stderr, fmt, ## args)*/
#define DEBUG_SEQ_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_response(buf);
#else
#define DEBUG_SEQ_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif





#ifdef DEBUG_IF
/*#define DEBUG_PRINT(fmt, args...)    fprintf(stderr, fmt, ## args)*/
#define DEBUG_IF_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_response(buf);
#else
#define DEBUG_IF_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif





#define tube1 0x01
#define tube2 0x02

#define SET_TUBE_TEMP 0x01
#define SET_TUBE_IDLE 0x02


#define tube_status 0x1

#define nTubes     16     /*Number of tubes to use*/
#define NUM_TIMERS nTubes /*There should be 1 for each tube*/

 /* An array to hold handles to the created timers. */
 xTimerHandle xTimers[ NUM_TIMERS ];

 /* An array to hold a count of the number of times each timer expires. */
 long lExpireCounters[ NUM_TIMERS ] = { 0 };
 
 char buf[300]; /*buffer for debug printf*/
 bool Heater4_irq_handled = FALSE;

/* Private variables ---------------------------------------------------------*/
 
typedef enum
{
TUBE_INIT,
TUBE_IDLE,
TUBE_WAIT_TEMP, /*Wait until desired temperature are reached*/
TUBE_WAIT_TIME, /*Wait the specified time in the sequence*/
TUBE_NOT_INITIALIZED
}TubeStates;


#if 0

const char *  tube_states[] = {
"TUBE_INIT",
"TUBE_IDLE",
"TUBE_WAIT_TEMP", /*Wait until desired temperature are reached*/
"TUBE_WAIT_TIME", /*Wait the specified time in the sequence*/
"TUBE_NOT_INITIALIZED"
};
#endif




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
uint16_t temp; /*Settemp in 0.1 degrees*/
uint16_t time; /*time in secs*/
TubestateTypeDef state;    /*Current stage:[M]elting(1), [A]nnealing(2), [E]xtension(3) or [I]ncubation(4) */
}stateCmdTypeDef;

typedef struct
{
TubeStates state;
uint16_t LoopStart;
uint16_t LoopIterations;
int SeqIdx;
stateCmdTypeDef *data;
portCHAR ucMessageID;/*last message processed in statemachine*/
uint16_t event_reg;/*last event read from tube*/
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

const char *  heater[] = {
/*ADS_DRDY,*/
"Heater4-Tube[6-7]",
"Heater1-Tube[0-1]",
"Heater2-Tube[2-3]",
"Heater3-Tube[4-5]",
"Heater5-Tube[8-9]",
"Heater6-Tube[10-11]",
"Heater7-Tube[12-13]",
"Heater8-Tube[14-15]",
"nExtiGpio"
};

ExtiGpioTypeDef tube2heater[]={
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
{6,7},/*Heater4*/
{0,1},/*Heater1*/	
{2,3},/*Heater2*/
{4,5},/*Heater3*/
{8,9},/*Heater5*/
{10,11},/*Heater6*/
{12,13},/*Heater7*/
{14,15} /*Heater8*/
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
#if 0
stateCmdTypeDef TubeSeq[nTubes][6]={/*{{600,10000,Melting},{0,3,LoopStart},{400,5000,Annealing},{550,5000,Extension},{0,0,LoopEnd},{0,0,End}},*/
                                   	{{300,100,Melting},{550,1800,Annealing},{400,1800,Extension},{0,0,End},{0,0,End}},
                                	{{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                 	{{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                    {{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                    {{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                	{{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                 	{{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                    {{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                    {{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                    {{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                    {{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                    {{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                    {{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                    {{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                    {{500, 9000,Melting},{0,12,LoopStart},{940,200,Annealing},{0,0,LoopEnd},{0,0,End}},
                                    {{500, 8000,Melting},{0,2,LoopStart},{930,3000,Annealing},{0,0,LoopEnd},{0,0,End}}};
#endif
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
                          {TUBE_NOT_INITIALIZED,0,0,0,NULL,0,0xFFFF}};



extern xQueueHandle ModbusQueueHandle;
extern xQueueHandle TubeSequencerQueueHandle;



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
long x = TubeNum;
 DEBUG_IF_PRINTF("Tube[%d]StartTubeTimer Time[%d]",TubeNum,time);
/*time in mSec*/
#if 1
     if( xTimers[ x ] == NULL )
     {
         /* The timer was not created. */
     }
     else
     {
         /* Start the timer.  No block time is specified, and even if one was
         it would be ignored because the RTOS scheduler has not yet been
         started. */
         
		 if(xTimerChangePeriod( xTimers[ x ],100 * time,100)!= pdPASS )
	 	{
	 	}
         if( xTimerStart( xTimers[ x ], 0 ) != pdPASS )
         {
             /* The timer could not be set into the Active state. */
         }
     }
#endif
}



StopTubeTimer(long TubeNum)
{

  if( xTimers[ TubeNum ] == NULL )
	   {
		   /* The timer was not created. */
	   }
	   else
	   {
		  xTimerStop( xTimers[ TubeNum ], 0 );
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
//if(GPIO_PinSource3 == p->PINSOURCE)
//{
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
//}else
//{
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//}
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;

  GPIO_EXTILineConfig(p->PORTSOURCE , p->PINSOURCE);
  EXTI_Init(&EXTI_InitStructure);
  /* Clear any pending interrupts */
  EXTI_ClearITPendingBit(p->EXTI_LINE);
  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannel = p->IRQ_CH;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  p++;
}
  //ExtIrqDisable(Heater1);  
#if 0 
   gpio_extint_t *T =  &gpio_EXTI_CNF[0];
  while(T->PINSOURCE != 0xFF)
  {
  DEBUG_PRINTF("%d %d %d %d ",T->PINSOURCE,T->PORTSOURCE,T->EXTI_LINE,T->IRQ_CH);
  T++;
  }
#endif

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


  //DEBUG_PRINTF("ENABLE IRQ %d %d %d %d ",gpio_EXTI_CNF[heater].PINSOURCE,gpio_EXTI_CNF[heater].PORTSOURCE,gpio_EXTI_CNF[heater].EXTI_LINE,gpio_EXTI_CNF[heater].IRQ_CH);

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
    WriteModbusRegsReq *p;
if(reg == SET_TUBE_IDLE)
{
  DEBUG_IF_PRINTF("Tube[%d] MODBUS WRITE_REG[SET_TUBE_IDLE] ",tube);
}
else
{
#if 1 
	
		msg=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+datasize*sizeof(u16));
		*data=((*data&0xFF)<<8)|(*data>>8);
		msg->ucMessageID=WRITE_MODBUS_REGS;
		p=(WriteModbusRegsReq *)msg->ucData;
		p->slave=tube+0x02;
		p->addr=reg;
		memcpy(p->data, data, datasize*sizeof(u16));
		p->datasize=datasize;
	    p->reply=TubeSequencerQueueHandle;
		xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
	//	DEBUG_PRINTF("Tube[%d]MODBUS WRITE_REG ID[%d] ADR[%d] data[%x]SIZE[%d]",tube, tube,p->addr,(((u16)(p->data[0])<<8)|(p->data[1])),p->datasize);
#else /*Simulate response from Heater CPU*/
    ReadModbusRegsRes *p;

    msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsRes));
    msg->ucMessageID=READ_MODBUS_REGS_RES;
	p=(ReadModbusRegsRes *)msg->ucData;
	p->slave=tube+2;
	p->addr=EVENT_REG;
	p->data[0]=SEQUENCE_EVENT_TUBE1|SEQUENCE_EVENT_TUBE2;
	//memcpy(p->data, data, datasize);
	p->datasize=datasize;
	p->resultOk=TRUE;
	xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
    DEBUG_PRINTF("Tube[%d] MODBUS WRITE_REG[%d]data[%d]",tube,p->addr,*data);
#endif

}


}



void ReadTubeHeaterReg(u8 tube, u16 reg, u16 datasize, bool from_isr)
{
#if 1
	{
	//tube=0x02;
	//reg = 16;
	//datasize = 1;
    xMessage *msg;
    ReadModbusRegsReq *p;
	portBASE_TYPE taskWoken = pdTRUE;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq));
    msg->ucMessageID=READ_MODBUS_REGS;
    p=(ReadModbusRegsReq *)msg->ucData;
    p->slave=tube+0x02;
    p->addr=reg;
    p->datasize=datasize;
    p->reply=TubeSequencerQueueHandle;
	if (from_isr == TRUE)
	{
	  xQueueSendFromISR(ModbusQueueHandle,&msg,&taskWoken);
	}else
	{
      xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
	}
	DEBUG_IF_PRINTF("Tube[%d]MODBUS READ_REG ID[%d] ADR[%d] SIZE[%d] CALLER:%s",tube, p->slave,p->addr, p->datasize, (from_isr == TRUE) ?"ISR":"NORM");
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

DEBUG_IF_PRINTF("Tube[%d]MODBUS READ_REG[tube_status] ",tube);

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
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;

GPIO_Init(GPIOD, &GPIO_InitStructure);

/* GPIOC Configuration: Channel 1, 2 and 3 as alternate function push-pull */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
GPIO_Init(GPIOC, &GPIO_InitStructure);
}



HeaterEventHandler(ExtiGpioTypeDef Heater)
{
  switch (Heater)
  {
  case Heater1:
    ReadTubeHeaterReg(0,EVENT_REG/*ADC_code1_reg*/,5, TRUE);
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

#define heater2tube(heater) (heater*2)-2

void EXTI_Handler(void)
{
ExtiGpioTypeDef ExtiGpio = Heater1; 
  /*ADS DRDY and Heater4_eventline is sharing EXTI_Line10 IRQ, so we need to find the source of IRQ then we read the status of the gpio pin*/
 

#if 1
  if(SET == EXTI_GetFlagStatus(ADS_EXTI_LINE))
  {
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10/*ADS_DRDY_PINSOURCE*/) == Bit_RESET)
    {
      ADS_Handler();
     // DEBUG_PRINTF("I");
    }
	if((GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)/*gpio_EXTI_CNF[Heater4].PINSOURCE */== Bit_SET)&&(Heater4_irq_handled == FALSE))
  	{
    	//HeaterEventHandler(Heater4);
    	ReadTubeHeaterReg(6,EVENT_REG/*ADC_code1_reg*/,5, TRUE);
		Heater4_irq_handled = TRUE;
		DEBUG_IF_PRINTF("INTERRUPT-READ STATUS ON HEATER[%s]",heater[Heater4]);
	}
    EXTI_ClearITPendingBit(ADS_EXTI_LINE);
  }
      
#endif
  while(ExtiGpio < nExtiGpio) /*Always starts at Heater1 since */
  	{
	if (SET == EXTI_GetFlagStatus(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE))
    {
	  DEBUG_IF_PRINTF("INTERRUPT-READ STATUS ON HEATER[%s]",heater[ExtiGpio]);
	  ReadTubeHeaterReg(heater2tube[ExtiGpio].tube_1/*heater2tube(ExtiGpio)*/,EVENT_REG/*ADC_code1_reg*/,5, TRUE);

	 // HeaterEventHandler(Heater1/*ExtiGpio*/);
 	  EXTI_ClearITPendingBit(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE);
	}
  ExtiGpio++;
  }

}
uint16_t dac_2_temp(signed short dac)
{
  signed long res;
  res =(((dac*29549)/10000)+ 77175)/100 ;
  
 // DEBUG_PRINTF("DAC2TEMP[%4x]=[%d.%dC][%4x]",dac,res/10,res%10,res);
  return (uint16_t)res;
}




signed short temp_2_dac(int16_t temp)
{
  signed short res;
  res = (10000*(((long)temp*100)-77175))/29549;
 // DEBUG_PRINTF("TEMP2DAC[%d.%dC]=[%d][%4x]",temp/10,temp%10,res,res);
  return res;
}

int create_seq(long TubeId,int Nstates )
{
      Tubeloop_t *T = &Tube[TubeId];
	  T->data = pvPortMalloc(sizeof(stateCmdTypeDef)*Nstates);
      return (int)T->data;
}


bool insert_state_to_seq(long TubeId,uint16_t time, uint16_t temp )
{
    bool result = TRUE;
	Tubeloop_t *T = &Tube[TubeId];
	stateCmdTypeDef *TSeq = &(T->data[T->SeqIdx]);

//	stateCmdTypeDef *TSeq = &TubeSeq[TubeId][T->SeqIdx];

	
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

void stop_tube_seq( long TubeId)
{
	u16 data;

    if(Tube[TubeId].state == TUBE_WAIT_TIME)
    {
   	  StopTubeTimer(TubeId);
    }
	data = SET_IDLE_MODE;
	WriteTubeHeaterReg(TubeId,TUBE_COMMAND_REG,&data,sizeof(data)/2); /*Set idle mode for tube to stop heating*/
	Tube[TubeId].SeqIdx = 0;
    Tube[TubeId].state = TUBE_IDLE;

	
}




void get_tube_state(long TubeId)
{
  //if(TubeId < 17)
  //{
  if(Tube[TubeId].data != NULL)
  	{
  	  DEBUG_PRINTF("State for Tube SEQ_ID:%d state:%s CURR time:%d temp:%d LAST MSG:%s EVENT:%x",Tube[TubeId].SeqIdx,tube_states[Tube[TubeId].state],Tube[TubeId].data[Tube[TubeId].SeqIdx].temp,Tube[TubeId].data[Tube[TubeId].SeqIdx].time,signals_txt[Tube[TubeId].ucMessageID],Tube[TubeId].event_reg);
  	}else
  	{
  	  DEBUG_PRINTF("NO VALID SEQ FOUND State for Tube SEQ_ID:%d state:%s LAST MSG:%s EVENT:%x",Tube[TubeId].SeqIdx,tube_states[Tube[TubeId].state],signals_txt[Tube[TubeId].ucMessageID],Tube[TubeId].event_reg);
  	}
  	//DEBUG_PRINTF("Tube[%d]EVENT[%x]:%s-%s-%s-%s-%s-%s-%s-%s",TubeId,Tube[TubeId].event_reg,((Tube[TubeId].event_reg & SEQUENCE_EVENT_TUBE1)?"S_EV_T1 ":" "),((Tube[TubeId].event_reg & SEQUENCE_EVENT_TUBE1)?"S_EV_T2 ":" "),((Tube[TubeId].event_reg & INIT_HW_ERROR_TUBE1)?"HW_ERR_T1 ":" "),((Tube[TubeId].event_reg & INIT_HW_ERROR_TUBE2)?"HW_ERR_T2 ":" "),((Tube[TubeId].event_reg & TUBE1_NOT_PRESENT)?"NO_T1 ":" "),((Tube[TubeId].event_reg & TUBE2_NOT_PRESENT)?"NO_T2 ":" "),((Tube[TubeId].event_reg & EVENT6)?"EV6 ":" "),((Tube[TubeId].event_reg & EVENT7)?"EV7 ":" "));

  //}else
  //{
   // for(TubeId=0;TubeId < 16; TubeId++)
    //{
   // 	DEBUG_PRINTF("State for Tube SEQ_ID:%d state:%s CURR time:%d temp:%d LAST MSG:%s EVENT[%x]:%s%s%s%s%s%s%s%s",Tube[TubeId].SeqIdx,tube_states[Tube[TubeId].state],Tube[TubeId].data[Tube[TubeId].SeqIdx].temp,Tube[TubeId].data[Tube[TubeId].SeqIdx].time,signals_txt[Tube[TubeId].ucMessageID],Tube[TubeId].event_reg,((Tube[TubeId].event_reg & SEQUENCE_EVENT_TUBE1)?"S_EV_T1 ":" "),((Tube[TubeId].event_reg & SEQUENCE_EVENT_TUBE1)?"S_EV_T2 ":" "),((Tube[TubeId].event_reg & INIT_HW_ERROR_TUBE1)?"HW_ERR_T1 ":" "),((Tube[TubeId].event_reg & INIT_HW_ERROR_TUBE2)?"HW_ERR_T2 ":" "),((Tube[TubeId].event_reg & TUBE1_NOT_PRESENT)?"NO_T1 ":" "),((Tube[TubeId].event_reg & TUBE2_NOT_PRESENT)?"NO_T2 ":" "),((Tube[TubeId].event_reg & EVENT6)?"EV6 ":" "),((Tube[TubeId].event_reg & EVENT7)?"EV7 ":" "));
  //  	DEBUG_PRINTF("Tube[%d]EVENT[%x]:%s-%s-%s-%s-%s-%s-%s-%s",TubeId,Tube[TubeId].event_reg,((Tube[TubeId].event_reg & SEQUENCE_EVENT_TUBE1)?"S_EV_T1 ":" "),(Tube[TubeId].event_reg & SEQUENCE_EVENT_TUBE1)?"S_EV_T2 ":" "),(Tube[TubeId].event_reg & INIT_HW_ERROR_TUBE1)?"HW_ERR_T1 ":" "),(Tube[TubeId].event_reg & INIT_HW_ERROR_TUBE2)?"HW_ERR_T2 ":" "),(Tube[TubeId].event_reg & TUBE1_NOT_PRESENT)?"NO_T1 ":" "),(Tube[TubeId].event_reg & TUBE2_NOT_PRESENT)?"NO_T2 ":" "),(Tube[TubeId].event_reg & EVENT6)?"EV6 ":" "),(Tube[TubeId].event_reg & EVENT7)?"EV7 ":" "));
   // }
 // }
//	ReadTubeHeaterReg(TubeId,EVENT_REG,5,FALSE);


#if 0
	Tube[TubeId].SeqIdx
	tube_states[Tube[TubeId].state]
	signals_txt[Tube[TubeId].ucMessageID]
	Tube[TubeId].data[Tube[TubeId].SeqIdx].temp
	Tube[TubeId].data[Tube[TubeId].SeqIdx].time
#endif
}


void TubeStateHandler(long TubeId,xMessage *msg)
{

  xMessage *new_msg;
  long *p;
  u16 data;
  int a = 0;
  uint16_t temps[]={0,50,100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1050,1100,1150,1200,1250,1300,1350,1400,1450,1500,1550,1600,1650,1700,1750,1800};
  uint16_t i = 0;
  Tubeloop_t *T = &Tube[TubeId];
 // stateCmdTypeDef *TSeq = &TubeSeq[TubeId][Tube[TubeId].SeqIdx];

if (T->data != NULL)
	
{
  stateCmdTypeDef *TSeq = &(T->data[T->SeqIdx]);
  switch (TSeq->state)
  {
  case LoopStart:/*{3,0,LoopStart}*/
    T->LoopStart = T->SeqIdx+1;/*Loop from next entry*/
    T->LoopIterations = TSeq->time;
	DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] *** LOOPSTART @StarID[%d] Iterations[%d] ***",TubeId,tube_states[T->state],signals_txt[msg->ucMessageID],T->SeqIdx,T->LoopIterations);
    T->SeqIdx++; /*Going to next sequence*/
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
//	data = 1;
//    WriteTubeHeaterReg(TubeId,0x12,&data,1);/*Set tube state to idle = 1*/
	data = SET_IDLE_MODE;
	WriteTubeHeaterReg(TubeId,TUBE_COMMAND_REG,&data,sizeof(data)/2); /*Set idle mode for tube to stop heating*/
	T->SeqIdx = 0;
  break;
  case Melting:
  case Annealing:
  case Extension:
  case Incubation:
    DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] Seq Active set new temp %d.%02dC",TubeId,tube_states[T->state],signals_txt[msg->ucMessageID],TSeq->temp/10,TSeq->temp%10);/*ASCII 155 norm 167*/
    while(T->data[a].state != End)
   	{
      //DEBUG_PRINTF("%d.%02dC @ %d.%02dsecs ",T->data[a].temp/10,T->data[a].temp%10,T->data[a].time/10,T->data[a].time%10);
	  a++;
   	}
#if 0
	while(temps[i]<1800)
	{
	dac_2_temp(temp_2_dac(temps[i]));
    i++;
	}
#endif

	data = SET_AUTOMATIC_MODE;
	WriteTubeHeaterReg(TubeId,TUBE_COMMAND_REG,&data,sizeof(data)/2); /*Set idle mode for tube to stop heating*/

	data = temp_2_dac(TSeq->temp);
		
//    ReadTubeHeaterReg(2,EVENT_REG,1);
	dac_2_temp(0xDC14);

	WriteTubeHeaterReg(TubeId,SETPOINT_REG,&data,sizeof(data)/2);
	T->state = TUBE_WAIT_TEMP;
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
/*
void json_test()
{
		const char *js;
	int r;
	jsmn_parser p;
	jsmntok_t tokens[10];

	js = "{\"a\": 0}";

	jsmn_init(&p);
	r = jsmn_parse(&p, js, tokens, 10);
	tokens[0].type


}*/

void TubeSequencerTask( void * pvParameter)
{
short usData;
xMessage *msg,*new_msg;
long TimerId,TubeId;
u16 modbus_addr;
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
//while(i<nExtiGpio)
//{
 // ExtIrqDisable(Heater1);
//}


while(1)
{
  /*wait for queue msg*/
  if( xQueueReceive( TubeSequencerQueueHandle, &msg, portMAX_DELAY) == pdPASS )
  {
	DEBUG_PRINTF("TubeSeq[%s]",signals_txt[msg->ucMessageID]);
	switch(msg->ucMessageID)
	{
	case TUBE_TEST_SEQ:
		TubeId = *((long *)(msg->ucData));
        Tube[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
		 modbus_data[0]=1;
	     //WriteTubeHeaterReg(0,EVENT_REG,&modbus_data[0], 1); /*Read & Clear pending events on heater*/
		
		//for(i=0;i<15;i++/*i=i+2*/)
		//{
   		  ReadTubeHeaterReg(TubeId,EVENT_REG,1, FALSE); /*Read & Clear pending events on heater*/
		//}
		while(heater != nExtiGpio)
		{
		 // ExtIrqEnable(heater);
		 // DEBUG_PRINTF("ENABLE IRQ[%d]",heater);
		  heater++;
		}
//		for(i=0;i<15;i=i++)
//		{

	//	  WriteTubeHeaterReg(i,EVENT_REG,&modbus_data[0], 1); /*Read & Clear pending events on heater*/
//		}
		//
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
	  //DEBUG_PRINTF("-->Tube[%d]@%s TubeSeq[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID]);
 	break;
	case READ_MODBUS_REGS_RES:
		preg=(ReadModbusRegsRes *)msg->ucData;
        TubeId = preg->slave-2;
        Tube[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
		modbus_addr = preg->addr;
	    /*modbus_data =  preg->data[1];*/
		modbus_result = preg->resultOk;
		for(i=0;i<(preg->datasize);i++)
		{
		  modbus_data[i] =(((u16)(preg->data[i*2])<<8)|(preg->data[(i*2)+1]));
		}
	    DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s]ADDR[%d]size[%d]data[%x]STATUS[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],preg->addr,preg->datasize,/*(((u16)(preg->data[0])<<8)|(preg->data[1]))*/modbus_data[0],(preg->resultOk==TRUE)?"PASS":"FAIL");
		
        if(preg->addr == SETPOINT_REG)
        {
		 DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s] SetPoint[%d]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],dac_2_temp(modbus_data[0])/10);
		 //ReadTubeHeaterReg(2,EVENT_REG,1);

		}
		if(preg->addr == EVENT_REG)
		{
           if(preg->resultOk==TRUE)
           	{
			   if (TubeId == 6)
			   {
			    	Heater4_irq_handled == FALSE; /*Now we have read the content of the event register and are ready to recieve a new IRQ from Heater4*/
			   }
			  DEBUG_PRINTF("Tube[%d]EVENT[%x]:%s-%s-%s-%s-%s-%s-%s-%s",TubeId,modbus_data[0],((modbus_data[0] & SEQUENCE_EVENT_TUBE1)?"S_EV_T1 ":" "),((modbus_data[0] & SEQUENCE_EVENT_TUBE1)?"S_EV_T2 ":" "),((modbus_data[0] & INIT_HW_ERROR_TUBE1)?"HW_ERR_T1 ":" "),((modbus_data[0] & INIT_HW_ERROR_TUBE2)?"HW_ERR_T2 ":" "),((modbus_data[0] & TUBE1_NOT_PRESENT)?"NO_T1 ":" "),((modbus_data[0] & TUBE2_NOT_PRESENT)?"NO_T2 ":" "),((modbus_data[0] & EVENT6)?"EV6 ":" "),((modbus_data[0] & EVENT7)?"EV7 ":" "));
			  Tube[TubeId].event_reg = modbus_data[0];
              if(Tube[TubeId].state == TUBE_NOT_INITIALIZED)/*&&(modbus_data[0] & INIT_HW_ERROR_TUBE1)*//*No errors on current tube, set it to IDLE so its ready for use*/
  			  {
     			 Tube[TubeId].state = TUBE_IDLE;
     			 modbus_data[0]=1;
     			 ExtIrqEnable(tube2heater[TubeId]);
     			 //DEBUG_PRINTF("ENABLE IRQ ON HEATER[%d]",tube2heater[TubeId]);
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
          if((Tube[TubeId].state == TUBE_WAIT_TEMP)&&((modbus_data[0] & SEQUENCE_EVENT_TUBE1)||(modbus_data[0] & SEQUENCE_EVENT_TUBE2)))
          	{
		     DEBUG_PRINTF("Tube[%d] TEMPERATURE REACHED ? %x-%d - %d - %d - %d",TubeId,modbus_data[1],dac_2_temp(modbus_data[1])/10,dac_2_temp(modbus_data[2])/10,dac_2_temp(modbus_data[3])/10,dac_2_temp(modbus_data[4])/10);
          	}
        /*reg = p->addr;*/
		
	    //DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] TEMP reached start timer %d.%02d Sec",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],TubeSeq[TubeId][Tube[TubeId].SeqIdx].time/1000,TubeSeq[TubeId][Tube[TubeId].SeqIdx].time%1000);
	  if((Tube[TubeId].state == TUBE_WAIT_TEMP)/*&&(modbus_data[0]==0x01)*/) /*The heater has signalled an IRQ and here the status of the tube is read*/
	  {
		   Tubeloop_t *T = &Tube[TubeId];
		  stateCmdTypeDef *TSeq = &(T->data[T->SeqIdx]);

	    DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s] TEMP reached start timer %d.%02d Sec",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],/*TubeSeq[TubeId][Tube[TubeId].SeqIdx].time*/TSeq->time/10,/*TubeSeq[TubeId][Tube[TubeId].SeqIdx].time%10*/TSeq->time%10);
 	    StartTubeTimer(TubeId,/*TubeSeq[TubeId][Tube[TubeId].SeqIdx].time*/TSeq->time);
	    Tube[TubeId].state = TUBE_WAIT_TIME;
	  }
	  }
	break;
	case TIMER_EXPIRED:                       /*Waiting time for tube ended*/
	  TubeId = *((long *)(msg->ucData));
      Tube[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
	  DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID]);
	  Tube[TubeId].SeqIdx++; /*Going to next sequence*/
	  new_msg=pvPortMalloc(sizeof(xMessage)+sizeof(long));
	  new_msg->ucMessageID=NEXT_TUBE_STATE;
	  p=(long *)new_msg->ucData;
	  *p=TubeId;
	//  DEBUG_PRINTF("@Tube[%d] TIMER_EXPIRED ",TubeId);
	  xQueueSend(TubeSequencerQueueHandle, &new_msg, portMAX_DELAY);
	break;
    case WRITE_MODBUS_REGS_RES:
      wres = ((WriteModbusRegsRes *)(msg->ucData));
      TubeId = wres->slave-2;
      Tube[TubeId].ucMessageID = msg->ucMessageID; /*Log last state for tube*/
	  modbus_addr = wres->addr;
	  modbus_result = wres->resultOk;
	  
	  DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s] Tube[%d]ADDR[%d]size[%d]STATUS[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],wres->slave-2,wres->addr,wres->datasize,(wres->resultOk==TRUE)?"PASS":"FAIL");
	//  DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID]);
	 // ReadTubeHeaterReg(TubeId+2,setPoint_reg,1);/*DEBUG remove later Dummy to trigger read*/
	break;
	default:
	  DEBUG_SEQ_PRINTF("Tube[%d]@%s TubeSeq[%s]State[%s] ***UNHANDLED STATE***",0xFF,tube_states[Tube[0].state],signals_txt[msg->ucMessageID]);
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
	portBASE_TYPE taskWoken = pdFALSE;

	/* Optionally do something if the pxTimer parameter is NULL. */
	configASSERT( pxTimer );

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
	*p=lArrayIndex;
	
	DEBUG_IF_PRINTF("Tube[%d] TIMER_EXPIRED ",lArrayIndex);
    xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);

}
