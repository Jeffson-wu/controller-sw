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



const char *  signals_txt[] = {
  "FIRST_MSG",
  "WRITE_MODBUS_REGS",
  "WRITE_MODBUS_REGS_RES",
  "TIMER_EXPIRED",
  "START_TUBE_SEQ",
  "NEXT_TUBE_STATE",
  "READ_MODBUS_REGS",
  "READ_MODBUS_REGS_RES",
  "START_TUBE",
  "DATA_FROM_TUBE",
  "LAST_MSG"
};





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

/* Private variables ---------------------------------------------------------*/
 
typedef enum
{
TUBE_INIT,
TUBE_IDLE,
TUBE_WAIT_TEMP, /*Wait until desired temperature are reached*/
TUBE_WAIT_TIME, /*Wait the specified time in the sequence*/
TUBE_NOT_INITIALIZED
}TubeStates;
const char *  tube_states[] = {
"TUBE_INIT",
"TUBE_IDLE",
"TUBE_WAIT_TEMP", /*Wait until desired temperature are reached*/
"TUBE_WAIT_TIME", /*Wait the specified time in the sequence*/
"TUBE_NOT_INITIALIZED"
};

typedef struct
{
TubeStates state;
uint16_t LoopStart;
uint16_t LoopIterations;
int SeqIdx;
}Tubeloop_t;



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

Tubeloop_t Tube[nTubes]= {{TUBE_IDLE,0,0,0},
	                      {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0},
                          {TUBE_IDLE,0,0,0}};



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
if(GPIO_PinSource3 == p->PINSOURCE)
{
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
}else
{
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
}
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;

  GPIO_EXTILineConfig(p->PORTSOURCE , p->PINSOURCE);
  EXTI_Init(&EXTI_InitStructure);
  /* Clear any pending interrupts */
  EXTI_ClearITPendingBit(p->EXTI_LINE);

  NVIC_InitStructure.NVIC_IRQChannel = p->IRQ_CH;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  p++;
}
  ExtIrqDisable(Heater1);  
 


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
    WriteModbusRegsReq *p;
    msg=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+datasize*sizeof(u16));
if(reg == SET_TUBE_IDLE)
{
  DEBUG_IF_PRINTF("Tube[%d] MODBUS WRITE_REG[SET_TUBE_IDLE] ",tube);
}
else
{
#if 1
	
		*data=((*data&0xFF)<<8)|(*data>>8);
		msg->ucMessageID=WRITE_MODBUS_REGS;
		p=(WriteModbusRegsReq *)msg->ucData;
		p->slave=tube+0x02;
		p->addr=reg;
		memcpy(p->data, data, datasize*sizeof(u16));
		p->datasize=datasize;
	    p->reply=TubeSequencerQueueHandle;
		xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);
		DEBUG_IF_PRINTF("Tube[%d]MODBUS WRITE_REG ID[%d] ADR[%d] data[%x]SIZE[%d]",tube, p->slave,p->addr,(((u16)(p->data[0])<<8)|(p->data[1])),p->datasize);
#else
	msg->ucMessageID=READ_MODBUS_REGS_RES;
	p=(WriteModbusRegsReq *)msg->ucData;
	p->slave=tube;
	p->addr=reg;
	memcpy(p->data, data, datasize);
	p->datasize=datasize;
	xQueueSend(TubeSequencerQueueHandle, &msg, portMAX_DELAY);
    DEBUG_IF_PRINTF("Tube[%d] MODBUS WRITE_REG[SET_TUBE_TEMP]Temp[%d�C]",tube,*data);
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
	//DEBUG_IF_PRINTF("Tube[%d]MODBUS READ_REG ID[%d] ADR[%d] SIZE[%d]",tube, p->slave,p->addr, p->datasize);
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
    	//HeaterEventHandler(Heater4);
	}
    EXTI_ClearITPendingBit(ADS_EXTI_LINE);
  }
      
#endif
  while(ExtiGpio < nExtiGpio) /*Always starts at HEater1 since */
  	{
	if (SET == EXTI_GetFlagStatus(gpio_EXTI_CNF[ExtiGpio].EXTI_LINE))
    {
	  DEBUG_PRINTF("INTERRUPT-READ STATUS ON HEATER[%d]",ExtiGpio);
	  HeaterEventHandler(Heater1/*ExtiGpio*/);
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





void TubeStateHandler(long TubeId,xMessage *msg)
{

  xMessage *new_msg;
  long *p;
  u16 data;
  uint16_t temps[]={0,50,100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1050,1100,1150,1200,1250,1300,1350,1400,1450,1500,1550,1600,1650,1700,1750,1800};
  uint16_t i = 0;
  stateCmdTypeDef *TSeq = &TubeSeq[TubeId][Tube[TubeId].SeqIdx];
  Tubeloop_t *T = &Tube[TubeId];

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
	data = 1;
    WriteTubeHeaterReg(TubeId,0x12,&data,1);/*Set tube state to idle = 1*/
	T->SeqIdx = 0;
  break;
  case Melting:
  case Annealing:
  case Extension:
  case Incubation:
    DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] Seq Active set new temp %d.%02dC",TubeId,tube_states[T->state],signals_txt[msg->ucMessageID],TSeq->temp/10,TSeq->temp%10);/*ASCII 155 norm 167*/
    
#if 0
	while(temps[i]<1800)
	{
	dac_2_temp(temp_2_dac(temps[i]));
    i++;
	}
#endif
	data = temp_2_dac(TSeq->temp);
		
//    ReadTubeHeaterReg(2,EVENT_REG,1);
	dac_2_temp(0xDC14);

	WriteTubeHeaterReg(TubeId,SETPOINT_REG,&data,sizeof(data)/2);
	T->state = TUBE_WAIT_TEMP;
  break;
  default:
  break;
}


}


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

char text1[]="{\n\"name\": \"Jack (\\\"Bee\\\") Nimble\", \n\"format\": {\"type\":		 \"rect\", \n\"width\": 	 1920, \n\"height\":	 1080, \n\"interlace\":  false,\"frame rate\": 24\n}\n}";	


InitTubeTimers();
//while(i<nExtiGpio)
//{
  ExtIrqDisable(Heater1);
//}


while(1)
{
  /*wait for queue msg*/
  if( xQueueReceive( TubeSequencerQueueHandle, &msg, portMAX_DELAY) == pdPASS )
  {
	//DEBUG_PRINTF("TubeSeq[%s]",signals_txt[msg->ucMessageID]);
	switch(msg->ucMessageID)
	{
	case START_TUBE_SEQ:
		TubeId = *((long *)(msg->ucData));
		if((Tube[TubeId].state == TUBE_IDLE) && (Tube[TubeId].SeqIdx==0))
		{
 		  ReadTubeHeaterReg(TubeId,EVENT_REG,1, FALSE); /*Read & Clear pending events on heater*/
		  TubeStateHandler(TubeId,msg);
		}else
		{
	      DEBUG_PRINTF("Tube[%d]@%s FAILED TO START TUBE SEQUENCE ",TubeId,tube_states[Tube[TubeId].state]);
		}
	break;
    case NEXT_TUBE_STATE:
      TubeId = *((long *)(msg->ucData));
	  TubeStateHandler(TubeId,msg);
	  //DEBUG_PRINTF("-->Tube[%d]@%s TubeSeq[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID]);
 	break;
	case READ_MODBUS_REGS_RES:
		preg=(ReadModbusRegsRes *)msg->ucData;
        TubeId = preg->slave-2;
		modbus_addr = preg->addr;
	    /*modbus_data =  preg->data[1];*/
		modbus_result = preg->resultOk;
		for(i=0;i<(preg->datasize);i++)
		{
		  modbus_data[i] =(((u16)(preg->data[i*2])<<8)|(preg->data[(i*2)+1]));
		}
	    DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] Tube[%d]ADDR[%d]size[%d]data[%x]STATUS[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],preg->slave,preg->addr,preg->datasize,/*(((u16)(preg->data[0])<<8)|(preg->data[1]))*/modbus_data[0],(preg->resultOk==TRUE)?"PASS":"FAIL");
		
        if(preg->addr == SETPOINT_REG)
        {
		 DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] SetPoint[%d]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],dac_2_temp(modbus_data[0])/10);
		 //ReadTubeHeaterReg(2,EVENT_REG,1);

		}
		if(preg->addr == EVENT_REG)
		{
		  DEBUG_PRINTF("EVENT = %d -> TEMPERATURE REACHED ? %x-%d - %d - %d - %d",modbus_data[0],modbus_data[1],dac_2_temp(modbus_data[1])/10,dac_2_temp(modbus_data[2])/10,dac_2_temp(modbus_data[3])/10,dac_2_temp(modbus_data[4])/10);
		
        /*reg = p->addr;*/
		
	    //DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] TEMP reached start timer %d.%02d Sec",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],TubeSeq[TubeId][Tube[TubeId].SeqIdx].time/1000,TubeSeq[TubeId][Tube[TubeId].SeqIdx].time%1000);
	  if((Tube[TubeId].state == TUBE_WAIT_TEMP)&&(modbus_data[0]==0x01)) /*The heater has signalled an IRQ and here the status of the tube is read*/
	  {
	    DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] TEMP reached start timer %d.%02d Sec",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],TubeSeq[TubeId][Tube[TubeId].SeqIdx].time/10,TubeSeq[TubeId][Tube[TubeId].SeqIdx].time%10);
 	    StartTubeTimer(TubeId,TubeSeq[TubeId][Tube[TubeId].SeqIdx].time);
	    Tube[TubeId].state = TUBE_WAIT_TIME;
	  }
	  }
	break;
	case TIMER_EXPIRED:                       /*Waiting time for tube ended*/
	  TubeId = *((long *)(msg->ucData));
	  DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID]);
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
	  modbus_addr = wres->addr;
	  modbus_result = wres->resultOk;
	  
	  DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s] Tube[%d]ADDR[%d]size[%d]STATUS[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID],wres->slave,wres->addr,wres->datasize,(wres->resultOk==TRUE)?"PASS":"FAIL");
	//  DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s]",TubeId,tube_states[Tube[TubeId].state],signals_txt[msg->ucMessageID]);
	 // ReadTubeHeaterReg(TubeId+2,setPoint_reg,1);/*DEBUG remove later Dummy to trigger read*/
	break;
	default:
	  DEBUG_PRINTF("Tube[%d]@%s TubeSeq[%s]State[%s] ***UNHANDLED STATE***",0xFF,tube_states[Tube[0].state],signals_txt[msg->ucMessageID]);
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
