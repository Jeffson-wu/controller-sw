/**
  ******************************************************************************
  * @file    cooleandlidtask.c
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    30-Oct -2013
  * @brief   Cooler and lid heater task
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
  *
  ******************************************************************************
  * TIM3,CH1:
  * TIM3,CH2:
  * TIM3,CH3:
  * TIM4,CH3:
  * TIM4,CH4:
  * ADC CH0:
  * ADC CH1:
  * ADC CH2:
  * ADC CH3: 
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
#include "timers.h"
#include "semphr.h"
#include "signals.h"
#include "ads1148.h"
#include "pid.h"
#include "pwm.h"
#include "gdi.h"
#include "util.h"

/* ---------------------------------------------------------------------------*/
//#define DEBUG /*General debug shows state changes of tubes (new temp, new time etc.)*/
//#define DEBUG_COOL
//#define STANDALONE /*Defines if the M3 Runs with or without Linux box*/

char buf[20];
#define PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif
#define LOCK_OUTPUT GPIO_Pin_8
#define USE_CL_DATA_LOGGING
#ifdef USE_CL_DATA_LOGGING
  #define CL_SAMPLES_PER_LOG  10    //Each log is the avarage over this number of samples
  #define CL_LOG_ELEMENT_SIZE 4     //Log all four sensors
  #define CL_LOG_QUEUE_SIZE   4     //Queue length
#endif
/* Private typedef -----------------------------------------------------------*/
typedef enum {
  STOP_STATE,
  MANUAL_STATE,
  CTRL_OPEN_LOOP_STATE,
  CTRL_CLOSED_LOOP_STATE,
  nCTRL_STATES
} controllerState_t;

typedef enum {
  PELTIER_1,
  //PELTIER_2,
  //PELTIER_3,
  nPELTIER
} peltierID_t;

typedef enum {
  FAN_1,
  //PELTIER_2,
  //PELTIER_3,
  nFAN
} fanID_t;

typedef enum {
  LID_HEATER_1,
  LID_HEATER_2,
  nLID_HEATER
} lidHeaterID_t;

typedef struct {
  controllerState_t state;
  int16_t           setPoint;
  uint16_t          *pwmVal;
  int16_t           *adcVal;
  int16_t           setPointLL;
  int16_t           setPointHL;
  int8_t            hysteresisActiveFlag;
} regulatorData_t;

typedef struct FAN_DATA{
  fanID_t         fanID;
  regulatorData_t regulator;
} fanData_t;

typedef struct PELTIER_DATA{
  peltierID_t     peltierID;
  regulatorData_t regulator;
} peltierData_t;

typedef struct LID_DATA{
  lidHeaterID_t   lidHeaterID;
  regulatorData_t regulator;
} lidData_t;

#ifdef USE_CL_DATA_LOGGING
typedef int16_t cl_data_t;
typedef struct CL_LOG_DATA_ELEMENT {
  u32 seqNum;
  cl_data_t cldata[CL_LOG_ELEMENT_SIZE];
} cl_logDataElement_t;

typedef struct CL_LOG_DATA_QUEUE {
  s16 head; 
  s16 tail;
  cl_logDataElement_t cl_logDataElement[CL_LOG_QUEUE_SIZE];
} cl_logDataQueue_t;

typedef struct CL_DATA_LOG_T {
  u32 sequence;  // Running sequence number
  s16 avgCnt;    // Nof samples in sum - when avgCnt==10 the avg. is added to the log
  s32 accum[CL_LOG_ELEMENT_SIZE];
} cl_dataLog_t;
#endif
/* ---------------------------------------------------------------------------*/
// command queue
xQueueHandle CoolAndLidQueueHandle;
extern xQueueHandle TubeSequencerQueueHandle;
bool msgSent = FALSE;

// Parameters for ADC
static int16_t adcCh[4] = {0, 0, 0, 0};

// Parameters for PWM
static uint16_t pwmCh[5] = {0, 0, 0, 0, 0};

static peltierData_t peltierData[1] = {
  {PELTIER_1, {STOP_STATE, -26213, &pwmCh[0], &adcCh[0]}}
};

static lidData_t lidData[1] = {
  {LID_HEATER_1, {STOP_STATE, -26213, &pwmCh[3], &adcCh[1]}}
};

static fanData_t fanData[1] = {
  {FAN_1, {STOP_STATE, 0, &pwmCh[4], &adcCh[3]}}
};

#ifdef USE_CL_DATA_LOGGING
static cl_dataLog_t cl_dataLog = {0,0,{0,0,0,0}};
static cl_logDataQueue_t cl_logDataQueue = {0,0,{{0,{0,0,0,0}},{0,{0,0,0,0}},{0,{0,0,0,0}},{0,{0,0,0,0}}}}; 
#endif

/* ---------------------------------------------------------------------------*/
/* Private prototypes                                                         */
/* ---------------------------------------------------------------------------*/
cl_logDataElement_t * cl_enqueue(cl_logDataQueue_t * pQueue);
cl_logDataElement_t * cl_dequeue(cl_logDataQueue_t * pQueue);

/* ---------------------------------------------------------------------------*/
/* Private functions                                                          */
/* ---------------------------------------------------------------------------*/
void standAlone() //These settings should be made from the Linux Box
{

  *peltierData[0].regulator.pwmVal = 8000;
  *fanData[0].regulator.pwmVal = 20000; //40% of 32767

  *lidData[0].regulator.pwmVal = 25000;
  *lidData[1].regulator.pwmVal = 0;
  pwmCh[3] = 12000; ///###JRJ DEBUG 10% on Aux
}


/* ---------------------------------------------------------------------------*/
/* Fan handling */
/* ---------------------------------------------------------------------------*/
void fan(fanData_t *fanData){
  regulatorData_t *reg;
  reg = &fanData->regulator;
  reg->setPoint = -13528; //40oC
  int64_t out = 0;
  int16_t Kp = -10;

  switch (reg->state) {
    case STOP_STATE:
    {
      *reg->pwmVal = 0;
      reg->state = CTRL_CLOSED_LOOP_STATE; //Starts when power on
    }
    break;
    case CTRL_CLOSED_LOOP_STATE:
    {
    	out = Kp*(reg->setPoint - *reg->adcVal);
    }
    break;
    default:
    break;

  }

	if (out > 32767)
		out = 32767;
	if (out < 15000)
		out = 15000;
	*reg->pwmVal = out;
}


/* ---------------------------------------------------------------------------*/
/* Peltier handling */
/* ---------------------------------------------------------------------------*/
void peltier(peltierData_t *peltierData){
  regulatorData_t *reg;
  reg = &peltierData->regulator;
  reg->setPoint = -26811; //4oC
  int64_t out = 0;
  int16_t Kp = -20;

  switch (reg->state) {
    case STOP_STATE:
    {
      *reg->pwmVal = 0;
      reg->state = CTRL_CLOSED_LOOP_STATE; //Starts when power on
    }
    break;
    case CTRL_CLOSED_LOOP_STATE:
    {
    	out = Kp*(reg->setPoint - *reg->adcVal);
    }
    break;
    default:
    break;
  }
	if (out > 25000)
		out = 25000;
	if (out < 0)
		out = 0;
	*reg->pwmVal = out;
}

/* ---------------------------------------------------------------------------*/
/* Lid handling */
/* ---------------------------------------------------------------------------*/
void lid(lidData_t *lidData)
{
  regulatorData_t *reg;
  reg = &lidData->regulator;
  reg->setPoint = 14620; //145oC
  int64_t out = 0;
  int16_t Kp = 120;


  switch (reg->state) {
    case STOP_STATE:
    {
      msgSent = FALSE;
      *reg->pwmVal = 0;
      reg->state = CTRL_OPEN_LOOP_STATE;
    } 
    break;
    case MANUAL_STATE:
    {
    	out = reg->setPoint;
      msgSent = FALSE;
    } 
    break;
    case CTRL_OPEN_LOOP_STATE:
    {
    	out = 32767;

    	if (*reg->adcVal > (reg->setPoint-2000)) //ca -10oC
    		reg->state = CTRL_CLOSED_LOOP_STATE;
    }
    break;
    case CTRL_CLOSED_LOOP_STATE:
    {
    	out = Kp*(reg->setPoint - *reg->adcVal);

    	if (*reg->adcVal < reg->setPoint-2500)
    		reg->state = CTRL_OPEN_LOOP_STATE;
    }
    break;
    default:
    break;
  }
	if (out > 32767)
		out = 32767;
	if (out < 0)
		out = 0;
	*reg->pwmVal = out;
}

/* ---------------------------------------------------------------------------*/
/* Fan handling */
/* ---------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/* Log handling */
/* ---------------------------------------------------------------------------*/
#ifdef USE_CL_DATA_LOGGING
/* ---------------------------------------------------------------------------*/
// Enqueue elament. Return pointer to element so it can be written.
cl_logDataElement_t * cl_enqueue(cl_logDataQueue_t * pQueue)
{
#ifdef DEBUG
  int h, t;
#endif
  cl_logDataElement_t * pElement;

  taskENTER_CRITICAL(); //push irq state
  if((pQueue->tail - CL_LOG_QUEUE_SIZE) == pQueue->head) { pElement = NULL; } // Return null if queue is full
  else {
    pQueue->tail++;
    pElement = &pQueue->cl_logDataElement[pQueue->tail % CL_LOG_QUEUE_SIZE];
  }
#ifdef DEBUG
  h = pQueue->head; 
  t = pQueue->tail;
#endif
  taskEXIT_CRITICAL();
#ifdef DEBUG
  t = t; h = h;
  //DEBUG_PRINTF("Enqueue: head %d tail %d", h, t);
#endif
  return pElement;
}

/* ---------------------------------------------------------------------------*/
// Dequeue elament. Return pointer to element so it can be read.
cl_logDataElement_t * cl_dequeue(cl_logDataQueue_t * pQueue)
{
#ifdef DEBUG
  int h, t;
#endif
  cl_logDataElement_t * pElement;

  taskENTER_CRITICAL(); //push irq state
  if(pQueue->tail == pQueue->head) {pElement = NULL; } // Return null if queue is empty
  else {
    pQueue->head++;
    pElement = &pQueue->cl_logDataElement[pQueue->head % CL_LOG_QUEUE_SIZE];
  }
#ifdef DEBUG
  h = pQueue->head; 
  t = pQueue->tail;
  t = t; h = h;
#endif
  taskEXIT_CRITICAL();
  //DEBUG_PRINTF("Dequeue: head %d tail %d", h, t);
  return pElement;
}

/* ---------------------------------------------------------------------------*/
void cl_dataQueueAdd(u32 seqNumber, cl_data_t data[])
{
  cl_logDataElement_t * poutData;
  
  poutData = cl_enqueue(&cl_logDataQueue);
  if(NULL != poutData)
  {
    //DEBUG_PRINTF("dataQueueAdd @0x%08X %04x %04x %04x %04x", (unsigned int)poutData, data[0], data[1], data[2], data[3]);
    poutData->seqNum = seqNumber;
    poutData->cldata[0] = data[0];
    poutData->cldata[1] = data[1];
    poutData->cldata[2] = data[2];
    poutData->cldata[3] = data[3];
  }
  else
  {
    //DEBUG_PRINTF("DataQueueAdd - buffer full");
  }
}

/* ---------------------------------------------------------------------------*/
void logInit()
{
  cl_dataLog.sequence = 0;  // Running sequence number
  cl_dataLog.avgCnt   = 0;  // Nof samples in sum - when avgCnt==10 the avg. is added to the log
  // Accumulated value for averaging over CL_SAMPLES_PER_LOG samples (avgCnt)
  cl_dataLog.accum[0] = cl_dataLog.accum[1] = cl_dataLog.accum[2] = cl_dataLog.accum[3] = 0;
  //cl_logDataQueue.cl_logDataElement[0].seqNum = 0; // 4 elements
  //cl_logDataQueue.cl_logDataElement[0].cldata[0] = 0; // 4 data per element
}

/* ---------------------------------------------------------------------------*/
void logUpdate(int16_t * ch0value, int16_t * ch1value, int16_t * ch2value, int16_t * ch3value)
{
  cl_dataLog.avgCnt += 1;
  cl_dataLog.accum[0] += *ch0value;
  cl_dataLog.accum[1] += *ch1value;
  cl_dataLog.accum[2] += *ch2value;
  cl_dataLog.accum[3] += *ch3value;

  // Is the log buffer full?
  if(CL_SAMPLES_PER_LOG <= cl_dataLog.avgCnt) {
    cl_data_t data[4];
    cl_dataLog.avgCnt = 0;
    //Do avarage of the values
    data[0] = cl_dataLog.accum[0]/CL_SAMPLES_PER_LOG;
    data[1] = cl_dataLog.accum[1]/CL_SAMPLES_PER_LOG;
    data[2] = cl_dataLog.accum[2]/CL_SAMPLES_PER_LOG;
    data[3] = cl_dataLog.accum[3]/CL_SAMPLES_PER_LOG;
    cl_dataLog.accum[0] = cl_dataLog.accum[1] = cl_dataLog.accum[2] = cl_dataLog.accum[3] = 0;
    //put in queue
    //DEBUG_PRINTF("Li: %ld %04x,%04x,%04x,%04x", cl_dataLog.sequence, data[0], data[1], data[2], data[3]);
    cl_dataQueueAdd(cl_dataLog.sequence, data);
    cl_dataLog.sequence += 1;  // Running sequence number
  }
}

/* ---------------------------------------------------------------------------*/
/* Write log data to Linux box (Called from gdi, thus running in gdi context) */
/* <uid>,<seq_number=s;log={t1,t2,t3,t4[,t1,t2,t3,t4 [,t1,t2,t3,t4[,t1,t2,t3,t4]]]}> */
/* or <uid>,OK\r  */

int getClLog(char *poutText )
{
  int i = 0;
  int nElements = 0;
  cl_logDataElement_t * pinData;
  char str[20];
  int dataAdded = 0;
  
  *poutText = 0;
  // Send all available log elements for each tube
  while(NULL != (pinData = cl_dequeue(&cl_logDataQueue)) )
  {
    //DEBUG_PRINTF("Lo: %ld %04x,%04x,%04x,%04x", pinData->seqNum, pinData->cldata[0], pinData->cldata[1], pinData->cldata[2], pinData->cldata[3]);
    dataAdded = 1;
    if(nElements == 0)
    { // Before payload
      strcat(poutText,"<seq_number=");
      Itoa(pinData->seqNum, str);
      strcat(poutText,str);
      strcat(poutText,";log={");
    }
    else
    { // Just next batch of payload data
      strcat(poutText, ",");       
    }
    for(i=0; i<CL_LOG_ELEMENT_SIZE; i++)
    { // Add payload
      //int templen=0;
      Itoa(pinData->cldata[i], str);
      //templen = strlen(str);
      strcat(poutText,str);
      if(CL_LOG_ELEMENT_SIZE - 1 > i)
      { 
        strcat(poutText, ","); 
      }
    }
    nElements+=CL_LOG_ELEMENT_SIZE;
  }
  
  if(dataAdded) { 
    poutText[strlen(poutText)]=0;
    strcat(poutText, "}>");
  } 
  //DEBUG_PRINTF("Lenght of log %d",strlen(poutText));
  return nElements;
}

#endif //USE_CL_DATA_LOGGING

/* ---------------------------------------------------------------------------*/
/* Task main loop                                                             */
/* ---------------------------------------------------------------------------*/
void CoolAndLidTask( void * pvParameters )
{
  xSemaphoreHandle xADSSemaphore = NULL;

  xMessage *msg;

#ifdef DEBUG_COOL
  int8_t cnt = 0;
#endif

  /* Create ADC synchrinization semaphore and let the ADC ISR know about it */
  vSemaphoreCreateBinary(xADSSemaphore);
  assert_param(NULL != xADSSemaphore);
  xSemaphoreTake(xADSSemaphore, portMAX_DELAY); //Default is taken. ISR will give.

  adsSetIsrSemaphore(xADSSemaphore);
  /* Start convertion and let timeout handle subsequent calls to adsContiniueSequence */

  logInit();

#if 1
  if(0 == ads1148Init())
  {
    PRINTF("ADS1148 OK\r\n");

    adsStartSeq();
    adsIrqEnable();
  }
  else
  {
    // #### Fatal error handling    
    PRINTF("ADS1148 NOT OK\r\n");
    configASSERT(pdFALSE);
  }
#endif

  adsConfigConversionTimer(&adsTimerCallback);

#ifdef STANDALONE
  standAlone();
#endif

  while(1)
  {
  #ifdef DEBUG_COOL
      if (cnt == 50)
      {
        DEBUG_PRINTF("PELC:%ld,%d,PELH:%ld,LID1:%ld,%d,ST:%d,LID2:%ld,%d,ST:%d", dac_2_temp(adcCh[0]), pwmCh[0], dac_2_temp(adcCh[3]), dac_2_temp(adcCh[1]), pwmCh[1],lidData[0].regulator.state, dac_2_temp(adcCh[2]), pwmCh[2],lidData[1].regulator.state);
        cnt = 0;
      }
      cnt++;
  #endif
    /* The control task is synchronized to the ADC interrupt by semaphore        */
    /* The ADC is startet by a timer that determines the sampling frequency      */
    /* wait indefinitely for the semaphore to become free i.e. the ISR frees it. */
    /* This also means the frequency is controlled by the ADC */
    xSemaphoreTake(xADSSemaphore, portMAX_DELAY);
    /* The semaphore is given when the ADC is done */
    /* Read lastest ADC samples into buffer */
    adsGetLatest(&adcCh[0], &adcCh[1], &adcCh[2], &adcCh[3]);
#ifndef STANDALONE
    peltier(&peltierData[0]);
    fan(&fanData[0]);
    lid(&lidData[0]);
    //lid(&lidData[1]);
#endif

    PWM_Set(pwmCh[0], PeltierCtrl1PWM);
    PWM_Set(pwmCh[1], TopHeaterCtrl1PWM);
    PWM_Set(pwmCh[2], TopHeaterCtrl2PWM);
    PWM_Set(pwmCh[3], AuxCtrlPWM);
    PWM_Set(pwmCh[4], FANctrlPWM);

    /* Add to log */
    logUpdate(&adcCh[0], &adcCh[1], &adcCh[2], &adcCh[3]);

    /* Handle incomming messages if any */
    if( xQueueReceive( CoolAndLidQueueHandle, &msg, /*Do not block*/ 0) == pdPASS )
    {
      switch(msg->ucMessageID)
      {
        case SET_FAN_SPEED:
        {
          long p;
          p = *((uint16_t *)(msg->ucData));
          *fanData[0].regulator.pwmVal = p * 32768/100;
        }
        break;
        case SET_COOL_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          peltierData[0].regulator.setPoint = temp_2_dac(p->value);
        }
        break;
        case SET_LID_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          lidData[p->idx-1].regulator.setPoint = temp_2_dac(p->value);
          lidData[0].regulator.state = CTRL_CLOSED_LOOP_STATE;
          lidData[1].regulator.state = CTRL_CLOSED_LOOP_STATE;
        }
        break;
        case SET_LID_PWM:
        {
        SetCooleAndLidReq *p;
        p=(SetCooleAndLidReq *)(msg->ucData);
        *(lidData[p->idx-1].regulator.pwmVal) = (p->value);
        lidData[p->idx-1].regulator.state = MANUAL_STATE;
        }
        break;
        case START_LID_HEATING:
        {
          lidData[0].regulator.state = CTRL_CLOSED_LOOP_STATE;
          lidData[1].regulator.state = CTRL_CLOSED_LOOP_STATE;
        }
        break;
        case STOP_LID_HEATING:
        {
          lidData[0].regulator.state = STOP_STATE;
          lidData[1].regulator.state = STOP_STATE;
        }
        break;
        case SET_LID_LOCK:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          if(1 == p->value) { GPIO_SetBits(GPIOA, LOCK_OUTPUT);   }
          if(0 == p->value) { GPIO_ResetBits(GPIOA, LOCK_OUTPUT); }
        }
        break;
        case SET_COOL_AND_LID:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          switch(p->idx) {
            case 0:
              peltierData[0].regulator.setPoint = temp_2_dac(p->value);
              break;
            case 1:
              lidData[0].regulator.setPoint = temp_2_dac(p->value);
              break;
            case 2:
              //lidData[1].regulator.setPoint = temp_2_dac(p->value);
              break;
            case 3: //Aux PWM connecter
              pwmCh[3] = p->value * 32768/100;
              PWM_Set(pwmCh[3], AuxCtrlPWM);
              break;
            case 4: //Fan ctrl
              pwmCh[4] = p->value * 32768/100;
              PWM_Set(pwmCh[4], FANctrlPWM);
              break;
            case 5:
              if(1 == p->value) { GPIO_SetBits(GPIOA, LOCK_OUTPUT);   }
              if(0 == p->value) { GPIO_ResetBits(GPIOA, LOCK_OUTPUT); }
              break;
            default:
              break;
          }
        }
        break;
        default:
        break; //ignore message
      }      
      vPortFree(msg);
    }
      
  }
  // We are not supposed to end, but if so kill this task.
  vTaskDelete(NULL);
}

