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

/*-----------------------------------------------------------*/
#define DEBUG /*General debug shows state changes of tubes (new temp, new time etc.)*/
#define DEBUG_COOL
#ifdef DEBUG
char buf[20];
#define DEBUG_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif

//#define STANDALONE /*Defines if the M3 Runs with or without Linux box*/

#define LOCK_OUTPUT GPIO_Pin_8

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

/*-----------------------------------------------------------*/
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
  reg->setPoint = -10289; //50oC
  int64_t out = 0;
  int16_t Kp = -6;

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
  reg->setPoint = -21705; //10oC
  reg->setPointLL = reg->setPoint - 200;
  reg->setPointHL = reg->setPoint + 200;
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
	if (out > 20000)
		out = 20000;
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
  reg->setPoint = 5597; //100oC
  reg->setPointLL = reg->setPoint - 200;
  reg->setPointHL = reg->setPoint + 200;
  int64_t out = 0;
  int16_t Kp = 120;

  switch (reg->state) {
    case STOP_STATE:
    {
      msgSent = FALSE;
      *reg->pwmVal = 0;
      reg->hysteresisActiveFlag = 0;
      reg->state = CTRL_OPEN_LOOP_STATE;
    } 
    break;
    case MANUAL_STATE:
    {
      msgSent = FALSE;
    } 
    break;
    case CTRL_OPEN_LOOP_STATE:
    {
    	out = 32767;

    	if (*reg->adcVal > 3927) //95oC
    		reg->state = CTRL_CLOSED_LOOP_STATE;
    }
    break;
    case CTRL_CLOSED_LOOP_STATE:
    {
    	out = Kp*(reg->setPoint - *reg->adcVal);

    	if (*reg->adcVal < 3264) //93oC
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
void CooleAndLidTask( void * pvParameters )
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

#if 1
  if(0 == ads1148Init())
  {
    DEBUG_PRINTF("ADS1148 OK\r\n");

    adsStartSeq();
    adsIrqEnable();
  }
  else
  {
    // #### Fatal error handling
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
        DEBUG_PRINTF("PELC:%ld,%d,PELH:%ld,LID1:%ld,%d,ST:%dLID2:%ld,%d,ST:%d", dac_2_temp(adcCh[0]), pwmCh[0], dac_2_temp(adcCh[3]), dac_2_temp(adcCh[1]), pwmCh[1],lidData[0].regulator.state, dac_2_temp(adcCh[2]), pwmCh[2],lidData[1].regulator.state);
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
        case SET_COOLE_AND_LID:
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
