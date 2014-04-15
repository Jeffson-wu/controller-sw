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

/*-----------------------------------------------------------*/
#define LOCK_OUTPUT GPIO_Pin_15  //GPIO_Pin_8

//#define DEBUG

typedef enum {
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
  LID_HEATER_1,
  LID_HEATER_2,
  nLID_HEATER
} lidHeaterID_t;

typedef struct {
  uint16_t          *pwmVal;
  int16_t           *adcVal;
  controllerState_t  state;
  int16_t            pv;
  pidData_t          pid;
} regulatorData_t;

typedef struct PELTIER_DATA{
  peltierID_t     peltierID;
  int16_t         setpoint;
  regulatorData_t regulator;
} peltierData_t;

typedef struct LID_DATA{
  lidHeaterID_t   lidHeaterID;
  int16_t         setpoint;
  regulatorData_t regulator;
} lidData_t;

/*-----------------------------------------------------------*/
// command queue
xQueueHandle CooleAndLidQueueHandle;

// Parameters for ADC
static int16_t adcCh[4] = {0, 0, 0, 0};

// Parameters for PWM
static uint16_t pwmCh[5] = {0, 0, 0, 0, 0};

static peltierData_t peltierData[1] = {
  {PELTIER_1,    /*setpoint*/0, {&pwmCh[0], &adcCh[0], CTRL_OPEN_LOOP_STATE}}
};

static lidData_t lidData[2] = {
  {LID_HEATER_1, /*setpoint*/0, {&pwmCh[1], &adcCh[1], CTRL_OPEN_LOOP_STATE}},
  {LID_HEATER_2, /*setpoint*/0, {&pwmCh[2], &adcCh[2], CTRL_OPEN_LOOP_STATE}}
};


/* ---------------------------------------------------------------------------*/
/* Peltier handling */
/* ---------------------------------------------------------------------------*/
void initPeltier(peltierData_t *ppeltierData, int16_t kp, int16_t ki, int16_t kd)
{
  pid_Init(kp, ki , kd, &ppeltierData->regulator.pid);
  pid_SetSetPoint(ppeltierData->setpoint, &ppeltierData->regulator.pid);
}

peltier(peltierData_t *peltierData){
  int r;
  int16_t ov;
  regulatorData_t *reg;

  reg = &peltierData->regulator;

  reg->pv = *reg->adcVal;
  switch (reg->state) {
    case CTRL_OPEN_LOOP_STATE:
    {
      int16_t error = (reg->pid.setPoint - reg->pv);
      if (error < -1000) {
        *reg->pwmVal = 32767;
      } else {
        reg->state = CTRL_CLOSED_LOOP_STATE;
      }
    }
    break;
    case CTRL_CLOSED_LOOP_STATE:
    {
      ov = pid_Controller(reg->pv, &peltierData->regulator.pid);
      *reg->pwmVal = (ov/2)-16384;
    }
    break;
  }
}

/* ---------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/* Lid handling */
/* ---------------------------------------------------------------------------*/
void initLid(lidData_t *plidData, int16_t kp, int16_t ki, int16_t kd)
{
  pid_Init(kp, ki , kd, &plidData->regulator.pid);
  pid_SetSetPoint(plidData->setpoint, &plidData->regulator.pid);
}

lid(lidData_t *lidData)
{
  int16_t ov;

  regulatorData_t *reg;
  reg = &lidData->regulator;
  
  reg->pv = *reg->adcVal;
  switch (reg->state) {
    case CTRL_OPEN_LOOP_STATE:
    {
      int16_t error = (reg->pid.setPoint - reg->pv);
      if (error > 1000) {
        *reg->pwmVal = 32767;
      } else {
        reg->state = CTRL_CLOSED_LOOP_STATE;
      }
    } 
    break;
    case CTRL_CLOSED_LOOP_STATE:
    {
      ov = pid_Controller(reg->pv, &lidData->regulator.pid);
      *reg->pwmVal = (ov/2)-16384;
    }
    break;
  }

}
/* ---------------------------------------------------------------------------*/
/* Fan handling */
/* ---------------------------------------------------------------------------*/

/*-----------------------------------------------------------*/
void CooleAndLidTask( void * pvParameters )
{
  xSemaphoreHandle xADSSemaphore = NULL;
  short usData;
  xMessage *msg;
  int i; //iterator
#ifdef DEBUG
  int count = 0;
  char str[20];
#endif

  /* Create ADC synchrinization semaphore and let the ADC ISR know about it */
  vSemaphoreCreateBinary(xADSSemaphore);
  assert_param(NULL != xADSSemaphore);
  xSemaphoreTake(xADSSemaphore, portMAX_DELAY); //Default is taken. ISR will give.

  adsSetIsrSemaphore(xADSSemaphore);
  /* Start convertion and let timeout handle subsequent calls to adsContiniueSequence */
  if(0 == ads1148Init())
  {
#ifdef DEBUG
    gdi_send_msg_response("ADS1148 OK\r\n");
#endif
    adsStartSeq();
    adsIrqEnable();
  }
  else
  {
    // #### Fatal error handling
  }
  adsConfigConversionTimer(&adsTimerCallback);
  initPeltier(&peltierData[0], K_P*SCALING_FACTOR, K_I*SCALING_FACTOR, K_D*SCALING_FACTOR); 
  initLid(&lidData[0], K_P*SCALING_FACTOR, K_I*SCALING_FACTOR, K_D*SCALING_FACTOR);
  initLid(&lidData[1], K_P*SCALING_FACTOR, K_I*SCALING_FACTOR, K_D*SCALING_FACTOR);

  while(1)
  {

    /* The control task is synchronized to the ADC interrupt by semaphore */
    /* wait indefinitely for the semaphore to become free i.e. the ISR frees it. */
    /* This also means the frequency is controlled by the ADC */
    xSemaphoreTake(xADSSemaphore, portMAX_DELAY);
    /* The semaphore is given when the ADC is done */
    /* Read lastest ADC samples into buffer */
    adsGetLatest(&adcCh[0], &adcCh[1], &adcCh[2], &adcCh[3]);

    peltier(&peltierData[i]);
    for(i = 0; i < 2; i ++)
    {
      lid(&lidData[i]);
    }

#ifdef DEBUG
    /* Debugging the feedback value */
    {
      sprintf(str, "c: %d" /* "I=%d, O=%d", adcCh[0], pwmCh[0]*/ , count++);
      gdi_send_msg_response(str);
    }
#endif

    // Fan ctrl ??
    
    PWM_Set(pwmCh[0], PeltierCtrlPWM1);
    PWM_Set(pwmCh[1], PeltierCtrlPWM2);
    PWM_Set(pwmCh[2], PeltierCtrlPWM3);
    PWM_Set(pwmCh[3], TopHeaterCtrlPWM);
    PWM_Set(pwmCh[4], FANctrlPWM);
    
    if( xQueueReceive( CooleAndLidQueueHandle, &msg, /*Do not block*/ 0) == pdPASS )
    {
      switch(msg->ucMessageID)
      {
        case SET_FAN:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          pwmCh[4] = p->value * 32768/100;
        }
        break;
        case SET_COOLE_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          peltierData[0].setpoint = temp_2_dac(p->value);
        }
        break;
        case SET_LID_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          lidData[0].setpoint = lidData[1].setpoint = temp_2_dac(p->value);
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
              peltierData[0].setpoint = temp_2_dac(p->value);
              break;
            case 1:
              lidData[0].setpoint = temp_2_dac(p->value);
              break;
            case 2:
              lidData[1].setpoint =temp_2_dac(p->value);
              break;
            case 3: //In old lid heater connecter
              pwmCh[3] = p->value;
              break;
            case 4: //Fan ctrl
              pwmCh[4] = p->value;
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


