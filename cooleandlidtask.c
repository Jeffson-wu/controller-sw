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
  **/

/* Private feature defines ---------------------------------------------------*/
#define USE_M3_ADC
//#define USE_ADS1148 : discontiniued, now using internal ADC
#define USE_CL_DATA_LOGGING
#define USE_ANALOG_WATCH_DOG
#define USE_TWO_LEVEL_LID_POWER
#define DISABLE_ERROR_REPOTING

/* Private debug define ------------------------------------------------------*/
//#define DEBUG /*General debug shows state changes of tubes (new temp, new time etc.)*/
//#define DEBUG_COOL
//#define DEBUG_LOGGING
//#define STANDALONE /*Defines if the M3 Runs with or without Linux box*/

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
#include "stm32f10x_dac.h"
#ifdef USE_M3_ADC
#include "adc.h"
#else
#include "ads1148.h"
#endif
#include "pid.h"
#include "pwm.h"
#include "gdi.h"
#include "util.h"
#include "nvs.h"
#include "../heater-sw/heater_reg.h"
#include "cooleandlidtask.h"
/* ---------------------------------------------------------------------------*/
char buf[20];
#define PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif
#ifdef DEBUG_LOGGING
#define DEBUG_LOGGING_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_LOGGING_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif

#define LOCK_OUTPUT_PIN   GPIO_Pin_8
#define LOCK_OUTPUT_PORT  GPIOA
#define LID_DETECT_PIN    GPIO_Pin_5
#define LID_DETECT_PORT   GPIOB
#define PELTIER_EN_PIN    GPIO_Pin_6
#define PELTIER_EN_PORT   GPIOC
#define PELTIER_DAC_PIN   GPIO_Pin_4
#define PELTIER_DAC_PORT  GPIOA
#define DAC_UPPER_LIMIT   3725

#ifdef USE_CL_DATA_LOGGING
  #define CL_SAMPLES_PER_LOG  10    //Each log is the avarage over this number of samples
  #define CL_LOG_ELEMENT_SIZE 4     //Log all four sensors
  #define CL_LOG_QUEUE_SIZE   4     //Queue length
#endif
#define Swap2Bytes(val) ( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) )

/* Private typedef -----------------------------------------------------------*/
const char *cl_states[] =
{
  "clnok",    /* CL temperatures not reached - Not OK to start PCR            */
  "clok",     /* CL temperatures reached     - OK to start PCR                */
};

typedef enum CL_STATES_T {
  CL_STATE_CLNOK,
  CL_STATE_CLOK,
  nCL_STATES
} cl_states_t;

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
  nPELTIER
} peltierID_t;

typedef enum {
  FAN_1,
  //FAN_2,
  nFAN
} fanID_t;

typedef enum {
  LID_HEATER_1,
  //LID_HEATER_2,
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

typedef struct CALIB_DATA_T {
  int16_t c_1;
} calib_data_t;

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

// Events to be sent to the Linux box
static uint16_t cl_status = 0;
static uint8_t clState = CL_STATE_CLNOK;
static bool coolTempOK = FALSE;
static bool lidTempOK = FALSE;

// Parameters for ADC
static int16_t adcCh[4] = {0, 0, 0, 0};

// ADC diff for Fan
static int16_t adcDiff[1] = {0};

// Parameters for PWM
static uint16_t pwmCh[6] = {0, 0, 0, 0, 0, 0};

// Parameters for DAC
static uint16_t dacCh[1] = {0};

/*                      GPIO                         On PCB Rev2      On PCB Rev3
 * pwmCh[0], TIM4,CH3 - PB8 -  J175 : PWM0_TIM4CH3 - TopHeater1Ctrl   TopHeater1Ctrl
 * pwmCh[1], TIM4,CH4 - PB9 -  J26  : PWM1_TIM4CH4 - FAN control      TopHeater2Ctrl
 * pwmCh[2], TIM3,CH1 - PC6 -  J33  : PWM2_TIM3CH1 - Peltier PWM 1    -
 * pwmCh[3], TIM3,CH2 - PC7 -  J176 : PWM3_TIM3CH2 - TopHeater2Ctrl   FAN control
 * pwmCh[4], TIM3,CH3 - PC8 -  J35  : PWM4_TIM3CH3 - AUX PWM          AUX PWM
 * pwmCh[5],                                                          dummy PWM
 * DAC_OUT1     PA4
 * DAC_OUT2     PA5
 * ADC PCB Rev2:                       
 * adcCh[0], ADC CH0 -         J177 : RTD1         - Peltier_sens1  (Cold side)
 * adcCh[1], ADC CH1 -         J173 : RTD2         - TopHeaterSens1
 * adcCh[2], ADC CH2 -         J174 : RTD3         - TopHeaterSens2 (Ambient Air)
 * adcCh[3], ADC CH3 -         J178 : RTD4         - Peltier_sens2  (Hot side)
 * ADC PCB Rev3: 
 * adcCh[0], ADC12_IN0 -       J??? :              - Peltier_sens1  (Cold side)
 * adcCh[1], ADC12_IN5 -       J??? :              - TopHeaterSens1
 * adcCh[2], ADC12_IN6 -       J??? :              - TopHeaterSens2 (Ambient Air)
 * adcCh[3], ADC12_IN7 -       J??? :              - Peltier_sens2  (Hot side)

   NTC_ADC_IN0        a PA0/ADC12_IN0
   Peltier_DAC        a PA4/DAC_OUT1 (/ADC12_IN4)
   NTC_ADC_IN5        a PA5/ADC12_IN5 (/DAC_OUT2)
   NTC_ADC_IN6        a PA6/ADC12_IN6
   NTC_ADC_IN7        a PA7/ADC12_IN7
   LockCtrlOut        d PA8/TIM1_CH1 (/MCO)

   LID_DETECT         d PB5
   I2C1_SCL           d PB6/I2C1_SCL/TIM4_CH1
   I2C1_SDA           d PB7/I2C1_SDA/TIM4_CH2
   TopHeater1CtrlPWM  p PB8/TIM4_CH3
   TopHeater2CtrlPWM  p PB9/TIM4_CH4
   TopHeaterMainPWM   p PB10/TIM2_CH3   TP?

   ADC10_VMON         a PC0/ADC12_IN10
   HW_ID              a PC1/ADC12_IN11
   PeltierCtrl_EN     d PC6/TIM3_CH1
   FANctrlPWM         p PC7/TIM3_CH2
   AuxCtrlPWM         p PC8/TIM3_CH3    TP?
   HeartBeatLED       d PC9/TIM3_CH4
 */ 

// Fan controll is based on the temp diff: adcDiff[0] =  Fin temp - Ambient temp
static int16_t *adcDiffSource[2] = {&adcCh[4], &adcCh[2]}; 

static peltierData_t peltierData[nPELTIER] = {
  {PELTIER_1, {STOP_STATE, -26213, &dacCh[0], &adcCh[3]}}
};

static lidData_t lidData[nLID_HEATER] = {
  {LID_HEATER_1, {STOP_STATE, -26213, &pwmCh[5], &adcCh[2]}}
};

static uint16_t *pwmChMirror[2] = {&pwmCh[0], &pwmCh[1]};

static fanData_t fanData[nFAN] = {
  {FAN_1, {STOP_STATE, 0, &pwmCh[3], /*&adcDiff[0]*/ &adcCh[0]}}
};

calib_data_t __attribute__ ((aligned (2))) calib_data[3] = {
  /* Default calibration data - use if no valid data found in NVS */
    { 15000   },       /* Lid     90degC */
    { -19963 },       /* Peltier  15degC */
    { -13528 }        /* Fan      40degC */
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
/* functions                                                                  */
/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
static void gpioInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = PELTIER_DAC_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(PELTIER_DAC_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LOCK_OUTPUT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(LOCK_OUTPUT_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = PELTIER_EN_PIN;
  GPIO_Init(PELTIER_EN_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LID_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(LID_DETECT_PORT, &GPIO_InitStructure);
}


/* ---------------------------------------------------------------------------*/
void stopPeltier()
{
  GPIO_ResetBits(PELTIER_EN_PORT, PELTIER_EN_PIN);
  *peltierData[0].regulator.pwmVal = 0;
  //*peltierData[1].regulator.pwmVal = 0;
  peltierData[0].regulator.state = STOP_STATE;
  //peltierData[1].regulator.state = STOP_STATE;
}

/* ---------------------------------------------------------------------------*/
void standAlone() //These settings should be made from the Linux Box
{

  *peltierData[0].regulator.pwmVal = 8000;
  *fanData[0].regulator.pwmVal = 20000; //40% of 32767

  *lidData[0].regulator.pwmVal = 25000;
  *lidData[1].regulator.pwmVal = 0;
  pwmCh[3] = 12000; ///###JRJ DEBUG 10% on Aux
  coolTempOK = TRUE;
  lidTempOK  = TRUE;
}

/* ---------------------------------------------------------------------------*/
void setCLStatusReg(uint16_t status)
{
  taskENTER_CRITICAL(); //push irq state
  cl_status |= status;
  taskEXIT_CRITICAL();
}

/* ---------------------------------------------------------------------------*/
uint16_t getCLStatusReg(void)
{
  uint16_t ret;
  taskENTER_CRITICAL();
  ret = cl_status;
  cl_status = 0;
  taskEXIT_CRITICAL();
  return ret;
}

/* ---------------------------------------------------------------------------*/
/* Read out latest ACD values                                                 */
/* This function is called from gdi and thus is executed in gdi context.      */
/* ---------------------------------------------------------------------------*/
int getAdc(char *poutText)
{
  char str[20];
  int16_t adcData[4];

  *poutText = 0;
#ifdef USE_M3_ADC
  adcGetLatest(&adcData[0], &adcData[1], &adcData[2], &adcData[3]);
#else
  adsGetLatest(&adcData[0], &adcData[1], &adcData[2], &adcData[3]);
#endif
  strcat(poutText,"adc_values={");
  Itoa(adcData[0], str);
  strcat(poutText,str);
  strcat(poutText, ","); 
  Itoa(adcData[1], str);
  strcat(poutText,str);
  strcat(poutText, ","); 
  Itoa(adcData[2], str);
  strcat(poutText,str);
  strcat(poutText, ","); 
  Itoa(adcData[3], str);
  strcat(poutText,str);
  strcat(poutText, "}");
  return 0;
}

/* ---------------------------------------------------------------------------*/
/* Fan handling */
/* ---------------------------------------------------------------------------*/
void fan(fanData_t *fanData){
  regulatorData_t *reg;
  reg = &fanData->regulator;
  reg->setPoint = 2600; //40oC
  int64_t out = 0;
  int16_t Kp = -10;

  switch (reg->state) {
    case STOP_STATE:
    {
      *reg->pwmVal = 0;
      reg->state = CTRL_CLOSED_LOOP_STATE; //Starts when power on
    }
    break;
    case MANUAL_STATE:
    {
      return; // No action in manuel state
    } 
    break;
    case CTRL_OPEN_LOOP_STATE:
    {
      reg->state = CTRL_CLOSED_LOOP_STATE;
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
    { out = 32767; }
  if (out < 12000)
    { out = 12000; }
  //*reg->pwmVal = out;
  *reg->pwmVal = 17000; // ToDo: remove hard coded value
}

/* ---------------------------------------------------------------------------*/
/* Peltier handling */
/* ---------------------------------------------------------------------------*/
// TODO: JRJ implement reset for current source at least once every 4 hours
void peltier(peltierData_t *peltierData){
  regulatorData_t *reg;
  reg = &peltierData->regulator;
  reg->setPoint = 1600; //??oC
  //reg->setPointLL = reg->setPoint - 20;
  //reg->setPointHL = reg->setPoint + 20;
  int64_t out = 0;
  int16_t Kp = -2;

  switch (reg->state) {
    case STOP_STATE:
    {
      *reg->pwmVal = 0;
      GPIO_ResetBits(PELTIER_EN_PORT, PELTIER_EN_PIN);  //Disable Peltier
      reg->state = CTRL_CLOSED_LOOP_STATE; //Starts when power on
    }
    break;
    case MANUAL_STATE:
    {
      return; // No action in manuel state
    } 
    break;
    case CTRL_OPEN_LOOP_STATE:
    {
      GPIO_SetBits(PELTIER_EN_PORT, PELTIER_EN_PIN); //Enable peltier
      reg->state = CTRL_CLOSED_LOOP_STATE;
    } 
    break;
    case CTRL_CLOSED_LOOP_STATE:
    {
      GPIO_SetBits(PELTIER_EN_PORT, PELTIER_EN_PIN); //Enable peltier
      out = Kp*(reg->setPoint - *reg->adcVal);
    }
    break;
    default:
    break;
  }
  if( abs(reg->setPoint - *reg->adcVal) < 10 ) { coolTempOK = TRUE; }
  if (out > DAC_UPPER_LIMIT)
    { out = DAC_UPPER_LIMIT; }
  if (out < 0)
    { out = 0; }
//    *reg->pwmVal = out;

  *reg->pwmVal = 1100; //ToDo: remove.


}

/* ---------------------------------------------------------------------------*/
/* Lid handling */
/* ---------------------------------------------------------------------------*/
void lid(lidData_t *lidData)
{
  regulatorData_t *reg;
  reg = &lidData->regulator;
  reg->setPoint = 2700; //???oC
  //reg->setPointLL = reg->setPoint - 200;
  //reg->setPointHL = reg->setPoint + 200;
  int64_t out = 0;
  int16_t Kp = 45; //1.5;

  switch (reg->state) {
    case STOP_STATE:
    {
      *reg->pwmVal = 0;
      reg->hysteresisActiveFlag = 0;
      //reg->state = CTRL_OPEN_LOOP_STATE;
      reg->state = CTRL_CLOSED_LOOP_STATE; // << remove
    } 
    break;
    case MANUAL_STATE:
    {
      return; // No action in manuel state
    } 
    break;
    case CTRL_OPEN_LOOP_STATE:
    {
      out = 32767;

      if (*reg->adcVal > (reg->setPoint-40)) //ca -10oC
        reg->state = CTRL_CLOSED_LOOP_STATE;
    }
    break;
    case CTRL_CLOSED_LOOP_STATE:
    {
      out = Kp*(reg->setPoint - *reg->adcVal);

      //if (*reg->adcVal < reg->setPoint-100) // << incomment
      //  reg->state = CTRL_OPEN_LOOP_STATE;
    }
    break;
    default:
    break;
  }
  if( abs(reg->setPoint - *reg->adcVal) < 10 ) { lidTempOK = TRUE; }
  if (out > 32767)
    { out = 32767; }
  if (out < 0)
    { out = 0; }
  *reg->pwmVal = out;

/*
#ifdef DEBUG_COOL
  static int8_t cnt = 0;
#endif

#ifdef DEBUG_COOL
    if (cnt == 50)
    {
      DEBUG_PRINTF("LID_Err:%d", (reg->setPoint - *reg->adcVal));
      cnt = 0;
    }
    cnt++;
#endif
*/
}

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
  DEBUG_LOGGING_PRINTF("Enqueue: head %d tail %d", h, t);
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
  DEBUG_LOGGING_PRINTF("Dequeue: head %d tail %d", h, t);
  return pElement;
}

/* ---------------------------------------------------------------------------*/
void cl_dataQueueAdd(u32 seqNumber, cl_data_t data[])
{
  cl_logDataElement_t * poutData;
  
  poutData = cl_enqueue(&cl_logDataQueue);
  if(NULL != poutData)
  {
    DEBUG_LOGGING_PRINTF("dataQueueAdd @0x%08X %04x %04x %04x %04x", (unsigned int)poutData, data[0], data[1], data[2], data[3]);
    poutData->seqNum = seqNumber;
    poutData->cldata[0] = data[0];
    poutData->cldata[1] = data[1];
    poutData->cldata[2] = data[2];
    poutData->cldata[3] = data[3];
  }
  else
  {
    DEBUG_LOGGING_PRINTF("DataQueueAdd - buffer full");
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
/* Linux Box interpretation = Ambient, top heater, could side, warm side      */
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
    DEBUG_LOGGING_PRINTF("Li: %ld %04x,%04x,%04x,%04x", cl_dataLog.sequence, data[0], data[1], data[2], data[3]);
    cl_dataQueueAdd(cl_dataLog.sequence, data);
    cl_dataLog.sequence += 1;  // Running sequence number
  }
}

/* ---------------------------------------------------------------------------*/
/* Public functions                                                           */
/* ---------------------------------------------------------------------------*/

/* Write log data to Linux box (Called from gdi, thus running in gdi context) */
/* state=<state>,seq_number=<seqNum>;log={<t0>,<t1>,<t2>,<t3>[,<t0>,<t1>,<t2>,<t3>[,<t0>,<t1>,<t2>,<t3>[,<t0>,<t1>,<t2>,<t3>]]]]} */
/* or state=<state>\r  */

int getClLog(char *poutText )
{
  int i = 0;
  int nElements = 0;
  cl_logDataElement_t * pinData;
  char str[20];
  int dataAdded = 0;
  
  *poutText = 0;

  if( coolTempOK && lidTempOK ) { clState = CL_STATE_CLOK; } else { clState = CL_STATE_CLNOK; }
  strcat(poutText,"state=");
#ifdef DISABLE_ERROR_REPOTING
  strcat(poutText,cl_states[CL_STATE_CLOK]); // Juste say everything is OK
#else
  strcat(poutText,cl_states[clState]);
#endif
  //#### strcat(poutText,",");

  // Send all available log elements
  while(NULL != (pinData = cl_dequeue(&cl_logDataQueue)) )
  {
    DEBUG_LOGGING_PRINTF("Lo: %ld %04x,%04x,%04x,%04x", pinData->seqNum, pinData->cldata[0], 
                         pinData->cldata[1], pinData->cldata[2], pinData->cldata[3]);
    dataAdded = 1;
    if(nElements == 0)
    { // Before payload
      strcat(poutText,",seq_number=");
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
    strcat(poutText, "}>"); // #### Remove '>' when KS has updated tube mand.
  } 
  nElements++; // "state=<clState>" is always added
  DEBUG_LOGGING_PRINTF("Lenght of log %d",strlen(poutText));
  return nElements;
}

#endif //USE_CL_DATA_LOGGING

/* ---------------------------------------------------------------------------*/
/* Read out HW events                                                         */
/* This function is called from gdi and thus is executed in gdi context.      */
/* ---------------------------------------------------------------------------*/
int getCoolandlidHWReport(char *poutText)
{
  uint16_t event;
  char str[5];

#ifdef USE_M3_ADC
  event = 0;  // Add similar fn for M3 ADC
#else
  event = getADSStatusReg();
#endif
  event |= getCLStatusReg();
  if(event)
  {
    strcat(poutText,",coolandlid_event=");
    Itoa(event, str);
    strcat(poutText,str);
    //strcat(poutText,">");
    return 1;
  }
  return 0;
}

/* ---------------------------------------------------------------------------*/
/* coolLidReadRegs and coolLidWriteRegs are implemented use the same com i/f  */
/* as for controlling the heaters (on the M0s)                                */
/* "addr" is first reg, "datasize" is reg count, "buffer" is wrong endian     */
/* ---------------------------------------------------------------------------*/
bool coolLidReadRegs(u8 slave, u16 addr, u16 datasize, u16 *buffer)
{
  int reg;
  u16 val;
  for (reg = 0; reg < datasize; reg++)
  {
    switch((heater_regs_t)(addr + reg))
    {
      case SETPOINT_REG:
        switch(slave)
        {
          case LID_ADDR:
            val = lidData[0].regulator.setPoint;
            break;
          case PELTIER_ADDR:
            val = peltierData[0].regulator.setPoint;
            break;
          case FAN_ADDR:
            val = fanData[0].regulator.setPoint;
            break;
          default:
            break;
        }
      default:
        break;
    }
    *(buffer + reg) = Swap2Bytes(val); // correct endianess
  }
  return FALSE;
}

/* ---------------------------------------------------------------------------*/
/* "addr" is first reg, "datasize" is reg count, "data" is wrong endian       */
bool coolLidWriteRegs(u8 slave, u16 addr, u16 *data, u16 datasize)
{
  int reg;
  u16 val;
  for (reg = 0; reg < datasize; reg++)
  {
    val = Swap2Bytes(*(data + reg)); // correct endianess
    switch((heater_regs_t)(addr + reg))
    {
      case SETPOINT_REG:
        switch(slave)
        {
          case LID_ADDR:
            lidData[0].regulator.setPoint = val;
            lidTempOK = FALSE;
            break;
          case PELTIER_ADDR:
            peltierData[0].regulator.setPoint = val;
            coolTempOK = FALSE;
            break;
          case FAN_ADDR:
            fanData[0].regulator.setPoint = val;
            break;
          default:
            break;
        }
        break;
      case TUBE_COMMAND_REG:
        /* Here the slave is irrelevant */
        if (WRITE_CAL_DATA == val) {
          calib_data[0].c_1 = lidData[0].regulator.setPoint;
          calib_data[1].c_1 = peltierData[0].regulator.setPoint;
          calib_data[2].c_1 = fanData[0].regulator.setPoint;
          NVSwrite(sizeof(calib_data), calib_data);
          DEBUG_PRINTF("\r\nWrote calib.\r\n");
        }
        else if(SET_IDLE_MODE == val)
        {
          xMessage *msgout;
          msgout = pvPortMalloc(sizeof(xMessage));
          if(msgout)
          {
            msgout->ucMessageID = DISABLE_NEIGHBOUR_TUBE_TEMP;
            xQueueSend(TubeSequencerQueueHandle, &msgout, portMAX_DELAY);
          }
        }
        else if(SET_AUTOMATIC_MODE == val)
        {
          xMessage *msgout;
          msgout = pvPortMalloc(sizeof(xMessage));
          if(msgout)
          {
            msgout->ucMessageID = ENABLE_NEIGHBOUR_TUBE_TEMP;
            xQueueSend(TubeSequencerQueueHandle, &msgout, portMAX_DELAY);
          }
        }
        break;
      default:
        break;
    }
  }
  return FALSE;
}

/* ---------------------------------------------------------------------------*/
/* Task main loop                                                             */
/* ---------------------------------------------------------------------------*/
void CoolAndLidTask( void * pvParameters )
{
  xSemaphoreHandle xADCSemaphore = NULL;

  xMessage *msg;

#ifdef DEBUG_COOL
  int8_t cnt = 0;
#endif

  gpioInit();
  /* Create ADC synchrinization semaphore and let the ADC ISR know about it */
  vSemaphoreCreateBinary(xADCSemaphore);
  vQueueAddToRegistry(xADCSemaphore,(char *)"ADS sem");
  assert_param(NULL != xADCSemaphore);
  xSemaphoreTake(xADCSemaphore, portMAX_DELAY); //Default is taken. ISR will give.
  logInit();
#ifdef USE_M3_ADC
  // adcInit() is called from main() to obtain HW REV ID first thing. 
  awdInit(7 /*chToWatch*/);
  adcSetIsrSemaphore(xADCSemaphore);
  vQueueAddToRegistry(xADCSemaphore,(char *)"ADC sem");
  adcConfigConversionTimer(&adcTimerCallback);
  adcStartSeq();
  DEBUG_PRINTF("ADC Initialized\r\n");
#else
  adsSetIsrSemaphore(xADCSemaphore);
  vTaskDelay(1000); /* Wait for ADC to be ready */
  adsConfigConversionTimer(&adsTimerCallback);
  if(0 == ads1148Init())
  {
    PRINTF("ADS1148 OK\r\n");
    /* Start convertion and let timeout handle subsequent calls to adsContiniueSequence */
    adsStartSeq();
    adsIrqEnable();
  }
  else
  {// #### Fatal error handling    
    PRINTF("ADS1148 NOT OK\r\n");
    configASSERT(pdFALSE);
  }
#endif
  /* --> Init DAC */
  DAC_InitTypeDef DAC_InitStruct;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = 0;
  DAC_InitStruct.DAC_OutputBuffer   = DAC_OutputBuffer_Enable;
  DAC_InitStruct.DAC_Trigger        = DAC_Trigger_None;
  DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_DeInit();
  DAC_Init(DAC_Channel_1, &DAC_InitStruct);
  DAC_Cmd( DAC_Channel_1, ENABLE);
  /* <-- Init DAC */

// use "setCLStatusReg(HW_DEFAULT_CAL_USED)" if default calib is used
#ifdef STANDALONE
  standAlone();
  setCLStatusReg(0xf00f); //Debug####JRJ
  DEBUG_PRINTF("CL stand alone\r\n");
#endif
  /* Read calibration data form NVS */
  if(0 != NVSread(sizeof(calib_data), calib_data) ) { 
    PRINTF("\r\nUsing default calib:\r\n");    // How to do this on the M3?? - setHWStatusReg(HW_DEFAULT_CAL_USED);
  } else {
    PRINTF("\r\nUsing stored calib:\r\n");
  }
  lidData[0].regulator.setPoint     = calib_data[0].c_1;
  peltierData[0].regulator.setPoint = calib_data[1].c_1;
  fanData[0].regulator.setPoint     = calib_data[2].c_1;

  while(1)
  {
  #ifdef DEBUG_COOL
    if (cnt == 40)
    {
      static int toggle = 0;
      DEBUG_PRINTF("PEL_PWM:%d, PEL:%ld, FAN_PWM:%d, FAN:%ld, LID_PWM:%d, LID:%ld", pwmCh[0], dac_2_temp(adcCh[0]), pwmCh[3], dac_2_temp(adcCh[3]), pwmCh[4], dac_2_temp(adcCh[1]));
      //DEBUG_PRINTF("Adc:%4d, %4d, %4d, %4d", adcCh[0], adcCh[1], adcCh[2], adcCh[3]);
      cnt = 0;
      if(toggle) {
        DEBUG_PRINTF("vmon: %d", readADC(ADC_VMON_MUX_CH));
        toggle = 0;
      } else {
        DEBUG_PRINTF("hw_id: %d", readADC(ADC_HW_REV_ID_MUX_CH));
        toggle = 1;
      }
    }
    cnt++;
  #endif
    /* The control task is synchronized to the ADC interrupt by semaphore        */
    /* The ADC is startet by a timer that determines the sampling frequency      */
    /* wait indefinitely for the semaphore to become free i.e. the ISR frees it. */
    /* This also means the frequency is controlled by the ADC */
    xSemaphoreTake(xADCSemaphore, portMAX_DELAY);
    /* The semaphore is given when the ADC is done */
    /* Read lastest ADC samples into buffer */
#ifdef USE_M3_ADC
    adcGetLatest(&adcCh[0], &adcCh[1], &adcCh[2], &adcCh[3]);
#else
    adsGetLatest(&adcCh[0], &adcCh[1], &adcCh[2], &adcCh[3]);
#endif
    adcDiff[0] =  adc_2_temp(*adcDiffSource[1]) - adc_2_temp(*adcDiffSource[0]); // Fan controll is based on the temp diff
#ifndef STANDALONE
    peltier(&peltierData[0]);
    fan(&fanData[0]);
    lid(&lidData[0]);
    //lid(&lidData[1]);
#endif

#ifdef USE_TWO_LEVEL_LID_POWER
    if(1) { //(NoWell == mode) {
      // Bottom heaters disabled : full power on top heater
      if(MANUAL_STATE != lidData[0].regulator.state) {
        *pwmChMirror[0] = *pwmChMirror[1] = *(lidData[0].regulator.pwmVal);
      }
    }
    else
    {
      // Bottom heaters enabled : reduced power on top heater
      static char toggle = 0;
      if(0 == toggle)
      {
        *pwmChMirror[1] = *(lidData[0].regulator.pwmVal);
        *pwmChMirror[0] = 0;
        toggle = 1;
      }
      else
      {
        *pwmChMirror[0] = *(lidData[0].regulator.pwmVal);
        *pwmChMirror[1] = 0;
        toggle = 0;
      }
    }
#endif

    PWM_Set(pwmCh[0], PWM0_TIM4CH3);
    PWM_Set(pwmCh[1], PWM1_TIM4CH4);
    //PC6 is now Peltier_EN  PWM_Set(pwmCh[2], PWM2_TIM3CH1);
    PWM_Set(pwmCh[3], PWM3_TIM3CH2);
    PWM_Set(pwmCh[4], PWM4_TIM3CH3);
    
    if (dacCh[0] > DAC_UPPER_LIMIT) { dacCh[0] = DAC_UPPER_LIMIT; }
    DAC_SetChannel1Data(DAC_Align_12b_R, dacCh[0]);
    //DAC_SetChannel2Data(DAC_Align_12b_R, dacCh[1]);

    /* Add to log */
#ifdef USE_CL_DATA_LOGGING
    logUpdate(&adcCh[0], &adcCh[1], &adcCh[2], &adcCh[3]);
#endif

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
          fanData[0].regulator.state = CTRL_CLOSED_LOOP_STATE;
        }
        break;
        case SET_COOL_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          peltierData[0].regulator.setPoint = temp_2_adc(p->value);
          peltierData[0].regulator.state = CTRL_CLOSED_LOOP_STATE;
          coolTempOK = FALSE;
        }
        break;
        case SET_LID_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          if(p->idx-1 < nLID_HEATER) {
            lidData[p->idx-1].regulator.setPoint = temp_2_adc(p->value);
            lidData[p->idx-1].regulator.state = CTRL_CLOSED_LOOP_STATE;
            //lidData[1].regulator.state = CTRL_CLOSED_LOOP_STATE;
          }
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
          //lidData[1].regulator.state = CTRL_CLOSED_LOOP_STATE;
          coolTempOK = FALSE;
          lidTempOK = FALSE;
        }
        break;
        case STOP_LID_HEATING:
        {
          lidData[0].regulator.state = STOP_STATE;
          //setCLStatusReg(0xf00f); //Set flag for stopped lid heating ####JRJ
          //lidData[1].regulator.state = STOP_STATE;
        }
        break;
        case SET_LID_LOCK:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          if(1 == p->value) { GPIO_SetBits(GPIOA, LOCK_OUTPUT_PIN);   }
          if(0 == p->value) { GPIO_ResetBits(GPIOA, LOCK_OUTPUT_PIN); }
        }
        break;
        case SET_COOL_AND_LID:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          switch(p->idx) {
            case 0:
              peltierData[0].regulator.setPoint = temp_2_adc(p->value);
              coolTempOK = FALSE;
              break;
            case 1:
              lidData[0].regulator.setPoint = temp_2_adc(p->value);
              lidTempOK = FALSE;
              break;
            case 2:
              break;
            case 3:
              break;
            case 4: //Fan ctrl
              *fanData[0].regulator.pwmVal = p->value * 32768/100;
              PWM_Set(*fanData[0].regulator.pwmVal, PWM1_TIM4CH4);
              break;
            case 5:
              DEBUG_PRINTF("Set LidLock @ %d", p->value);
              if(1 == p->value) { GPIO_SetBits(LOCK_OUTPUT_PORT,   LOCK_OUTPUT_PIN); }
              if(0 == p->value) { GPIO_ResetBits(LOCK_OUTPUT_PORT, LOCK_OUTPUT_PIN); }
              break;
            case 6:
              PRINTF("ERROR - LogReq propagated to CL Task!");
              break;
            default:
              break;
          }
        }
        break;
        case SET_PWM:
        {
          SetPWMReq *pPWMReq;
          pPWMReq = (SetPWMReq *)(msg->ucData);
          pwmCh[pPWMReq->idx] = (pPWMReq->value * 32768/100);
          DEBUG_PRINTF("Set PWM ch %d @ %d", pPWMReq->idx, pwmCh[pPWMReq->idx]);
          switch(pPWMReq->idx)
          {
            case 0:
              /* pwmCh[0], TIM4,CH3 - PB8 - J175 : PWM0_TIM4CH3 - TopHeater1Ctrl */
              if(0 == pPWMReq->value) {
                lidData[0].regulator.state = STOP_STATE;
              } else {
                lidData[0].regulator.state = MANUAL_STATE;
              }
              break;
            case 1:
              /* pwmCh[1], TIM4,CH4 - PB9 - J26  : PWM1_TIM4CH4 - FAN control */
              if(0 == pPWMReq->value) {
                fanData[0].regulator.state = STOP_STATE;
              } else {
                fanData[0].regulator.state = MANUAL_STATE;
              }
              break;
            case 2:
              /* pwmCh[2], TIM3,CH1 - PC6 - J33  : PWM2_TIM3CH1 - Peltier PWM 1 */
              if(0 == pPWMReq->value) {
                peltierData[0].regulator.state = STOP_STATE;
              } else {
                peltierData[0].regulator.state = MANUAL_STATE;
              }
              break;
            case 3:
              /* pwmCh[3], TIM3,CH2 - PC7 - J176 : PWM3_TIM3CH2 - TopHeater2Ctrl */
              //lidData[1].regulator.state = MANUAL_STATE;
              break;
            case 4:
              /* pwmCh[4], TIM3,CH3 - PC8 - J35  : PWM4_TIM3CH3 - Peltier PWM 2 */
              // peltierData[1].regulator.state = MANUAL_STATE;
              break;
            default:
            break;
          }
        }
        break;
        case SET_DAC:
        {
          SetDACReq *p;
          p=(SetDACReq *)(msg->ucData);
          if(p->idx <= sizeof(dacCh)/sizeof(uint16_t))
          {
            dacCh[p->idx] = (((p->value * 4096) - 1)/100) & 0x0FFF;
            DEBUG_PRINTF("Set DAC ch %d @ %d", p->idx, dacCh[p->idx]);
            switch(p->idx)
            {
              case 0:
                if (0 == p->value) { 
                  GPIO_ResetBits(PELTIER_EN_PORT, PELTIER_EN_PIN); 
                } else { 
                  GPIO_SetBits(PELTIER_EN_PORT, PELTIER_EN_PIN);
                }
                peltierData[0].regulator.state = MANUAL_STATE;
                break;
              case 1:
                //??[?].regulator.state = MANUAL_STATE;
                break;
              default:
                break;
            }
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

