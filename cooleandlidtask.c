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
//#define USE_TWO_LEVEL_LID_POWER
#define DISABLE_ERROR_REPOTING
#define USE_LID_DETECT_FEATURE


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

//#include "pid.h"
#include "fan.h"
#include "lidheater.h"
#include "peltier.h"
#include "pwm.h"
#include "gdi.h"
#include "util.h"
#include "nvs.h"
#include "debug.h"
#include <heater_reg.h>
#include "cooleandlidtask.h"

/* ---------------------------------------------------------------------------*/
#define PRINTF(fmt, args...)      sprintf(dbgbuf, fmt, ## args);  send_msg_on_monitor(dbgbuf);
#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...)      sprintf(dbgbuf, fmt, ## args);  send_msg_on_monitor(dbgbuf);
#else
#define DEBUG_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif
#ifdef DEBUG_LOGGING
#define DEBUG_LOGGING_PRINTF(fmt, args...)      sprintf(dbgbuf, fmt, ## args);  send_msg_on_monitor(dbgbuf);
#else
#define DEBUG_LOGGING_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif

#define PELT_EN_TOGGLE_TICKS  14400000  // 4 hours 4*60*60*(ticks/sec) 14400000ms : ( 500/*ms*/ / portTICK_PERIOD_MS )
#define PELT_EN_TOGGLE_DISABLE_TIME 10 // 10 ms
#define CL_STATUS_TICKS   300000  // 30 sec 0.5*60*(ticks/sec) 60000ms : ( 500/*ms*/ / portTICK_PERIOD_MS )
#define LOCK_OUTPUT_PIN   GPIO_Pin_8
#define LOCK_OUTPUT_PORT  GPIOA
#define LID_DETECT_PIN    GPIO_Pin_5
#define LID_DETECT_PORT   GPIOB
#define PELTIER_EN_PIN    GPIO_Pin_6
#define PELTIER_EN_PORT   GPIOC
#define PELTIER_DAC_PIN   GPIO_Pin_4
#define PELTIER_DAC_PORT  GPIOA

#ifdef USE_CL_DATA_LOGGING
  #define CL_SAMPLES_PER_LOG  10    //Each log is the avarage over this number of samples
  #define CL_LOG_ELEMENT_SIZE 5     //Log all four sensors
  #define CL_LOG_QUEUE_SIZE   5     //Queue length
#endif
#define Swap2Bytes(val) ( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) )

/* Private typedef -----------------------------------------------------------*/
const char *cl_states[] =
{
  "clnok",    		/* CL temperatures not reached 	- Not OK to start PCR            */
  "clok",			/* CL temperatures reached     	- OK to start PCR                */
  "cllidok",
  "cllidnok",		/* CL lid error                	- Not OK to start PCR            */
  "clcoolok",
  "clcoolnok",		/* CL peltier error            	- Not OK to start PCR            */
  "clambok",
  "clambnok"		/* CL ambient temp too high		- Not OK to start PCR            */
};

typedef enum CL_STATES_T {
  CL_STATE_CLNOK,
  CL_STATE_CLOK,
  CL_STATE_CLLIDOK,
  CL_STATE_CLLIDERROR,
  CL_STATE_CLCOOLOK,
  CL_STATE_CLCOOLERROR,
  CL_STATE_CLAMBOK,
  CL_STATE_CLAMBERROR,
  nCL_STATES
} cl_states_t;

typedef enum CL_LID_OPEN_STATE_T {
  CL_LID_ERROR,
  CL_LID_OPEN,
  CL_LID_CLOSED,
  nCL_LID_states
} cl_lid_open_states_t;

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
  u16 head; 
  u16 tail;
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
TimerHandle_t BQ24600TimerBegin;
TimerHandle_t BQ24600TimerEnd;
TimerHandle_t CLStatusTimer;
extern xQueueHandle TubeSequencerQueueHandle;
bool msgSent = FALSE;

// Events to be sent to the Linux box
static uint16_t cl_status = 0;
static cl_states_t clState = CL_STATE_CLNOK;
static bool coolTempOK = FALSE;
static bool lidTempOK = FALSE;
static bool coolTempError = TRUE;
static bool lidTempError = TRUE;
static bool ambTempError = TRUE;
static cl_lid_open_states_t lidState = CL_LID_ERROR;

// Parameters for ADC
static int16_t adcCh[4] = {0, 0, 0, 0};

// ADC diff for Fan
//static int16_t adcDiff[1] = {0};

// Parameters for PWM
static uint16_t pwmCh[6] = {0, 0, 0, 0, 0, 0};

// Parameters for DAC
static uint16_t dacCh[1] = {0};

/*                      GPIO           On PCB Rev3
 * pwmCh[0], TIM4,CH3 - PB8 -  J173 :  TopHeater1Ctrl - lid heater
 * pwmCh[1], TIM4,CH4 - PB9 -  J173 :  TopHeater2Ctrl - middle heater
 * pwmCh[2], TIM3,CH1 - PC6 -       :  -
 * pwmCh[3], TIM3,CH2 - PC7 -  J26  :  FAN control
 * pwmCh[4], TIM3,CH3 - PC8 -       :  AUX PWM
 * pwmCh[5],                           dummy PWM
 * DAC_OUT1     PA4
 * DAC_OUT2     PA5
 * ADC PCB Rev3:                       Usage            Text on PCB 
 * adcCh[0], ADC12_IN5 - PA5 - J177 :  Ambient Air      Top heater 1
 * adcCh[1], ADC12_IN6 - PA6 - J173 :  TopHeaterSens    Top heater PCB connector
 * adcCh[2], ADC12_IN7 - PA7 - J182 :  Cold side        Temp sensor 3
 * adcCh[3], ADC12_IN0 - PA0 - J181 :  Middle heater    Temp sensor 4

   NTC_ADC_IN0        a PA0/ADC12_IN0
   Peltier_DAC        a PA4/DAC_OUT1  (/ADC12_IN4)
   NTC_ADC_IN5        a PA5/ADC12_IN5 (/DAC_OUT2)
   NTC_ADC_IN6        a PA6/ADC12_IN6
   NTC_ADC_IN7        a PA7/ADC12_IN7
   LockCtrlOut        d PA8/TIM1_CH1  (/MCO)

   LID_DETECT         d PB5
   I2C1_SCL           d PB6/I2C1_SCL/TIM4_CH1
   I2C1_SDA           d PB7/I2C1_SDA/TIM4_CH2
   TopHeater1CtrlPWM  p PB8/TIM4_CH3
   TopHeater2CtrlPWM  p PB9/TIM4_CH4
   TopHeaterMainPWM   p PB10/TIM2_CH3   TP7

   ADC10_VMON         a PC0/ADC12_IN10
   HW_ID              a PC1/ADC12_IN11
   PeltierCtrl_EN     d PC6/TIM3_CH1
   FANctrlPWM         p PC7/TIM3_CH2
   AuxCtrlPWM         p PC8/TIM3_CH3    TP8
   HeartBeatLED       d PC9/TIM3_CH4
 */ 

// Fan controll is based on the temp diff: adcDiff[0] =  Fin temp - Ambient temp
//static int16_t *adcDiffSource[2] = {&adcCh[3], &adcCh[0]};

static int16_t *adcAmbient = &adcCh[3];

static peltier_t peltier[nPELTIER] = {
  {PELTIER_1, CTR_STOP_STATE, { &dacCh[0], &adcCh[2]}} //, {-26213}
};

static lidHeater_t lidHeater[nLID_HEATER] = {
  {LID_HEATER_1, CTR_STOP_STATE, {&pwmCh[1], &adcCh[1]}},
  {LID_HEATER_2, CTR_STOP_STATE, {&pwmCh[0], &adcCh[0]}} //3
};

//Todo: remove   static uint16_t *pwmChMirror[2] = {&pwmCh[0], &pwmCh[1]};

static fan_t fan[nFAN] = {
  {FAN_1, CTR_STOP_STATE, {&pwmCh[3]}} //, /*&adcDiff[0]*/ &adcCh[3]}} //0
};

calib_data_t __attribute__ ((aligned (2))) calib_data[3] = {
  /* Default calibration data - use if no valid data found in NVS */
    { 15000  },       /* Lid     90degC */
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
#ifdef USE_LID_DETECT_FEATURE
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif // USE_LID_DETECT_FEATURE
}

#ifdef USE_LID_DETECT_FEATURE
/* ---------------------------------------------------------------------------*/
cl_lid_open_states_t getLidState(void)
{
  return lidState;
}

/* ---------------------------------------------------------------------------*/
/* This fn is executed in the context of the TimerTask */
void LidDetectedFromTmrTask(void *pvParameter1, uint32_t lidDetectState)
{
  if(lidState != CL_LID_ERROR)
  { //do not report initial event
    // For now we only report a lid open event.
    if(1 == lidDetectState)
    { // Lid has been opened
      PRINTF("Lid open");
      lidState = CL_LID_OPEN;
      //setCLStatusReg(uint16_t status);
    }
    if(0 == lidDetectState)
    { // Lid has been closed
      PRINTF("Lid closed");
      lidState = CL_LID_CLOSED;
      //setCLStatusReg(uint16_t status);
    }
  }
  else
  { //use initial event to set state - do not report
    if(1 == lidDetectState)
    { // Lid has been opened
      lidState = CL_LID_OPEN;
    }
    if(0 == lidDetectState)
    { // Lid has been closed
      lidState = CL_LID_CLOSED;
    }
  }
}

/* ---------------------------------------------------------------------------*/
/* This fn is executed in the context of the sysTick ISR */
void vApplicationTickHook()
{
  // debounce is required
  static u16 pinState = 0xAAAA; // 1010101010101010 -> do not get all 0 or all 1 straighr away
  static bool state = FALSE;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if(Bit_SET == GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5))
  {
    pinState = (pinState << 1) + 1;
  }
  else
  {
    pinState = (pinState << 1);
  }
  if(0x0000 == pinState) 
  {
    if(TRUE == state)
    {
      state = FALSE;
      xTimerPendFunctionCallFromISR( LidDetectedFromTmrTask,
                               NULL, (uint32_t)0,
                               &xHigherPriorityTaskWoken);
    }
  }
  if(0xFFFF == pinState) 
  {
    if(FALSE == state)
    {
      state = TRUE;
      xTimerPendFunctionCallFromISR( LidDetectedFromTmrTask,
                             NULL, (uint32_t)1,
                             &xHigherPriorityTaskWoken);
    }
  }
}
#else
  void vApplicationTickHook(){}
#endif // USE_LID_DETECT_FEATURE

/* ---------------------------------------------------------------------------*/
void togglePeltierEnd() {
  GPIO_SetBits(  PELTIER_EN_PORT, PELTIER_EN_PIN);
}

void togglePeltierBegin() {
  BQ24600TimerEnd = xTimerCreate((char *)"BQ24600TimerEnd",
            PELT_EN_TOGGLE_DISABLE_TIME, // The timer period in ticks.
            pdFALSE,               // auto-reload.
            ( void * ) 0,         // id.
            togglePeltierEnd         // callback.
            );
  if( BQ24600TimerEnd == NULL ) {
    PRINTF("Peltier driver start toggle timer end not created");
  } else {
    GPIO_ResetBits(PELTIER_EN_PORT, PELTIER_EN_PIN);
    xTimerStart(BQ24600TimerEnd, 0);
  }

}

/* ---------------------------------------------------------------------------*/
void CLStatus( xTimerHandle pxTimer )
{
	long lArrayIndex;
	xMessage *msg;
	long *p;

  if(NULL != pxTimer)
  {
    /* Which timer expired? */
    lArrayIndex = (long) pvTimerGetTimerID( pxTimer );

  	msg = pvPortMalloc(sizeof(xMessage)+sizeof(long));
    if(NULL == msg) { configASSERT(pdFALSE); } // This is a fatal error
    msg->ucMessageID = CHECK_LID_PELTIER_TEMP;
    p = (long *)msg->ucData;
    *p = lArrayIndex+1;
    PRINTF("LID and PELTIER TIMER_EXPIRED");
    xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
  }
}

/* ---------------------------------------------------------------------------*/
void initTogglePeltierTimer() {
  BQ24600TimerBegin = xTimerCreate((char *)"BQ24600TimerBegin",
            PELT_EN_TOGGLE_TICKS, // The timer period in ticks.
            pdTRUE,               // auto-reload.
            ( void * ) 0,         // id.
            togglePeltierBegin         // callback.
            );
  if( BQ24600TimerBegin == NULL ) {
    PRINTF("Peltier driver restart timer not created");
  } else {
    xTimerStart(BQ24600TimerBegin, 0);
  }
}

/* ---------------------------------------------------------------------------*/
void initCLStatusTimer() {
  CLStatusTimer = xTimerCreate((char *)"CLStatusTimer",
  					CL_STATUS_TICKS, // The timer period in ticks.
  					pdFALSE,               // no reload.
            ( void * ) 0,         // id.
            CLStatus         // callback.
            );
  if( CLStatusTimer == NULL ) {
    PRINTF("CL Status restart timer not created");
  } else {
    xTimerStart(CLStatusTimer, 0);
  }
}

/* ---------------------------------------------------------------------------*/
void stopPeltier()
{
  GPIO_ResetBits(PELTIER_EN_PORT, PELTIER_EN_PIN);
  *peltier[0].io.ctrVal = 0;
  //*peltierData[1].regulator.pwmVal = 0;
  peltier[0].state = CTR_STOP_STATE;
  //peltierData[1].regulator.state = STOP_STATE;
}

/* ---------------------------------------------------------------------------*/
void standAlone() //These settings should be made from the Linux Box
{

  *peltier[0].io.ctrVal = 8000;
  *fan[0].io.ctrVal = 20000; //40% of 32767

  *lidHeater[0].io.ctrVal = 25000;
  *lidHeater[1].io.ctrVal = 0;
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
/* Read out latest running variables                                          */
/* This function is called from gdi and thus is executed in gdi context.      */
/* <uid>,coolandlid_monitor={th_pwm,th_mode,fan_pwm,peltier_dac,peltier_voltage } */
/* ---------------------------------------------------------------------------*/
int getCLMonitor(char *poutText)
{
  char str[20];
  *poutText = 0;  //add string termination
  strcat(poutText,"coolandlid_monitor={");
  Itoa(*lidHeater[0].io.ctrVal, str);   // lid_pwm
  strcat(poutText,str);
  strcat(poutText, ",");

  Itoa(*lidHeater[1].io.ctrVal, str);   // mid_pwm
  strcat(poutText,str);
  strcat(poutText, ",");

  Itoa(*fan[0].io.ctrVal, str);         // fan_pwm
  strcat(poutText,str);
  strcat(poutText, ","); 

  Itoa(*peltier[0].io.ctrVal, str);     // peltier_dac
  strcat(poutText,str);
  strcat(poutText, ","); 

  Itoa(peltier[0].voltage, str);  // peltier_voltage
  strcat(poutText,str);
  strcat(poutText, "}");

  return 0;
}


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
    DEBUG_LOGGING_PRINTF("dataQueueAdd @0x%08X %04x %04x %04x %04x", (unsigned int)poutData, data[0], data[1], data[2], data[3], data[4]);
    poutData->seqNum = seqNumber;
    poutData->cldata[0] = data[0];
    poutData->cldata[1] = data[1];
    poutData->cldata[2] = data[2];
    poutData->cldata[3] = data[3];
    poutData->cldata[4] = data[4];
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
  cl_dataLog.accum[0] = cl_dataLog.accum[1] = cl_dataLog.accum[2] = cl_dataLog.accum[3] = cl_dataLog.accum[4] = 0;
  //cl_logDataQueue.cl_logDataElement[0].seqNum = 0; // 4 elements
  //cl_logDataQueue.cl_logDataElement[0].cldata[0] = 0; // 4 data per element
}

/* ---------------------------------------------------------------------------*/
/* Linux Box interpretation = Ambient, top heater, cold side, warm side */

//void logUpdate(lidHeater_t *lidHeater, peltier_t *peltier, fan_t *fan, int16_t *ambient)
void logUpdate(lidHeater_t *lidHeater, peltier_t *peltier, fan_t *fan, lidHeater_t *midHeater)
{
  cl_dataLog.avgCnt += 1;
  cl_dataLog.accum[0] += adc_to_temp(&lidHeater->ntcCoef, lidHeater->adcValFilt);
  cl_dataLog.accum[1] += adc_to_temp(&midHeater->ntcCoef, midHeater->adcValFilt);
  cl_dataLog.accum[2] += adc_to_temp(&peltier->ntcCoef, peltier->adcValFilt);
  cl_dataLog.accum[3] += peltier->t_hot_est;
  cl_dataLog.accum[4] += adc_to_temp(&peltier->ntcCoef, (int16_t)*adcAmbient);

  // Is the log buffer full?
  if(CL_SAMPLES_PER_LOG <= cl_dataLog.avgCnt) {
    cl_data_t data[5];
    cl_dataLog.avgCnt = 0;
    //Do avarage of the values
    data[0] = cl_dataLog.accum[0]/CL_SAMPLES_PER_LOG;
    data[1] = cl_dataLog.accum[1]/CL_SAMPLES_PER_LOG;
    data[2] = cl_dataLog.accum[2]/CL_SAMPLES_PER_LOG;
    data[3] = cl_dataLog.accum[3]/CL_SAMPLES_PER_LOG;
    data[4] = cl_dataLog.accum[4]/CL_SAMPLES_PER_LOG;
    cl_dataLog.accum[0] = cl_dataLog.accum[1] = cl_dataLog.accum[2] = cl_dataLog.accum[3] = cl_dataLog.accum[4] = 0;
    //put in queue
    DEBUG_LOGGING_PRINTF("Li: %ld %04x,%04x,%04x,%04x", cl_dataLog.sequence, data[0], data[1], data[2], data[3], data[4]);
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

int getClLog(char *poutText, int maxlen )
{
  int i = 0;
  int nElements = 0;
  cl_logDataElement_t * pinData;
  char str[20];
  int dataAdded = 0;
  //const char *cl_output[] = {"OK", "OK", "OK", "OK", "OK"};

  *poutText = 0;

  addStrToBuf(poutText,"state={", maxlen);


  /*
  if( coolTempOK && lidTempOK && ambTempOK)
  {
  	cl_state = cl_states[CL_STATE_CLOK];
  }
  */

#ifdef DISABLE_ERROR_REPOTING
  coolTempError = FALSE;
  lidTempError = FALSE;
  ambTempError = FALSE;
#endif
  if( coolTempError )
  {
	  clState = CL_STATE_CLCOOLERROR;
  }
  else
  {
	  clState = CL_STATE_CLCOOLOK;
  }
  addStrToBuf(poutText, cl_states[clState], maxlen);
  addStrToBuf(poutText, ",", maxlen);

  if( lidTempError )
  {
	  clState = CL_STATE_CLLIDERROR;
  }
  else
  {
	  clState = CL_STATE_CLLIDOK;
  }
  addStrToBuf(poutText, cl_states[clState], maxlen);
  addStrToBuf(poutText, ",", maxlen);

  if( ambTempError )
  {
	  clState = CL_STATE_CLAMBERROR;
  }
  else
  {
	  clState = CL_STATE_CLAMBOK;
	  //clState = cl_states[CL_STATE_CLNOK];
  }
  addStrToBuf(poutText, cl_states[clState], maxlen);
  addStrToBuf(poutText, "}", maxlen);


  //#### strcat(poutText,",");

  // Send all available log elements
  while(NULL != (pinData = cl_dequeue(&cl_logDataQueue)) )
  {
    DEBUG_LOGGING_PRINTF("Lo: %ld %04x,%04x,%04x,%04x", pinData->seqNum, pinData->cldata[0], 
                         pinData->cldata[1], pinData->cldata[2], pinData->cldata[3]);
    dataAdded = 1;
    if(nElements == 0)
    { // Before payload
      addStrToBuf(poutText,",seq_number=", maxlen);
      Itoa(pinData->seqNum, str);
      addStrToBuf(poutText,str,maxlen);
      addStrToBuf(poutText,";log={", maxlen);
    }
    else
    { // Just next batch of payload data
      addStrToBuf(poutText, ",", maxlen);
    }
    for(i=0; i<CL_LOG_ELEMENT_SIZE; i++)
    { // Add payload
      //int templen=0;
      Itoa(pinData->cldata[i], str);
      //templen = strlen(str);
      addStrToBuf(poutText,str, maxlen);
      if(CL_LOG_ELEMENT_SIZE - 1 > i)
      { 
        addStrToBuf(poutText, ",", maxlen); 
      }
    }
    nElements+=CL_LOG_ELEMENT_SIZE;
  }
  
  if(dataAdded) { 
    poutText[strlen(poutText)]=0;
    addStrToBuf(poutText, "}>", maxlen); // #### Remove '>' when KS has updated tube mand.
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
            val = lidHeater[0].controller.setPoint;
            break;
          case MID_ADDR:
            val = lidHeater[1].controller.setPoint;
            break;
          case PELTIER_ADDR:
            val = peltier[0].controller.setPoint;
            break;
          case FAN_ADDR:
            val = fan[0].controller.setPoint;
            break;
          default:
            break;
        }
        break;
      case TUBE_TEMP_REG:
        switch(slave)
        {
          case LID_ADDR:
            val = adc_to_temp(&lidHeater[0].ntcCoef, lidHeater[0].adcValFilt);
            break;
          case MID_ADDR:
            val = adc_to_temp(&lidHeater[1].ntcCoef, lidHeater[1].adcValFilt);
            break;
          case PELTIER_ADDR:
          	val = adc_to_temp(&peltier[0].ntcCoef, peltier[0].adcValFilt);
            break;
          case FAN_ADDR:
          	val = adc_to_temp(&fan[0].ntcCoef, fan[0].adcValFilt);
            break;
          default:
            break;
        }
        break;
			case PELTIER_VOLTAGE:
				switch(slave)
				{
					case PELTIER_ADDR:
						val = peltier[0].voltage;
						break;
					default:
						break;
				}
				break;
      //if( (17 <= slave) && (19 >= slave) ) // slave 17 - 19 maps to coolandlid
      case TUBE1_TEMP_REG:
        val = adc_to_temp(&lidHeater[0].ntcCoef, lidHeater[0].adcValFilt);
        break;
      case TUBE2_TEMP_REG:
        val = adc_to_temp(&peltier[0].ntcCoef, peltier[0].adcValFilt);
        break;
      case TUBE3_TEMP_REG:
        val = adc_to_temp(&lidHeater[1].ntcCoef, lidHeater[1].adcValFilt);
        break;
      case TUBE4_TEMP_REG:
        val = adc_to_temp(&fan[0].ntcCoef, fan[0].adcValFilt);
        break;
// Latest available raw ADC data.
      case ADC_CODE1_REG:
        val = adcCh[0];
        break;
      case ADC_CODE2_REG:
        val = adcCh[1];
        break;
      case ADC_CODE3_REG:
        val = adcCh[2];
        break;
      case ADC_CODE4_REG:
        val = adcCh[3];
        break;
      case PWM_1_REG:
#ifdef USE_TWO_LEVEL_LID_POWER
        if(NoWell == mode) {  // Preheat state
          val = 2 * (*lidHeater[0].io.ctrVal); //Top heater mode full power
        } else {
          val = *lidHeater[0].io.ctrVal; //Top heater mode half power
        }
#else
        val = *lidHeater[0].io.ctrVal; //Top heater mode half power
#endif
        break;
      case PWM_2_REG:
        val = *peltier[0].io.ctrVal;
        break;
      case PWM_3_REG:
        val = *fan[0].io.ctrVal;
        break;
      case PWM_4_REG:
        val = peltier[0].voltage; // Peltier voltage
        break;
      case PWM_4_REG+1:
        val = *lidHeater[1].io.ctrVal;
        break;
      default:
        break;
    }
    //DEBUG_PRINTF("coolLidReadRegs s:%d, r:%d, d:%d", slave, addr + reg, val);
    if(0 != buffer) { *(buffer + reg) = Swap2Bytes(val); } // correct endianess
  }
  return FALSE;
}

/* ---------------------------------------------------------------------------*/
/* "addr" is first reg, "datasize" is reg count, "data" is wrong endian       */
bool coolLidWriteRegs(u8 slave, u16 addr, u16 *data, u16 datasize)
{
  int reg;
  uint16_t val;
  for (reg = 0; reg < datasize; reg++)
  {
    val = Swap2Bytes(*(data + reg)); // correct endianess
    //DEBUG_PRINTF("coolLidWriteRegs s:%d, r:%d, d:%d", slave, addr + reg, val);
    switch((heater_regs_t)(addr + reg))
    {
      case SETPOINT_REG:
        switch(slave)
        {
          case LID_ADDR:
            lidHeater[0].controller.setPoint =  val; //temp_to_adc(&lidHeater[0].ntcCoef, val); //ToDO split into pwm og temp sp
            lidTempOK = FALSE;
            break;
          case MID_ADDR:
            lidHeater[1].controller.setPoint = val; //temp_to_adc(&lidHeater[1].ntcCoef, val); //ToDO split into pwm og temp sp
            //lidTempOK = FALSE;
            break;
          case PELTIER_ADDR:
          	//peltier_setpoint(&peltier[0], val);
          	peltier[0].setPoint = val;
            coolTempOK = FALSE;
            break;
          case FAN_ADDR:
            fan[0].controller.setPoint = val;
            break;
          default:
            break;
        }
        break;
      case TUBE_COMMAND_REG:
        switch(val)
        {
          case WRITE_CAL_DATA:
            /* Here the slave is irrelevant */
            calib_data[0].c_1 = lidHeater[0].controller.setPoint;
            calib_data[1].c_1 = peltier[0].controller.setPoint;
            calib_data[2].c_1 = fan[0].controller.setPoint;
            NVSwrite(sizeof(calib_data), calib_data);
            DEBUG_PRINTF("\r\nWrote calib.\r\n");
            break;
          case SET_IDLE_MODE:
            switch(slave)
            {
              case LID_ADDR:
                lidHeater[0].state = CTR_STOP_STATE;
                lidHeater[1].state = CTR_STOP_STATE;
                break;
              case PELTIER_ADDR:
                peltier[0].state = CTR_STOP_STATE;
                break;
              case FAN_ADDR:
                fan[0].state = CTR_STOP_STATE;
                break;
              default:
                break;
            }
            break;
          case SET_AUTOMATIC_MODE:
            switch(slave)
            {
              case LID_ADDR:
                lidHeater[0].state = CTR_OPEN_LOOP_STATE;
                lidHeater[1].state = CTR_OPEN_LOOP_STATE;
                lidTempOK = FALSE;
                break;
              case PELTIER_ADDR:
                peltier[0].state = CTR_CLOSED_LOOP_STATE;
                coolTempOK = FALSE;
                break;
              case FAN_ADDR:
                fan[0].state = CTR_CLOSED_LOOP_STATE;
                break;
              default:
                break;
            }
            break;
          case SET_MANUEL_MODE:
            switch(slave)
            {
              case LID_ADDR:
                lidHeater[0].state = CTR_MANUAL_STATE;
                lidTempOK = FALSE;
                break;
              case MID_ADDR:
                lidHeater[1].state = CTR_MANUAL_STATE;
                //lidTempOK = FALSE;
                break;
              case PELTIER_ADDR:
                peltier[0].state = CTR_MANUAL_STATE;
                coolTempOK = FALSE;
                break;
              case FAN_ADDR:
                fan[0].state = CTR_MANUAL_STATE;
                break;
              default:
                break;
            }
            break;
          case SET_MANUEL_STOP:
            switch(slave)
            {
              case LID_ADDR:
                lidHeater[0].state = CTR_STOP_STATE;
                break;
              case MID_ADDR:
                lidHeater[1].state = CTR_STOP_STATE;
                break;
              case PELTIER_ADDR:
                peltier[0].state = CTR_STOP_STATE;
                break;
              case FAN_ADDR:
                fan[0].state = CTR_STOP_STATE;
                break;
              default:
                break;
            }
            break;
          case SET_CTR_OPEN_LOOP:
            switch(slave)
            {
              case LID_ADDR:
                lidHeater[0].state = CTR_OPEN_LOOP_STATE;
                lidHeater[1].state = CTR_OPEN_LOOP_STATE;
                lidTempOK = FALSE;
                break;
              case PELTIER_ADDR:
                peltier[0].state = CTR_OPEN_LOOP_STATE;
                coolTempOK = FALSE;
                break;
              case FAN_ADDR:
                fan[0].state = CTR_OPEN_LOOP_STATE;
                break;
              default:
                break;
            }
            break;
          case SET_CTR_CLOSED_LOOP:
            switch(slave)
            {
              case LID_ADDR:
                lidHeater[0].state = CTR_CLOSED_LOOP_STATE;
                lidHeater[1].state = CTR_CLOSED_LOOP_STATE;
                PRINTF("TUBE_COMMAND_REG")
                break;
              case PELTIER_ADDR:
                peltier[0].state = CTR_CLOSED_LOOP_STATE;
                break;
              case FAN_ADDR:
                fan[0].state = CTR_CLOSED_LOOP_STATE;
                break;
              default:
                break;
            }
            break;
          default:
            break;
        }
        default:
          break;
    }
  }
  return FALSE;
}


/* ---------------------------------------------------------------------------*/
uint16_t getPeltierVoltage()
{
	return 81*(readADC(ADC_VMON_MUX_CH)*0x400) / 0x100000; // y=(A*scale)*(ADC*scale)/(scale*scale), scale=1024(0x400)
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
  initTogglePeltierTimer();
  initCLStatusTimer();

  lidHeater[0].controller.setPoint = calib_data[0].c_1; //Todo: needs cleaning
  peltier[0].controller.setPoint = calib_data[1].c_1;
  fan[0].controller.setPoint     = calib_data[2].c_1;

  init_peltier(&peltier[0]);
  init_fan(&fan[0]);
  init_lid_heater(&lidHeater[0]);
  init_mid_heater(&lidHeater[1]);

  lidHeater[0].ntcCoef.beta = 3940;
  lidHeater[0].ntcCoef.r_s_ohm = 470;

  lidHeater[1].ntcCoef.beta = 3984;
  lidHeater[1].ntcCoef.r_s_ohm = 10000;

  init_median_filter(&fan[0].medianFilter);
  init_median_filter(&peltier[0].medianFilter);
  init_median_filter(&lidHeater[0].medianFilter);
  init_median_filter(&lidHeater[1].medianFilter);

  peltier[0].max_adc = temp_to_adc(&peltier[0].ntcCoef, 400); //300
  /*
  lidHeater[0].max_adc = temp_to_adc(&lidHeater[0].ntcCoef, 1300);
  lidHeater[0].min_adc = temp_to_adc(&lidHeater[0].ntcCoef, 500);
  lidHeater[1].max_adc = temp_to_adc(&lidHeater[1].ntcCoef, 1300);
  lidHeater[1].min_adc = temp_to_adc(&lidHeater[1].ntcCoef, 500);
   */

  lidState = getLidState();
  DEBUG_PRINTF("Lid state: %d", (int)lidState);

  while(1)
  {
  #ifdef DEBUG_COOL
    if (cnt == 1)
    {
      static int toggle = 0;
      //PRINTF("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d", (int16_t)peltier->controller.setPoint, *peltier[0].io.ctrVal, peltier[0].adcValFilt, (int16_t)fan[0].controller.setPoint, *fan[0].io.ctrVal, *fan[0].io.adcVal, (int16_t)lidHeater[0].controller.setPoint, *lidHeater[0].io.ctrVal, lidHeater[0].adcValFilt, *lidHeater[0].io.adcVal);
      //PRINTF("%d, %d, %d, %d, %d, %d, %d", (int16_t)peltier->controller.setPoint, *peltier[0].io.ctrVal, peltier[0].adcValFilt, getPeltierVoltage(), (int16_t)fan[0].controller.setPoint, *fan[0].io.ctrVal, *fan[0].io.adcVal);
      /*
      PRINTF("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
      			adc_to_temp(&peltier[0].ntcCoef, (int16_t)*adcAmbient),
      			adc_to_temp(&peltier[0].ntcCoef, (int16_t)peltier->controller.setPoint),
      			*peltier[0].io.ctrVal,
      			peltier[0].temp,
      			peltier[0].voltage,
      			peltier[0].t_hot_est,
      			fan[0].adcValFilt,
      			adc_to_temp(&fan[0].ntcCoef, (int16_t)fan[0].controller.setPoint),
      			*fan[0].io.ctrVal,
      			(int16_t)*lidHeater[0].io.adcVal,
      			(int16_t)lidHeater[1].adcValFilt,
      			(int16_t)*lidHeater[1].io.adcVal,
      			(int16_t)lidHeater[1].adcValFilt);
       */
      PRINTF("%d, %d, %d, %d, %d, %d, %d ",
    			(int16_t)*lidHeater[0].io.adcVal,
    			adc_to_temp(&lidHeater[0].ntcCoef, (int16_t)lidHeater[0].adcValFilt),
    			lidHeater[0].setPoint,
    			(int16_t)*lidHeater[1].io.adcVal,
    			adc_to_temp(&lidHeater[1].ntcCoef, (int16_t)lidHeater[1].adcValFilt),
    			lidHeater[1].setPoint,
    			peltier[0].voltage)

      //DEBUG_PRINTF("Adc:%4d, %4d, %4d, %4d", adcCh[0], adcCh[1], adcCh[2], adcCh[3]);
      cnt = 0;
      if(toggle) {
        //DEBUG_PRINTF("vmon: %d", readADC(ADC_VMON_MUX_CH));
        toggle = 0;
      } else {
        //DEBUG_PRINTF("hw_id: %d", readADC(ADC_HW_REV_ID_MUX_CH));
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
    //adcDiff[0] =  adc_to_temp(&fan[0].ntcCoef, *adcDiffSource[1]) - adc_to_temp(&fan[0].ntcCoef, *adcDiffSource[0]); // Fan controll is based on the temp diff
#ifndef STANDALONE

  	peltier[0].temp = adc_to_temp(&peltier[0].ntcCoef, peltier[0].adcValFilt);
    uint16_t peltierTemp = temp_to_adc(&peltier[0].ntcCoef, peltier[0].t_hot_est);
    fan[0].controller.setPoint = temp_to_adc(&fan[0].ntcCoef, 400);

    peltier[0].voltage = getPeltierVoltage();
    peltier_controller(&peltier[0]);
    fan_controller(&fan[0], peltierTemp);  // Todo T to ADC

    lid_heater_controller(&lidHeater[LID_HEATER_1]);
    lidHeater[LID_HEATER_2].state = CTR_CLOSED_LOOP_STATE;
    lid_heater_controller(&lidHeater[LID_HEATER_2]);

    /*
    if (peltier[0].error == TRUE || lidHeater[0].error == TRUE)
    {
    	peltier[0].state = CTR_STOP_STATE;
    	lidHeater[0].state = CTR_STOP_STATE;
    	lidHeater[1].state = CTR_STOP_STATE;
    }
    */

    switch (peltier->state) {
      case CTR_STOP_STATE:
      {
        GPIO_ResetBits(PELTIER_EN_PORT, PELTIER_EN_PIN);  //Disable Peltier
      }
      break;
      case CTR_OPEN_LOOP_STATE:     // Fall through to enable peltier
      case CTR_CLOSED_LOOP_STATE:   // Fall through to enable peltier
      case CTR_MANUAL_STATE:
      {
        GPIO_SetBits(PELTIER_EN_PORT, PELTIER_EN_PIN); //Enable peltier
      }
      break;
      default:
      break;
    }

    // ToDo: Find limits.
    if( abs( adc_to_temp(&peltier[0].ntcCoef, peltier[0].controller.setPoint) - adc_to_temp(&peltier[0].ntcCoef, *peltier[0].io.adcVal) ) < 10 ) { coolTempOK = TRUE; }
    if( abs( adc_to_temp(&lidHeater[0].ntcCoef, lidHeater[0].controller.setPoint) - adc_to_temp(&lidHeater[0].ntcCoef, *lidHeater[0].io.adcVal) ) < 10 ) { lidTempOK = TRUE; }
    //if( abs( adc_to_temp(&lidHeater[1].ntcCoef, lidHeater[1].controller.setPoint) - adc_to_temp(&lidHeater[1].ntcCoef, *lidHeater[1].io.adcVal) ) < 10 ) { lidTempOK = TRUE; }

    if( adc_to_temp(&peltier[0].ntcCoef, *adcAmbient) < 300 ) // same coef as for peltier
    {
    	ambTempError = FALSE;
    }
    else
    {
    	ambTempError = TRUE;
    }

#endif

#ifdef USE_TWO_LEVEL_LID_POWER
    // TODO: Reset controller when changing TH power state
    if(CL_STATE_CLNOK == clState) {
      // Bottom heaters disabled : full power on top heater
      //if(MANUAL_STATE != lidHeater[0].state) {
        *pwmChMirror[0] = *pwmChMirror[1] = *(lidHeater[0].io.ctrVal);
      //}
    }
    else
    {
      // Bottom heaters enabled : reduced power on top heater
      static char th_pwr_toggle = 0;
      if(0 == th_pwr_toggle)
      {
        *pwmChMirror[1] = *(lidHeater[0].io.ctrVal);
        *pwmChMirror[0] = 0;
        th_pwr_toggle = 1;
      }
      else
      {
        *pwmChMirror[0] = *(lidHeater[0].io.ctrVal);
        *pwmChMirror[1] = 0;
        th_pwr_toggle = 0;
      }
    }
#else
    // Full power on top heater
    //if(CTR_MANUAL_STATE != lidHeater[0].state) {
//Todo: Remove      *pwmChMirror[0] = *pwmChMirror[1] = *(lidHeater[0].io.ctrVal);
    //}
#endif

    PWM_Set(pwmCh[0], PWM0_TIM4CH3);  /* pwmCh[0], TIM4,CH3 - PB8 -  J175 :  TopHeater1Ctrl */
    PWM_Set(pwmCh[1], PWM1_TIM4CH4);  /* pwmCh[1], TIM4,CH4 - PB9 -  J26  :  TopHeater2Ctrl */
    PWM_Set(pwmCh[3], PWM3_TIM3CH2);  /* pwmCh[3], TIM3,CH2 - PC7 -  J176 :  FAN control    */
    
    if (dacCh[0] > DAC_UPPER_LIMIT) { dacCh[0] = DAC_UPPER_LIMIT; }
    DAC_SetChannel1Data(DAC_Align_12b_R, dacCh[0]);
    //DAC_SetChannel2Data(DAC_Align_12b_R, dacCh[1]);

    /* Add to log */
#ifdef USE_CL_DATA_LOGGING
    //logUpdate(&lidHeater[0], &peltier[0], &fan[0], &adcCh[3]);
    logUpdate(&lidHeater[0], &peltier[0], &fan[0], &lidHeater[1]);
#endif
    configASSERT(peltier[0].io.adcVal == &adcCh[2]);

    /* Handle incomming messages if any */
    if( xQueueReceive( CoolAndLidQueueHandle, &msg, /*Do not block*/ 0) == pdPASS )
    {
      switch(msg->ucMessageID)
      {
        case SET_FAN_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          fan_setpoint(&fan[0], p->value);
          //fan_setpoint(&fan[0], 500);
          fan[0].state = CTR_INIT;
        }
        break;
        case SET_COOL_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          peltier_setpoint(&peltier[0], p->value);
          peltier[0].state = CTR_INIT;
          coolTempOK = FALSE;
        }
        break;
        case SET_LID_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          lid_heater_setpoint(&lidHeater[0], p->value);
          lidHeater[0].state = CTR_INIT;
          lidHeater[1].state = CTR_INIT;
        }
        break;
        case SET_MID_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          lid_heater_setpoint(&lidHeater[1], p->value);
          lidHeater[0].state = CTR_INIT;
          lidHeater[1].state = CTR_INIT;
        }
        break;
        case SET_LID_PWM:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          *(lidHeater[p->idx-1].io.ctrVal) = (p->value);
          lidHeater[p->idx-1].state = CTR_MANUAL_STATE;
        }
        break;
        case START_LID_HEATING:
        {
          lidHeater[0].state = CTR_OPEN_LOOP_STATE;
          lidHeater[1].state = CTR_OPEN_LOOP_STATE;
          coolTempOK = FALSE;
          lidTempOK = FALSE;
        }
        break;
        case STOP_LID_HEATING:
        {
          lidHeater[0].state = CTR_STOP_STATE;
          lidHeater[1].state = CTR_STOP_STATE;
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
              peltier[0].controller.setPoint = temp_to_adc(&peltier[0].ntcCoef, p->value);
              coolTempOK = FALSE;
              break;
            case 1:
              lidHeater[0].controller.setPoint = temp_to_adc(&lidHeater[0].ntcCoef, p->value);
              lidHeater[1].controller.setPoint = temp_to_adc(&lidHeater[1].ntcCoef, p->value);
              lidTempOK = FALSE;
              break;
            case 2:
              break;
            case 3:
              break;
            case 4: //Fan ctrl
              //*fan[0].io.ctrVal = p->value * 32768/100;
              //PWM_Set(*fan[0].io.ctrVal, PWM1_TIM4CH4);
            	fan[0].controller.setPoint = temp_to_adc(&fan[0].ntcCoef, p->value);
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
                lidHeater[0].state = CTR_STOP_STATE;
              } else {
                lidHeater[0].state = CTR_MANUAL_STATE;
              }
              break;
            case 1:
              /* pwmCh[1], TIM4,CH4 - PB9 - J26  : PWM1_TIM4CH4 - FAN control */
              if(0 == pPWMReq->value) {
                fan[0].state = CTR_STOP_STATE;
              } else {
                fan[0].state = CTR_MANUAL_STATE;
              }
              break;
            case 2:
              /* pwmCh[2], TIM3,CH1 - PC6 - J33  : PWM2_TIM3CH1 - Peltier PWM 1 */
              if(0 == pPWMReq->value) {
                peltier[0].state = CTR_STOP_STATE;
              } else {
                peltier[0].state = CTR_MANUAL_STATE;
              }
              break;
            case 3:
              /* pwmCh[3], TIM3,CH2 - PC7 - J176 : PWM3_TIM3CH2 - TopHeater2Ctrl */
              //lid[1].state = MANUAL_STATE;
              break;
            case 4:
              /* pwmCh[4], TIM3,CH3 - PC8 - J35  : PWM4_TIM3CH3 - Peltier PWM 2 */
              // peltier[1].state = MANUAL_STATE;
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
                peltier[0].state = CTR_MANUAL_STATE;
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
        case CHECK_LID_PELTIER_TEMP: //Todo: Add middle heater temp check!
        {
        	int16_t setPointHyst = 1000;

          if (*lidHeater[0].io.adcVal > (lidHeater[0].controller.setPoint - setPointHyst))
          {
          	lidTempOK = TRUE;
          }
          else if (*lidHeater[0].io.adcVal < (lidHeater[0].controller.setPoint - setPointHyst))
          {
          	lidTempError = TRUE;
          }

          /*
          if (*lidHeater[1].io.adcVal > (lidHeater[1].controller.setPoint - setPointHyst))
          {
          	lidTempOK = TRUE;
          }
          else if (*lidHeater[1].io.adcVal < (lidHeater[1].controller.setPoint - setPointHyst))
          {
          	lidTempError = TRUE;
          }
          */

          if (*peltier[0].io.adcVal < (peltier[0].controller.setPoint + setPointHyst))
          {
          	coolTempOK = TRUE;
          }
          else if (*peltier[0].io.adcVal > (peltier[0].controller.setPoint + setPointHyst))
          {
          	coolTempError = TRUE;
          }
        }
        break;
        case WRITE_MODBUS_REGS:
          {
            WriteModbusRegsReq *p;
            p=(WriteModbusRegsReq *)(msg->ucData);
            /* "addr" is first reg, "datasize" is reg count, "data" is wrong endian       */
            coolLidWriteRegs(p->slave, p->addr, (u16 *)p->data, p->datasize);
          }
          break;
        case READ_MODBUS_REGS:
          {
            xMessage *msgout;
            ReadModbusRegsReq *p;
            ReadModbusRegsRes *po;
            p=(ReadModbusRegsReq *)(msg->ucData);
            if(p->reply)
            {
              msgout=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsRes)+p->datasize*sizeof(u16));
              if(msgout)
              {
                po=(ReadModbusRegsRes *)(msgout->ucData);
                msgout->ucMessageID=READ_MODBUS_REGS_RES;
                po->slave=p->slave;
                po->addr=p->addr;
                po->datasize=p->datasize;
                po->resultOk=FALSE;
                //bool coolLidReadRegs(u8 slave, u16 addr, u16 datasize, u16 *buffer)
                coolLidReadRegs(p->slave, p->addr, p->datasize, (u16*)po->data);
                po->resultOk=NO_ERROR;
                xQueueSend(p->reply, &msgout, portMAX_DELAY);
              }
            }
            else
            {
              coolLidReadRegs(p->slave, p->addr, p->datasize, 0);
            }
          }
        default:
          break; //ignore message
      }      
      vPortFree(msg);
    }
  }
  // We are not supposed to end, but if so kill this task.
  vTaskDelete(NULL);
}
