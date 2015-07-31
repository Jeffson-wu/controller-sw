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


/* Private debug define ------------------------------------------------------*/
//#define DEBUG /*General debug shows state changes of tubes (new temp, new time etc.)*/
#define DEBUG_COOL
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
#include "../heater-sw/heater_reg.h"
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
  #define CL_LOG_ELEMENT_SIZE 4     //Log all four sensors
  #define CL_LOG_QUEUE_SIZE   4     //Queue length
#endif
#define Swap2Bytes(val) ( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) )

/* Private typedef -----------------------------------------------------------*/
const char *cl_states[] =
{
  "clnok",    		/* CL temperatures not reached - Not OK to start PCR            */
  "clok",					/* CL temperatures reached     - OK to start PCR                */
  "clliderr",			/* CL lid error                - Not ok to start PCR            */
  "clcoolerr",		/* CL peltier error            - Not ok to start PCR            */
  "clcoolliderr", /* CL peltier & lid error      - Not ok to start PCR            */
};

typedef enum CL_STATES_T {
  CL_STATE_CLNOK,
  CL_STATE_CLOK,
  CL_STATE_CLLIDERROR,
  CL_STATE_CLCOOLERROR,
  CL_STATE_CLCOOLLIDERROR,
  nCL_STATES
} cl_states_t;

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
TimerHandle_t BQ24600Timer;
TimerHandle_t CLStatusTimer;
extern xQueueHandle TubeSequencerQueueHandle;
bool msgSent = FALSE;

// Events to be sent to the Linux box
static uint16_t cl_status = 0;
static uint8_t clState = CL_STATE_CLNOK;
static bool coolTempOK = FALSE;
static bool lidTempOK = FALSE;
static bool coolTempError = FALSE;
static bool lidTempError = FALSE;

// Parameters for ADC
static int16_t adcCh[4] = {0, 0, 0, 0};

// ADC diff for Fan
static int16_t adcDiff[1] = {0};

// Parameters for PWM
static uint16_t pwmCh[6] = {0, 0, 0, 0, 0, 0};

// Parameters for DAC
static uint16_t dacCh[1] = {0};

/*                      GPIO           On PCB Rev3
 * pwmCh[0], TIM4,CH3 - PB8 -  J175 :  TopHeater1Ctrl
 * pwmCh[1], TIM4,CH4 - PB9 -  J26  :  TopHeater2Ctrl
 * pwmCh[2], TIM3,CH1 - PC6 -  J33  :  -
 * pwmCh[3], TIM3,CH2 - PC7 -  J176 :  FAN control
 * pwmCh[4], TIM3,CH3 - PC8 -  J35  :  AUX PWM
 * pwmCh[5],                           dummy PWM
 * DAC_OUT1     PA4
 * DAC_OUT2     PA5
 * ADC PCB Rev3:                       Usage            Text on PCB 
 * adcCh[0], ADC12_IN5 - PA5 - J177 :  Ambient Air      Top heater 1
 * adcCh[1], ADC12_IN6 - PA6 - J173 :  TopHeaterSens    Top heater PCB connector
 * adcCh[2], ADC12_IN7 - PA7 - J182 :  Cold side        Temp sensor 3
 * adcCh[3], ADC12_IN0 - PA0 - J181 :  Hot side         Temp sensor 4

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
static int16_t *adcDiffSource[2] = {&adcCh[3], &adcCh[0]};

static peltier_t peltier[nPELTIER] = {
  {PELTIER_1, CTR_STOP_STATE, { &dacCh[0], &adcCh[2]}} //, {-26213}
};

static lidHeater_t lidHeater[nLID_HEATER] = {
  {LID_HEATER_1, CTR_STOP_STATE, {&pwmCh[5], &adcCh[1]}} //-26213,
};

static uint16_t *pwmChMirror[2] = {&pwmCh[0], &pwmCh[1]};

static fan_t fan[nFAN] = {
  {FAN_1, CTR_STOP_STATE, {&pwmCh[3], /*&adcDiff[0]*/ &adcCh[3]}} //0
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
}

/* ---------------------------------------------------------------------------*/
void togglePeltier() {
  GPIO_ResetBits(PELTIER_EN_PORT, PELTIER_EN_PIN);
  GPIO_SetBits(  PELTIER_EN_PORT, PELTIER_EN_PIN);
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
  BQ24600Timer = xTimerCreate((char *)"BQ24600Timer",
            PELT_EN_TOGGLE_TICKS, // The timer period in ticks.
            pdTRUE,               // auto-reload.
            ( void * ) 0,         // id.
            togglePeltier         // callback.
            );
  if( BQ24600Timer == NULL ) {
    PRINTF("Peltier driver restart timer not created");
  } else {
    xTimerStart(BQ24600Timer, 0);
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
  Itoa(*lidHeater[0].io.ctrVal, str);   // th_pwm
  strcat(poutText,str);
  strcat(poutText, ","); 
  Itoa(clState, str);                   // clState
  strcat(poutText,str);
  strcat(poutText, ","); 
  Itoa(*fan[0].io.ctrVal, str);         // fan_pwm
  strcat(poutText,str);
  strcat(poutText, ","); 
  Itoa(*peltier[0].io.ctrVal, str);     // peltier_dac
  strcat(poutText,str);
  strcat(poutText, ","); 
  Itoa(readADC(ADC_VMON_MUX_CH), str);  // peltier_voltage
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

  if( coolTempOK && lidTempOK )
  {
  	clState = CL_STATE_CLOK;
  }
  else if( coolTempError )
  {
  	clState = CL_STATE_CLCOOLERROR;
  }
  else if( lidTempError )
  {
  	clState = CL_STATE_CLLIDERROR;
  }
  else
  {
  	clState = CL_STATE_CLNOK;
  }

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
            val = lidHeater[0].controller.setPoint;
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
      //if( (17 <= slave) && (19 >= slave) ) // slave 17 - 19 maps to coolandlid
      case PWM_1_REG:
        //if(NoWell == mode) {  // Preheat state
        val = 2 * (*lidHeater[0].io.ctrVal); //Top heater mode full power
        // } else {
        val = *lidHeater[0].io.ctrVal; //Top heater mode half power
        // }
        break;
      case PWM_2_REG:
        val = *peltier[0].io.ctrVal;
        break;
      case PWM_3_REG:
        val = *fan[0].io.ctrVal;
        break;
      case PWM_4_REG:
        val = readADC(ADC_VMON_MUX_CH); // Peltire voltage
        break;
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
  uint16_t val;
  for (reg = 0; reg < datasize; reg++)
  {
    val = Swap2Bytes(*(data + reg)); // correct endianess
    switch((heater_regs_t)(addr + reg))
    {
      case SETPOINT_REG:
        switch(slave)
        {
          case LID_ADDR:
            lidHeater[0].controller.setPoint = val;
            lidTempOK = FALSE;
            break;
          case PELTIER_ADDR:
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
                lidHeater[0].state = CTR_CLOSED_LOOP_STATE;
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

  lidHeater[0].controller.setPoint = calib_data[0].c_1;
  peltier[0].controller.setPoint = calib_data[1].c_1;
  fan[0].controller.setPoint     = calib_data[2].c_1;

  init_peltier(&peltier[0]);
  init_fan(&fan[0]);
  init_lid_heater(&lidHeater[0]);

  while(1)
  {
  #ifdef DEBUG_COOL
    if (cnt == 10)
    {
      static int toggle = 0;
      //PRINTF("PEL_OUT:%d, PEL_IN:%d, FAN_OUT:%d, FAN_IN:%d, LID_OUT:%d, LID_IN:%d", dacCh[0], adcCh[0], pwmCh[3], adcCh[3], pwmCh[5], adcCh[1]);

      //PRINTF("%d, %d, %d, %d, %d, %d", *peltier[0].io.ctrVal, *peltier[0].io.adcVal, *fan[0].io.ctrVal, *fan[0].io.adcVal, *lidHeater[0].io.ctrVal, *lidHeater[0].io.adcVal);
      PRINTF("%d, %d, %d, %d, %d, %d, %d", *peltier[0].io.ctrVal, *peltier[0].io.adcVal, adcCh[2], *fan[0].io.ctrVal, *fan[0].io.adcVal, *lidHeater[0].io.ctrVal, *lidHeater[0].io.adcVal);

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
    configASSERT(peltier[0].io.adcVal == &adcCh[2]);

    #ifdef USE_M3_ADC
    adcGetLatest(&adcCh[0], &adcCh[1], &adcCh[2], &adcCh[3]);
#else
    adsGetLatest(&adcCh[0], &adcCh[1], &adcCh[2], &adcCh[3]);
#endif
    adcDiff[0] =  adc_2_temp(*adcDiffSource[1]) - adc_2_temp(*adcDiffSource[0]); // Fan controll is based on the temp diff
#ifndef STANDALONE

    configASSERT(peltier[0].io.adcVal == &adcCh[2]);
    peltier_controller(&peltier[0]);
    configASSERT(peltier[0].io.adcVal == &adcCh[2]);
    fan_controller(&fan[0]);
    configASSERT(peltier[0].io.adcVal == &adcCh[2]);
    lid_heater_controller(&lidHeater[0]);
    configASSERT(peltier[0].io.adcVal == &adcCh[2]);



    switch (peltier->state) {
      case CTR_STOP_STATE:
      {
        GPIO_ResetBits(PELTIER_EN_PORT, PELTIER_EN_PIN);  //Disable Peltier
      }
      break;
      case CTR_OPEN_LOOP_STATE:
      {
        GPIO_SetBits(PELTIER_EN_PORT, PELTIER_EN_PIN); //Enable peltier
      }
      break;
      case CTR_CLOSED_LOOP_STATE:
      {
        GPIO_SetBits(PELTIER_EN_PORT, PELTIER_EN_PIN); //Enable peltier
      }
      break;
      default:
      break;
    }

    if( abs(peltier->controller.setPoint - *peltier[0].io.adcVal) < 10 ) { coolTempOK = TRUE; }
    if( abs(lidHeater->controller.setPoint - *lidHeater[0].io.adcVal) < 10 ) { lidTempOK = TRUE; }

#endif

#ifdef USE_TWO_LEVEL_LID_POWER
    // TODO: Reset controller when changing TH power state
    if(CL_STATE_CLNOK == clState) {
      // Bottom heaters disabled : full power on top heater
      if(MANUAL_STATE != lidHeater[0].state) {
        *pwmChMirror[0] = *pwmChMirror[1] = *(lidHeater[0].io.ctrVal);
      }
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
    if(CTR_MANUAL_STATE != lidHeater[0].state) {
      *pwmChMirror[0] = *pwmChMirror[1] = *(lidHeater[0].io.ctrVal);
    }
#endif
    configASSERT(peltier[0].io.adcVal == &adcCh[2]);

    PWM_Set(pwmCh[0], PWM0_TIM4CH3);  /* pwmCh[0], TIM4,CH3 - PB8 -  J175 :  TopHeater1Ctrl */
    PWM_Set(pwmCh[1], PWM1_TIM4CH4);  /* pwmCh[1], TIM4,CH4 - PB9 -  J26  :  TopHeater2Ctrl */
    PWM_Set(pwmCh[3], PWM3_TIM3CH2);  /* pwmCh[3], TIM3,CH2 - PC7 -  J176 :  FAN control    */
    
    if (dacCh[0] > DAC_UPPER_LIMIT) { dacCh[0] = DAC_UPPER_LIMIT; }
    DAC_SetChannel1Data(DAC_Align_12b_R, dacCh[0]);
    //DAC_SetChannel2Data(DAC_Align_12b_R, dacCh[1]);
    configASSERT(peltier[0].io.adcVal == &adcCh[2]);

    /* Add to log */
#ifdef USE_CL_DATA_LOGGING
    logUpdate(&adcCh[0], &adcCh[1], &adcCh[2], &adcCh[3]);
#endif
    configASSERT(peltier[0].io.adcVal == &adcCh[2]);

    /* Handle incomming messages if any */
    if( xQueueReceive( CoolAndLidQueueHandle, &msg, /*Do not block*/ 0) == pdPASS )
    {
      switch(msg->ucMessageID)
      {
        case SET_FAN_SPEED:
        {
          long p;
          p = *((uint16_t *)(msg->ucData));
          *fan[0].io.ctrVal = p * 32768/100;
          fan[0].state = CTR_CLOSED_LOOP_STATE;
        }
        break;
        case SET_COOL_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          peltier[0].controller.setPoint = temp_2_adc(p->value);
          peltier[0].state = CTR_CLOSED_LOOP_STATE;
          coolTempOK = FALSE;
        }
        break;
        case SET_LID_TEMP:
        {
          SetCooleAndLidReq *p;
          p=(SetCooleAndLidReq *)(msg->ucData);
          if(p->idx-1 < nLID_HEATER) {
            lidHeater[p->idx-1].controller.setPoint = temp_2_adc(p->value);
            lidHeater[p->idx-1].state = CTR_CLOSED_LOOP_STATE;
            //lidData[1].regulator.state = CTRL_CLOSED_LOOP_STATE;
          }
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
          lidHeater[0].state = CTR_CLOSED_LOOP_STATE;
          //lidData[1].regulator.state = CTRL_CLOSED_LOOP_STATE;
          coolTempOK = FALSE;
          lidTempOK = FALSE;
        }
        break;
        case STOP_LID_HEATING:
        {
          lidHeater[0].state = CTR_STOP_STATE;
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
              peltier[0].controller.setPoint = temp_2_adc(p->value);
              coolTempOK = FALSE;
              break;
            case 1:
              lidHeater[0].controller.setPoint = temp_2_adc(p->value);
              lidTempOK = FALSE;
              break;
            case 2:
              break;
            case 3:
              break;
            case 4: //Fan ctrl
              *fan[0].io.ctrVal = p->value * 32768/100;
              PWM_Set(*fan[0].io.ctrVal, PWM1_TIM4CH4);
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
        case CHECK_LID_PELTIER_TEMP:
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
        default:
          break; //ignore message
      }      
      vPortFree(msg);
    }
  }
  // We are not supposed to end, but if so kill this task.
  vTaskDelete(NULL);
}
