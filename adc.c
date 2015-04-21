/**
  ******************************************************************************
  * @file    adc.c
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    5 - Feb - 2015
  * @brief   Control of M3 internal ADC
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"
#include "gdi.h" //Debug printf
#include "pwm.h" //PWM_Stop()

/* ---------------------------------------------------------------------------*/
/* Private feature defines ---------------------------------------------------*/
// Kill peltier in case of thermic runaway
//#define USE_ANALOG_WATCH_DOG 
#define AWD_HIGH_THRESHOLD 4096
#define ADC1_DR_Address ((uint32_t)0x4001244C)

#define BETA 3984.0
#define COEF_A (-43.1879)
#define COEF_B (-13.0872)

/* Private debug defines -----------------------------------------------------*/
//#define DEBUG

char buf[20];
#define PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);
#else
#define DEBUG_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif

static xSemaphoreHandle ADCSemaphore = NULL;

/* Private typedef -----------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/* Private prototypes                                                         */
/* ---------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/* functions                                                                  */
/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
#define HEART_BEAT_LED GPIOC,GPIO_Pin_9
int32_t adc_2_temp(signed short adc)
{
#if 0
  int64_t res;
  res = BETA / (1/adc);
#else
  float res;
#if 0
  res = BETA / ((1/adc) * COEF_A - COEF_B);
#else
  GPIO_SetBits(HEART_BEAT_LED);/*RX LED*/
  res = BETA / logf( ((1/adc) * COEF_A - COEF_B) );
  GPIO_ResetBits(HEART_BEAT_LED);/*RX LED*/
#endif
#endif
  return (int32_t)res;
}

/* ---------------------------------------------------------------------------*/
signed short temp_2_adc(int16_t temp)
{
  int64_t res;
  res = (10000*(((int64_t)temp*100)-77175))/29549;
  return (signed short)res;
}

/* ---------------------------------------------------------------------------*/
void adcInit()
{
  ADC_InitTypeDef   ADC_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  // Init clk
  /* SystemCoreClock is 72MHz. Prescale by min. 6 = 12MHz ADC clk */
  /* We need to convert 4 channels 10 times a second. 12M/4/10 = 300000 cycles */
  /* are available for each conversion */
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  // Init GPIO
  GPIO_InitStructure.GPIO_Pin = ADC_PIN_CH_0 | ADC_PIN_CH_1 | ADC_PIN_CH_2 | ADC_PIN_CH_3; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(ADC_PORT_TEMP_SENSE, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ADC_AUX_PIN_CH_0 | ADC_AUX_PIN_CH_1; 
  GPIO_Init(ADC_PORT_AUX_SENSE, &GPIO_InitStructure);

  //Init irq
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0D   /*0x0B - 0x0F */;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; /*dont care*/
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Init ADC
  ADC_DeInit(ADC1);
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_InjectedSequencerLengthConfig(ADC1, 4);
  ADC_InjectedChannelConfig(ADC1, ADC_MUX_CH_0, 1, ADC_SampleTime_239Cycles5);
  ADC_InjectedChannelConfig(ADC1, ADC_MUX_CH_1, 2, ADC_SampleTime_239Cycles5);
  ADC_InjectedChannelConfig(ADC1, ADC_MUX_CH_2, 3, ADC_SampleTime_239Cycles5);
  ADC_InjectedChannelConfig(ADC1, ADC_MUX_CH_3, 4, ADC_SampleTime_239Cycles5);
  ADC_InjectedDiscModeCmd(ADC1, DISABLE);
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
  ADC_ExternalTrigInjectedConvCmd(ADC1, DISABLE);
  
  ADC_RegularChannelConfig(ADC1, ADC_AUX_MUX_CH_0, 1, ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_AUX_MUX_CH_1, 2, ADC_SampleTime_239Cycles5);

  ADC_Cmd(ADC1, ENABLE); //Switch on ADC

  // Calibrate ADC
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));

}

/* ---------------------------------------------------------------------------*/
void awdInit(uint8_t chToWatch) /* Initialise analog watch dog */
{
#ifdef USE_ANALOG_WATCH_DOG
  ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleInjecEnable);
  ADC_AnalogWatchdogThresholdsConfig(ADC1, AWD_HIGH_THRESHOLD, 0); /*HighThreshold, LowThreshold */
  ADC_AnalogWatchdogSingleChannelConfig(ADC1, chToWatch);
#endif
}

/* ---------------------------------------------------------------------------*/
uint16_t readADC(u8 channel)
{
  uint16_t value;
  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_1Cycles5);
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  value = ADC_GetConversionValue(ADC1);
  DEBUG_PRINTF("adc %d = %d", channel, value);
  return value;
}

/* ---------------------------------------------------------------------------*/
/* Read WH ID pin. Convert measured voltage to HW Revision                    */
uint16_t readHwRevId(void)
{
  uint16_t hw_id_value;
  hw_id_value = readADC(ADC_HW_REV_ID_MUX_CH);
  if      (hw_id_value < HW_ID_REV0_LEVEL) return 1 /*HW_REV_0*/;
  else if (hw_id_value < HW_ID_REV1_LEVEL) return 2 /*HW_REV_1*/;
  else if (hw_id_value < HW_ID_REV0_LEVEL) return 3 /*HW_REV_2*/;
  else return 0;
}

/* ---------------------------------------------------------------------------*/
void adcStartSeq(void)
{
  DEBUG_PRINTF("adcStartSeq\r\n");
  adcIrqEnable();
}

/* ---------------------------------------------------------------------------*/
/* Start a conversion burst (4 conversions) upon time out */
/* The semaphore is given upon completion of the conversion see ADS_Handler() */
void adcTimerCallback(xTimerHandle xTimer)
{
  ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
}

/* ---------------------------------------------------------------------------*/
void adcGetLatest(int16_t * ch0value, int16_t * ch1value, int16_t * ch2value, int16_t * ch3value)
{
  taskENTER_CRITICAL(); //push irq state
  *ch0value = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
  *ch1value = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
  *ch2value = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
  *ch3value = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_4);
  taskEXIT_CRITICAL();
}

/* ---------------------------------------------------------------------------*/
/* ADC_IT_JEOC: Injected channel, ADC_IT_EOC: regular  channel. */
void adcIrqDisable(void)
{
  ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);
}

/* ---------------------------------------------------------------------------*/
void adcIrqEnable(void)
{
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
}

/* ---------------------------------------------------------------------------*/
void adcConfigConversionTimer(tmrTIMER_CALLBACK convStartFn)
{
  xTimerHandle xTimer;
  DEBUG_PRINTF("adcConfigConversionTimer\r\n");
  xTimer= xTimerCreate((char *)"ADCTimer",      // Just a text name, not used by the kernel.
                       ((configTICK_RATE_HZ)/SAMPLING_FREQUENCY),  // conversion frequency.
                       pdTRUE,          // The timers will auto-reload themselves when they expire.
                       (void *) 1,      // Assign each timer a unique id equal to its array index.
                       convStartFn      // Each timer calls the same callback when it expires.
                       );
                           
  if( xTimer == NULL )
  {
    // The timer was not created.
  }
  else
  {
    // Start the timer.  No block time is specified, and even if one was
    // it would be ignored because the scheduler has not yet been
    // started.
    if( xTimerStart( xTimer, 0 ) != pdPASS )
    {
      // The timer could not be set into the Active state.
    }
  }
}

/* ---------------------------------------------------------------------------*/
void adcSetIsrSemaphore(xSemaphoreHandle sem)
{
  ADCSemaphore = sem;
}

/* ---------------------------------------------------------------------------*/
/* Sequential ADC is started by calling adsStartSeq() which starts the first conversion on ch 0  */
/*                                                                                                                                   */
void ADC_Handler(void)
{
  //  vTraceStoreISRBegin(??);
  static portBASE_TYPE xHigherPriorityTaskWoken;

#ifdef USE_ANALOG_WATCH_DOG
  if(SET == ADC_GetITStatus(ADC1, ADC_IT_AWD))
  {
    ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);
    stopPeltier();// #### JRJ #### KILL Peltier.
    //xTimerPendFunctionCallFromISR( fn, NULL, (uint32_t) 0, &xHigherPriorityTaskWoken );
    //  vTraceStoreISREnd();
    if(SET == ADC_GetITStatus(ADC1, ADC_IT_JEOC)) { ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC); }
    if(SET == ADC_GetITStatus(ADC1, ADC_IT_EOC))  { ADC_ClearITPendingBit(ADC1, ADC_IT_EOC); }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    return;
  }
  else
  {
#endif
    /* ADC_IT_JEOC: injected channels */
    if(SET == ADC_GetITStatus(ADC1, ADC_IT_JEOC))
    {
      ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
      xHigherPriorityTaskWoken = pdFALSE;
      /* Synchronize adcConfigConversionTimer. Do not require context switch in case
         running task is lower prio adcConfigConversionTimer (pdTRUE to do so) */
      xSemaphoreGiveFromISR(ADCSemaphore, &xHigherPriorityTaskWoken); 
    }
    /* ADC_IT_EOC: Regular channels */
    if(SET == ADC_GetITStatus(ADC1, ADC_IT_EOC))
    {
      ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
#if 0
      xHigherPriorityTaskWoken = pdFALSE;
      /* Synchronize adcConfigConversionTimer. Do not require context switch in case
         running task is lower prio adcConfigConversionTimer (pdTRUE to do so) */
      xSemaphoreGiveFromISR(ADCSemaphore, &xHigherPriorityTaskWoken); 
#endif
    }
#ifdef USE_ANALOG_WATCH_DOG
  }
#endif
  //  vTraceStoreISREnd();
}



