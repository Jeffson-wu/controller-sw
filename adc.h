/**
  ******************************************************************************
  * @file    adc.h 
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    5 - Feb - 2015
  * @brief   Header for Control of M3 internal ADC
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"
#include <stm32f10x_adc.h>
/* Exported types ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
// Port A channels
#define ADC_PORT_TEMP_SENSE GPIOA
#define ADC_PIN_CH_0 GPIO_Pin_4
#define ADC_PIN_CH_1 GPIO_Pin_5 
#define ADC_PIN_CH_2 GPIO_Pin_6 
#define ADC_PIN_CH_3 GPIO_Pin_7
#define ADC_MUX_CH_0 ADC_Channel_4
#define ADC_MUX_CH_1 ADC_Channel_5
#define ADC_MUX_CH_2 ADC_Channel_6
#define ADC_MUX_CH_3 ADC_Channel_7
//if using AWD this channel is monitored
#define ADC_MUX_AWD_CH ADC_MUX_CH_3

// Port C channels
#define ADC_PORT_AUX_SENSE GPIOC
#define ADC_AUX_PIN_CH_0 GPIO_Pin_0
#define ADC_AUX_PIN_CH_1 GPIO_Pin_1
#define ADC_AUX_MUX_CH_0 ADC_Channel_10
#define ADC_AUX_MUX_CH_1 ADC_Channel_11

// Vmon channel
#define ADC_VMON_MUX_CH ADC_AUX_MUX_CH_0
// HW ID channel
#define ADC_HW_REV_ID_MUX_CH ADC_AUX_MUX_CH_1
// HW ID levels
#define HW_ID_REV0_LEVEL 124
#define HW_ID_REV1_LEVEL 248
#define HW_ID_REV2_LEVEL 372
#define HW_ID_REV3_LEVEL 496

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
// read error state
uint16_t getADSStatusReg(void);
// Set up ADC as we use it
void adcInit(void);
// Initialise analog watch dog
void awdInit(uint8_t chToWatch);
// Read a single channel
uint16_t readADC(u8 channel);
// Get HW REV ID
uint16_t readHwRevId(void);
// Retrieve latest value from all ADC channels.
void adcGetLatest(int16_t * ch0value, int16_t * ch1value, int16_t * ch2value, int16_t * ch3value);
// Setup timer to start conversions each 1/10 s
void adcConfigConversionTimer(tmrTIMER_CALLBACK convStartFn);
// ADC is started on timeout
void adcTimerCallback(xTimerHandle xTimer);
// Tell ADC driver which semaphore to use for sync.
void adcSetIsrSemaphore(xSemaphoreHandle sem);
void adcIrqInit(void);
void adcIrqDisable(void);
void adcIrqEnable(void);
// Start sequential convertion all four ch.
void adcStartSeq(void);
// Stop sequential convertion.
int32_t adc_2_temp(signed short dac);
// Convert temperature to ADC value
signed short temp_2_adc(int16_t temp);


#endif /* __ADC_H */

/************************ (C) COPYRIGHT Xtel *****END OF FILE****/
