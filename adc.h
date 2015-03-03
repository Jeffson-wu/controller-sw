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
/* Exported types ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
// read error state
uint16_t getADSStatusReg(void);
// Set up ADC as we use it
void adcInit(void);
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
