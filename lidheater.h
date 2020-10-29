/**
  ******************************************************************************
  * @file    lidheater.h
  * @author  Jeppe Soendergaard Larsen <jsl@xtel.dk>
  * @version V1.0.0
  * @date    24-Aug-2015
  * @brief   Lid Heater Control
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIDHEATER_H
#define __LIDHEATER_H

#include "pid.h"
#include <math.h>

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/




typedef enum {
  MID_HEATER,
  LID_HEATER,
  nLID_HEATER
} lidHeaterID_t;
/*
typedef enum {
  LID_HEATER_1,
  LID_HEATER_2,
  nLID_HEATER
} lidHeaterID_t;
*/





typedef struct LID {
  lidHeaterID_t   		lidHeaterID;
  controllerState_t	 	state;
  io_t								io;
  controller_t        controller;
  rateLimiter_t       rateLimiter;
  medianFilter_t      medianFilter;
  ntcCoef_t           ntcCoef;
  int16_t             setPoint;
  int16_t             adcValFilt;
  int16_t							setPointLow;
  //int16_t							setPointLow_1;
  //int16_t							max_adc;
  //int16_t							min_adc;
  //int8_t							error;
} lidHeater_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void init_lid_heater(lidHeater_t * lidHeater);
void init_mid_heater(lidHeater_t * lidHeater);
void lid_heater_init_feedback_ctr(controller_t * controller);
void mid_heater_init_feedback_ctr(controller_t * controller);
void lid_heater_controller(lidHeater_t *lidHeater);
void lid_heater_init_rate_limiter(rateLimiter_t * rateLimiter);
void lid_heater_setpoint(lidHeater_t * lidHeater, int16_t value);
void lid_heater_init_ntc_coef(ntcCoef_t * ntcCoef);
int16_t lid_heater_power(lidHeater_t *lidHeater);

#endif /* __LIDHEATER_H */

/************************ (C) COPYRIGHT Xtel *****END OF FILE****/
