/**
  ******************************************************************************
  * @file    peltier.h
  * @author  Jeppe Soendergaard Larsen <jsl@xtel.dk>
  * @version V1.0.0
  * @date    24-Aug-2015
  * @brief   Peltier control
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PELTIER_H
#define __PELTIER_H

#include "pid.h"

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef enum {
  PELTIER_1,
  nPELTIER
} peltierID_t;

typedef struct PELTIER {
  peltierID_t     		peltierID;
  controllerState_t		state;
  io_t								io;
  controller_t        controller;
  rateLimiter_t       rateLimiter;
  medianFilter_t      medianFilter;
  ntcCoef_t           ntcCoef;
  //int16_t             (*adc_to_temp)(peltier_t);
  int16_t             setPoint;
  int16_t             adcValFilt;
  uint16_t						voltage;
  int16_t							t_hot_est;
  int16_t							max_adc;
  int8_t							error;
  int16_t							temp;
  filter_t            filter;
} peltier_t;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void init_peltier(peltier_t * peltier);
void peltier_init_feedback_ctr(controller_t * controller);
void peltier_controller(peltier_t *peltier);
void peltier_init_rate_limiter(rateLimiter_t * rateLimiter);
void peltier_setpoint(peltier_t * peltier, int16_t value);
void peltier_init_adc_to_temp(ntcCoef_t * ntcCoef);
uint16_t getPeltierVoltage();
void init_lp_filter(lp_filter_t * lp_filter);
void init_estfilter(lp_filter_t * lp_filter);

#endif /* __PELTIER_H */

/************************ (C) COPYRIGHT Xtel *****END OF FILE****/
