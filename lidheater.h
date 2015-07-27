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

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef enum {
  LID_HEATER_1,
  nLID_HEATER
} lidHeaterID_t;

typedef struct LID {
  lidHeaterID_t   		lidHeaterID;
  controllerState_t	 	state;
  io_t								io;
  controller_t        controller;
  rateLimiter_t       rateLimiter;
} lidHeater_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void init_lid_heater(lidHeater_t * lidHeater);
void lid_heater_init_feedback_ctr(controller_t * controller);
void lid_heater_controller(lidHeater_t *lidHeater);

#endif /* __LIDHEATER_H */

/************************ (C) COPYRIGHT Xtel *****END OF FILE****/
