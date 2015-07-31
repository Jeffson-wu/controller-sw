/**
  ******************************************************************************
  * @file    fan.h
  * @author  Jeppe Soendergaard Larsen <jsl@xtel.dk>
  * @version V1.0.0
  * @date    24-Aug-2015
  * @brief   Fan Control
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FAN_H
#define __FAN_H

#include "pid.h"

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef enum {
  FAN_1,
  nFAN
} fanID_t;

typedef struct FAN {
  fanID_t         		fanID;
  controllerState_t				 	state;
  io_t								io;
  controller_t        controller;
  rateLimiter_t       rateLimiter;
} fan_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void init_fan(fan_t * fan);
void fan_init_feedback_ctr(controller_t * controller);
void fan_controller(fan_t * fan);

#endif /* __FAN_H */

/************************ (C) COPYRIGHT Xtel *****END OF FILE****/
