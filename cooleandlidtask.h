/**
  ******************************************************************************
  * @file    cooleandlidtask.h 
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    30-Oct -2013
  * @brief   Header for Cooler and lid heater task
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COOLERANDLIDTASK_H
#define __COOLERANDLIDTASK_H

#include "pid.h"

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
  LID_ADDR = 17,
  PELTIER_ADDR,
  //PELTIER_ADDR2,
  FAN_ADDR,
  NEIGHBOUR_TUBE_TEMP_SWITCH,
  nofADDR
} coolandlid_addrs_t;

#if 0
typedef enum {
  STOP_STATE,
  MANUAL_STATE,
  CTRL_OPEN_LOOP_STATE,
  CTRL_CLOSED_LOOP_STATE,
  nCTRL_STATES
} controllerState_t;
#endif

#if 0
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
#endif

#if 0
typedef struct {
  controllerState_t state;
  int16_t           setPoint;
  uint16_t          *pwmVal;
  int16_t           *adcVal;
  int16_t           setPointLL;
  int16_t           setPointHL;
  int8_t            hysteresisActiveFlag;
} regulatorData_t;
#endif

#if 0

typedef struct {
  uint16_t             *ctrVal;
  int16_t             *adcVal;
} io_t;

#if 0
typedef struct {
  controllerState_t 	state;
  io_t								io;
  controller_t        controller;
  rateLimiter_t       rateLimiter;
} regulatorData_t;
#endif

typedef struct FAN {
  fanID_t         		fanID;
  controllerState_t 	state;
  io_t								io;
  controller_t        controller;
  rateLimiter_t       rateLimiter;
} fan_t;

typedef struct PELTIER {
  peltierID_t     		peltierID;
  controllerState_t 	state;
  io_t								io;
  controller_t        controller;
  rateLimiter_t       rateLimiter;
} peltier_t;

typedef struct LID {
  lidHeaterID_t   		lidHeaterID;
  controllerState_t 	state;
  io_t								io;
  controller_t        controller;
  rateLimiter_t       rateLimiter;
} lid_t;
#endif

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void CoolAndLidTask( void * pvParameters );
int getClLog(char *poutText );
bool coolLidReadRegs(u8 slave, u16 addr, u16 datasize, u16 *buffer);
bool coolLidWriteRegs(u8 slave, u16 addr, u16 *data, u16 datasize);
int getCoolandlidHWReport(char *poutText);
int getAdc(char *poutText);

#if 0
void init_ctr_feedback_coef(controller_t * controller);
void fan_controller(fan_t *fan);
void peltier_controller(peltier_t *peltier);
void lid_controller(lid_t *lid);
#endif

#endif /* __COOLERANDLIDTASK_H */

/************************ (C) COPYRIGHT Xtel *****END OF FILE****/

