/**
  ******************************************************************************
  * @file    pid.h 
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    8-Okt -2013
  * @brief   Header for pid controller
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PID_H
#define PID_H
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/*! \brief PID Status
 *
 * Setpoints and data used by the PID control algorithm
 */
typedef struct PID_DATA{
  //! Last process value, used to find derivative of process value.
  int16_t lastProcessValue;
  //! The Proportional tuning constant, multiplied with SCALING_FACTOR
  int16_t P_Factor;
  //! The Integral tuning constant, multiplied with SCALING_FACTOR
  int16_t I_Factor;
  //! The Derivative tuning constant, multiplied with SCALING_FACTOR
  int16_t D_Factor;
  //! Maximum allowed error, avoid overflow
  int16_t maxError;
  //! SetPoint
  int16_t setPoint;
  //! Maximum allowed sumerror, avoid overflow
  int32_t maxSumError;
  //! Summation of errors, used for integrate calculations
  int32_t sumError;
} pidData_t;

/* Private define ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define SCALING_FACTOR  (1<<13)

/*! \brief P, I and D parameter values
 *
 * The K_P, K_I and K_D values (P, I and D gains)
 * need to be modified to adapt to the application at hand
 */
//! \xrefitem todo "Todo" "Todo list"
#define K_P     3.00
//! \xrefitem todo "Todo" "Todo list"
#define K_I     0.01
//! \xrefitem todo "Todo" "Todo list"
#define K_D     0.00


/*! \brief Maximum values
 *
 * Needed to avoid sign/overflow problems
 */
// Maximum value of variables
#define MAX_INT         INT16_MAX
#define MAX_LONG        INT32_MAX
#define MAX_I_TERM      (MAX_LONG / 2)

// Boolean values
#define FALSE           0
#define TRUE            1

typedef enum {
	PID_P_FACTOR,
	PID_I_FACTOR,
	PID_D_FACTOR,
	nPID_FACTORS
} PidFactor_t;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, pidData_t *pid);
int16_t pid_Controller(int16_t processValue, pidData_t *pid_st);
void pid_ResetIntegrator(pidData_t *pid_st);

void pid_SetCoefficient(PidFactor_t factorselector, int16_t factor, pidData_t *pid);
void pid_SetSetPoint(int16_t setPoint, pidData_t *pid);

#endif /* PID_H */
  /************************ (C) COPYRIGHT Xtel *****END OF FILE****/

