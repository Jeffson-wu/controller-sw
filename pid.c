/**
  ******************************************************************************
  * @file    pid.c
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    8-Okt -2013
  * @brief   This file provides firmware functions to manage the PID Controller
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "stdint.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Initialisation of PID controller parameters.
 *
 *  Initialise the variables used by the PID algorithm.
 *
 *  p_factor  Proportional term.
 *  i_factor  Integral term.
 *  d_factor  Derivate term.
 *  pid  Struct with PID status.
 */
void pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, pidData_t *pid)
// Set up PID controller parameters
{
  // Start values for PID controller
  pid->sumError = 0;
  pid->lastProcessValue = 0;
  // Tuning constants for PID loop
  pid->P_Factor = p_factor;
  pid->I_Factor = i_factor;
  pid->D_Factor = d_factor;
  // Limits to avoid overflow
  pid->maxError = MAX_INT / (pid->P_Factor + 1);
  pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
  pid->setPoint = 0;
}


/* ---------------------------------------------------------------------------*/
/* PID control algorithm.
 *
 *  Calculates output from setpoint, process value and PID status.
 *
 *  setPoint  Desired value.
 *  processValue  Measured value.
 *  pid_st  PID status struct.
 */
int16_t pid_Controller(int16_t processValue, pidData_t *pid_st)
{
  int32_t ret, error, i_term, p_term, d_term, temp;

  error = pid_st->setPoint - processValue;

#if 0
  // Calculate Pterm and limit error overflow
  if (error > pid_st->maxError){
    p_term = MAX_INT;
  }
  else if (error < -pid_st->maxError){
    p_term = -MAX_INT;
  }
  else{
    p_term = pid_st->P_Factor * error;
  }
#endif
  p_term = (pid_st->P_Factor * error);

  // Calculate Iterm and limit integral runaway
  temp = pid_st->sumError + error;
  if(temp > pid_st->maxSumError){
    i_term = MAX_I_TERM;
    pid_st->sumError = pid_st->maxSumError;
  }
  else if(temp < -pid_st->maxSumError){
    i_term = -MAX_I_TERM;
    pid_st->sumError = -pid_st->maxSumError;
  }
  else{
    pid_st->sumError = temp;
    i_term = pid_st->I_Factor * pid_st->sumError;
  }

  // Calculate Dterm
  d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);

  pid_st->lastProcessValue = processValue;

  ret = (p_term + i_term + d_term) / SCALING_FACTOR;
  if(ret > MAX_INT){
    ret = MAX_INT;
  }
  else if(ret < -MAX_INT){
    ret = -MAX_INT;
  }

  return((int16_t)ret);
}

/*! \brief Resets the integrator.
 *
 *  Calling this function will reset the integrator in the PID regulator.
 */
void pid_ResetIntegrator(pidData_t *pid_st)
{
  pid_st->sumError = 0;
}

/* ---------------------------------------------------------------------------*/
void pid_SetCoefficient(PidFactor_t factorselector, int16_t factor, pidData_t *pid_st)
{
  switch(factorselector)
  {
    case PID_P_FACTOR:
    {
      pid_st->P_Factor = factor;
    }
    break;
    case PID_I_FACTOR:
    {
      pid_st->P_Factor = factor;
    }
    break;
    case PID_D_FACTOR:
    {
      pid_st->D_Factor = factor;
    }
    break;
    default:
    break;
  }
}

/* ---------------------------------------------------------------------------*/
void pid_SetSetPoint(int16_t setPoint, pidData_t *pid_st)
{
  pid_st->setPoint = setPoint;
}



/* ---------------------------------------------------------------------------*/

