/**
  ******************************************************************************
  * @file    pid.h
  * @author  Jeppe Soendergaard Larsen
  * @version V1.0.0
  * @date    24-Aug-2015
  * @brief   Controller
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "pid.h"

/* ---------------------------------------------------------------------------*/
/* PID control algorithm.
 *
 *  Calculates output from setpoint, process value and PID status.
 *
 *  setPoint  Desired value.
 *  processValue  Measured value.
 *  pid_st  PID status struct.
 */

double rate_limiter(rateLimiter_t *rateLimiter, double input)
{
  double output;
  double diff = input - rateLimiter->output;

  if (diff > rateLimiter->slewRateRising)
    diff = rateLimiter->slewRateRising;
  if (diff < rateLimiter->slewRateFaling)
    diff = rateLimiter->slewRateFaling;

  output = rateLimiter->output + diff;
  rateLimiter->output = output;
  return output;
}

double feedback_controller(controller_t *controller, int16_t processValue)
{
  double input = (double)(controller->setPoint - processValue);

  double output;
  output = (controller->diff_eq.N0*input + controller->diff_eq.N1*controller->diff_eq.input - controller->diff_eq.D1*controller->diff_eq.output);
  controller->diff_eq.input = input;
  controller->diff_eq.output = output;
  return output;
}

void reset_controller(controller_t *controller)
{
  controller->diff_eq.input = 0;
	controller->diff_eq.output = 0;
}

void reset_rateLimiter(rateLimiter_t *rateLimiter, int16_t adc)
{
	rateLimiter->output = (double)adc;//-20000;
}
