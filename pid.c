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

void init_median_filter(medianFilter_t *medianFilter)
{
  int i;
  for(i=0;i<MEDIAN_LENGTH;i++) {medianFilter->samples[i]=0; medianFilter->sortIdx[i]=i;}
  medianFilter->samplesIdx = 0;
}

int16_t median_filter(medianFilter_t *medianFilter, int16_t sample)
{ /* for simplicity only odd length is allowed */
  #if (MEDIAN_LENGTH < 3) || ( (MEDIAN_LENGTH % 2) == 0)
  #error "MEDIAN_LENGTH needs to be odd and at least 3"
  #endif
  uint8_t i, j;
  uint8_t tmp;

  /* insert new sample */
  medianFilter->samples[medianFilter->samplesIdx++] = sample;
  if(medianFilter->samplesIdx > MEDIAN_LENGTH-1) { medianFilter->samplesIdx = 0; } // circular buffer

  for(i = 0; i < MEDIAN_LENGTH - 1; i++)
  {
    for(j = 0; j < MEDIAN_LENGTH - i - 1; j++)
    {
      if(medianFilter->samples[medianFilter->sortIdx[j]] < medianFilter->samples[medianFilter->sortIdx[j+1]])
        {//swap
          tmp = medianFilter->sortIdx[j];
          medianFilter->sortIdx[j] = medianFilter->sortIdx[j+1];
          medianFilter->sortIdx[j+1] = tmp;
        }
    }
  }
  return medianFilter->samples[medianFilter->sortIdx[(MEDIAN_LENGTH/2)]]; // return middle value
}


