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

int16_t adc_to_temp(ntcCoef_t *ntcCoef, int16_t adc)
{
  int16_t temp;
  temp = (ntcCoef->beta / (logf((ntcCoef->r_s_ohm*(MAX_DAC_VALUE - adc)) / (adc*R_R_OHM) + ntcCoef->beta / T_R_KEL) - 273.15)) * 10.00;
  return temp;
}

/* ---------------------------------------------------------------------------*/
int16_t temp_to_adc(ntcCoef_t *ntcCoef, int16_t temp)
{
  int16_t adc;
  adc = (ntcCoef->r_s_ohm*MAX_DAC_VALUE / (R_R_OHM*exp(ntcCoef->beta*(1.00 / (temp / 10.00 + 273.15) - 1.00 / T_R_KEL)) + ntcCoef->r_s_ohm));
  return adc;
}

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
  //double input = (double)(processValue - controller->setPoint);
  double input = (double)(controller->setPoint - processValue);
  double output = (controller->diff_eq.N0*input + controller->diff_eq.N1*controller->diff_eq.input - controller->diff_eq.D1*controller->diff_eq.output);
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
	rateLimiter->output = (double)adc;
}

int16_t filter(filter_t *filter, int16_t adc)
{
	uint32_t samplesPerLog = 10;

	filter->adcValAccum += adc;
	if (0 == ++filter->avgCnt%samplesPerLog) 
	{
	    filter->avgCnt = 0;
	    filter->adcValMean = filter->adcValAccum / samplesPerLog;
	    filter->adcValAccum = 0;
	}
  return filter->adcValMean;
}

void reset_filter(filter_t *filter, int16_t adc)
{
  filter->avgCnt = 0;
  filter->adcValAccum = 0;
  filter->adcValMean = adc;
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


