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
  **/

/* Private feature defines ---------------------------------------------------*/
/* Private debug define ------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "fan.h"

/* ---------------------------------------------------------------------------*/
/* Fan handling */
/* ---------------------------------------------------------------------------*/
void init_fan(fan_t * fan)
{
	fan_init_feedback_ctr(&fan->controller);
  fan_init_rate_limiter(&fan->rateLimiter);
  init_median_filter(&fan->medianFilter);
  fan_init_adc_to_temp(&fan->ntcCoef);
  fan->controller.setPoint = fan->setPoint = 0;
}

void fan_init_feedback_ctr(controller_t * controller)
{
  controller->diff_eq.N0 = 3.015000E0;
  controller->diff_eq.N1 = -2.985000E0;
  controller->diff_eq.D1 = -1.000000E0;

  controller->diff_eq.minOutputValue = 15000; // fan stops below    //must be 0 in sim.... add heat from peltier.
  controller->diff_eq.maxOutputValue = 24000; // fan whistles above 25000
}

void fan_init_rate_limiter(rateLimiter_t * rateLimiter)
{
  rateLimiter->slewRateRising = 0;
  rateLimiter->slewRateFaling = -0;
}

void fan_setpoint(fan_t *fan, int16_t value)
{
  fan->setPoint = fan->controller.setPoint = temp_to_adc(&fan->ntcCoef, value);
}

void fan_init_adc_to_temp(ntcCoef_t * ntcCoef)
{
  ntcCoef->beta = 3984;
  ntcCoef->r_s_ohm = 10000;
}

void fan_controller(fan_t *fan)
{
  uint16_t ctr_out = 0;
  fan->adcValFilt = median_filter(&fan->medianFilter, *fan->io.adcVal);

  switch (fan->state) {
    case CTR_STOP_STATE:
    {
      *fan->io.ctrVal = 0;
      reset_controller(&fan->controller);
      reset_rateLimiter(&fan->rateLimiter, *fan->io.adcVal);
      fan->state = CTR_CLOSED_LOOP_STATE; // ToDo: FJERN NAAR KIM HAR FIKSET KOMMANDOEN FRA LINUX - JSL!!
      fan->setPoint = 2500;
    }
    break;
    case CTR_INIT:
    {
      reset_controller(&fan->controller);
      reset_rateLimiter(&fan->rateLimiter, *fan->io.adcVal);
      fan[0].state = CTR_CLOSED_LOOP_STATE;
    }
    break;
    case CTR_MANUAL_STATE:
    {
      ctr_out = (uint16_t)fan->setPoint;
    } 
    break;
    case CTR_OPEN_LOOP_STATE:
    {
    	return;
    } 
    break;
    case CTR_CLOSED_LOOP_STATE:
    {
    	fan->controller.setPoint = (double)fan->setPoint;
      ctr_out = (uint16_t)feedback_controller_neg(&fan->controller, fan->adcValFilt);
    }
    break;
    default:
    break;
  }
  
  if (ctr_out > fan->controller.diff_eq.maxOutputValue)
  {
  	ctr_out = fan->controller.diff_eq.maxOutputValue;
  }
  if (ctr_out < fan->controller.diff_eq.minOutputValue)
  {
  	ctr_out = fan->controller.diff_eq.minOutputValue;
  }

  *fan->io.ctrVal = ctr_out;
}
