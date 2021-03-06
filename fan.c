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
//#include "sequencer.h"
//#include "gdi.h"

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
	/*
  controller->diff_eq.N0 = 3.015000E0;
  controller->diff_eq.N1 = -2.985000E0;
  controller->diff_eq.D1 = -1.000000E0;
  */

  controller->diff_eq.N0 = 2.01E1; //10.25000;
  controller->diff_eq.N1 = -1.99E1; //-9.750000;
  controller->diff_eq.D1 = -1.000000;

  controller->diff_eq.minOutputValue = 15000; // fan stops below    //must be 0 in sim.... add heat from peltier.
  controller->diff_eq.maxOutputValue = 32767; // fan whistles above 25000
  //controller->diff_eq.maxOutputValue = 22340; // fan become too noisy above - ref 11v
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

int16_t fan_power(fan_t *fan)
{
  return fan->controller.diff_eq.output/PWM_UPPER_LIMIT*77; //Watt*10 new fan
}

void fan_controller(fan_t *fan, int16_t peltierTemp)
{
  uint16_t ctr_out = 0;
  //fan->adcValFilt = median_filter(&fan->medianFilter, *fan->io.adcVal);
  fan->adcValFilt = median_filter(&fan->medianFilter, peltierTemp);

  switch (fan->state) {
    case CTR_STOP_STATE:
    {
      *fan->io.ctrVal = 0;
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
      ctr_out = (uint16_t)fan->controller.setPoint;
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

      //PRINTF("FAN: %d, %d, %d, %d\n", (int16_t)peltierTemp, (int16_t)fan->adcValFilt, (int16_t)fan->controller.setPoint, (int16_t)ctr_out);

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
    if(CTR_MANUAL_STATE != fan->state) { // In CTR_MANUAL_STATE there is no min
      ctr_out = fan->controller.diff_eq.minOutputValue;
    }
  }

  *fan->io.ctrVal = ctr_out;
}
