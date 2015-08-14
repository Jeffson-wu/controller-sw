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
#define TEMPORARY

/* Includes ------------------------------------------------------------------*/
#include "fan.h"

/* ---------------------------------------------------------------------------*/
/* Fan handling */
/* ---------------------------------------------------------------------------*/
void init_fan(fan_t * fan)
{
	fan_init_feedback_ctr(&fan->controller);
  fan_init_rate_limiter(&fan->rateLimiter);
  fan_init_adc_to_temp(&fan->ntcCoef);
  fan->controller.setPoint = fan->setPoint = 0;
}

void fan_init_feedback_ctr(controller_t * controller)
{
  controller->diff_eq.N0 = 6.001500E0;
  controller->diff_eq.N1 = -5.998500E0;
  controller->diff_eq.D1 = -1.000000E0;
}

void fan_init_rate_limiter(rateLimiter_t * rateLimiter)
{
  rateLimiter->slewRateRising = 2;
  rateLimiter->slewRateFaling = -2;
}

void fan_setpoint(fan_t *fan, int16_t value)
{
  fan->setPoint = temp_to_adc(&fan->ntcCoef, value);
}

void fan_init_adc_to_temp(ntcCoef_t * ntcCoef)
{
  ntcCoef->r_s_ohm = 10000;
}

void fan_controller(fan_t *fan)
{
  int64_t ctr_out = 0;

  switch (fan->state) {
    case CTR_STOP_STATE:
    {
      *fan->io.ctrVal = 0;
    	fan->controller.setPoint = 0;
      //fan->state = CTR_CLOSED_LOOP_STATE; //Starts when power on
    }
    break;
    case CTR_MANUAL_STATE:
    {
    	ctr_out = (double)fan->controller.setPoint;
    } 
    break;
    case CTR_OPEN_LOOP_STATE:
    {
    	return;
    } 
    break;
    case CTR_CLOSED_LOOP_STATE:
    {
      fan->controller.setPoint = 2600; //<< ToDo: read from calib

		#ifdef TEMPORARY
    	ctr_out = 17000; //ToDo: remove.
		#else
    	ctr_out = feedback_controller(&fan->controller, *fan->io.adcVal);
		#endif
    }
    break;
    default:
    break;
  }

  if (ctr_out > 24000) // fan whistles above 25000
    { ctr_out = 24000; }
  if (ctr_out < 15000) // fan stops below
    { ctr_out = 15000; }
  *fan->io.ctrVal = ctr_out;
}
