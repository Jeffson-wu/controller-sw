/**
  ******************************************************************************
  * @file    lidheater.c
  * @author  Jeppe Soendergaard Larsen <jsl@xtel.dk>
  * @version V1.0.0
  * @date    24-Aug-2015
  * @brief   Lid control
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
#include "lidheater.h"

/* ---------------------------------------------------------------------------*/
/* Lid handling */
/* ---------------------------------------------------------------------------*/
void init_lid_heater(lidHeater_t * lidHeater)
{
  lid_heater_init_feedback_ctr(&lidHeater->controller);
  lid_heater_init_rate_limiter(&lidHeater->rateLimiter);
  lid_heater_init_ntc_coef(&lidHeater->ntcCoef);
  lidHeater->controller.setPoint = lidHeater->setPoint = 0;
}

void lid_heater_init_feedback_ctr(controller_t * controller)
{
  controller->diff_eq.N0 = 3.000500E2;
  controller->diff_eq.N1 = -2.999500E2;
  controller->diff_eq.D1 = -1.000000E0;
}

void lid_heater_init_rate_limiter(rateLimiter_t * rateLimiter)
{
  rateLimiter->slewRateRising = 2;
  rateLimiter->slewRateFaling = -2;
}

void lid_heater_setpoint(lidHeater_t * lidHeater, int16_t value)
{
  lidHeater->setPoint = temp_to_adc(&lidHeater->ntcCoef, value);
  lidHeater->setPointLow = temp_to_adc(&lidHeater->ntcCoef, (value-value*0.05));
}

void lid_heater_init_ntc_coef(ntcCoef_t * ntcCoef)
{ 
  ntcCoef->beta = 3940;
  ntcCoef->r_s_ohm = 470;
}

void lid_heater_controller(lidHeater_t *lidHeater)
{
  int64_t ctr_out = 0;

  lidHeater->adcValFilt = median_filter(&lidHeater->filter, *lidHeater->io.adcVal);

  switch (lidHeater->state) {
    case CTR_STOP_STATE:
    {
      *lidHeater->io.ctrVal = 0;
      reset_controller(&lidHeater->controller);
      reset_rateLimiter(&lidHeater->rateLimiter, *lidHeater->io.adcVal);
    } 
    break;
    case CTR_MANUAL_STATE:
    {
    	ctr_out = (double)lidHeater->controller.setPoint;
      return;
    } 
    break;
    case CTR_OPEN_LOOP_STATE:
    {
      ctr_out = PWM_UPPER_LIMIT;
    	if (*lidHeater->io.adcVal > lidHeater->setPointLow)
    	{
    		lidHeater->state = CTR_CLOSED_LOOP_STATE;
    	}
    }
    break;
    case CTR_CLOSED_LOOP_STATE:
    {
      lidHeater->controller.setPoint = (double)lidHeater->setPoint;
      ctr_out = feedback_controller(&lidHeater->controller, lidHeater->adcValFilt);
    }
    break;
    default:
    break;
  }

  if (ctr_out > PWM_UPPER_LIMIT)
  {
    ctr_out = PWM_UPPER_LIMIT;
  }
  else if (ctr_out < 0)
  {
    ctr_out = 0;
  }
  *lidHeater->io.ctrVal = ctr_out;
}
