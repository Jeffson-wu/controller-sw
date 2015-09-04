/**
  ******************************************************************************
  * @file    peltier.h
  * @author  Jeppe Soendergaard Larsen <jsl@xtel.dk>
  * @version V1.0.0
  * @date    24-Aug-2015
  * @brief   Peltier Control
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
#include "peltier.h"

/* ---------------------------------------------------------------------------*/
/* Peltier handling */
/* ---------------------------------------------------------------------------*/
void init_peltier(peltier_t * peltier)
{
  peltier_init_feedback_ctr(&peltier->controller);
  peltier_init_rate_limiter(&peltier->rateLimiter);
  init_median_filter(&peltier->medianFilter);
  peltier_init_adc_to_temp(&peltier->ntcCoef);
  peltier->controller.setPoint = peltier->setPoint = MAX_DAC_VALUE;
}

void peltier_init_feedback_ctr(controller_t * controller)
{
  controller->diff_eq.N0 = 1.005000E1;
  controller->diff_eq.N1 = -9.950000E0;
  controller->diff_eq.D1 = -1.000000E0;

  controller->diff_eq.minOutputValue = 0;
  controller->diff_eq.maxOutputValue = DAC_UPPER_LIMIT;
}

void peltier_init_rate_limiter(rateLimiter_t * rateLimiter)
{
  rateLimiter->slewRateRising = 4;
  rateLimiter->slewRateFaling = -4;
}

void peltier_setpoint(peltier_t * peltier, int16_t value)
{
  peltier->setPoint = temp_to_adc(&peltier->ntcCoef, value);
}

void peltier_init_adc_to_temp(ntcCoef_t * ntcCoef)
{
  ntcCoef->beta = 3984;
  ntcCoef->r_s_ohm = 10000;
}

void peltier_controller(peltier_t *peltier)
{
  uint16_t ctr_out = 0;

  peltier->adcValFilt = median_filter(&peltier->medianFilter, *peltier->io.adcVal);

  switch (peltier->state) {
    case CTR_STOP_STATE:
    {
      *peltier->io.ctrVal = 0;
    }
    break;
    case CTR_INIT:
    {
      reset_controller(&peltier->controller);
      reset_rateLimiter(&peltier->rateLimiter, *peltier->io.adcVal);
      peltier[0].state = CTR_CLOSED_LOOP_STATE;
    }
    break;
    case CTR_MANUAL_STATE:
    {
      ctr_out = (uint16_t)peltier->setPoint;
    }
    break;
    case CTR_OPEN_LOOP_STATE:
    {
    	return;
    }
    break;
    case CTR_CLOSED_LOOP_STATE:
    {
      peltier->controller.setPoint = rate_limiter(&peltier->rateLimiter, (double)peltier->setPoint);
      ctr_out = (uint16_t)feedback_controller_neg(&peltier->controller, peltier->adcValFilt);
    }
    break;
    default:
    break;
  }

  if (ctr_out > peltier->controller.diff_eq.maxOutputValue)
  {
    ctr_out = peltier->controller.diff_eq.maxOutputValue;
  }
  else if (ctr_out < peltier->controller.diff_eq.minOutputValue)
  {
    ctr_out = peltier->controller.diff_eq.minOutputValue;
  }

  *peltier->io.ctrVal = ctr_out;
}
