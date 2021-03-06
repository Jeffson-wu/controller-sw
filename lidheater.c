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
//#include "sequencer.h"
//#include "dll_dummy.h"
#include "stdio.h"
#include "debug.h"
/* ---------------------------------------------------------------------------*/
/* Lid handling */
/* ---------------------------------------------------------------------------*/
void init_lid_heater(lidHeater_t * lidHeater)
{
  lid_heater_init_feedback_ctr(&lidHeater->controller);
  lid_heater_init_rate_limiter(&lidHeater->rateLimiter);
  init_median_filter(&lidHeater->medianFilter);
  //lid_heater_init_ntc_coef(&lidHeater->ntcCoef);
  lidHeater->controller.setPoint = lidHeater->setPoint = 0;
  //lidHeater->error = FALSE;
}

void init_mid_heater(lidHeater_t * lidHeater)
{
  mid_heater_init_feedback_ctr(&lidHeater->controller);
  lid_heater_init_rate_limiter(&lidHeater->rateLimiter);
  init_median_filter(&lidHeater->medianFilter);
  //lid_heater_init_ntc_coef(&lidHeater->ntcCoef);
  lidHeater->controller.setPoint = lidHeater->setPoint = 0;
  //lidHeater->error = FALSE;
}

void lid_heater_init_feedback_ctr(controller_t * controller)
{
  controller->diff_eq.N0 = 3.000500E2;
  controller->diff_eq.N1 = -2.999500E2;
  controller->diff_eq.D1 = -1.000000E0;

  controller->diff_eq.minOutputValue = 0;
  //The topheater element melts down if we diserpate more power than this.
  //controller->diff_eq.maxOutputValue = (PWM_UPPER_LIMIT * 55) / 100;
  controller->diff_eq.maxOutputValue = (PWM_UPPER_LIMIT * 100) / 100;
}

void mid_heater_init_feedback_ctr(controller_t * controller)
{
  controller->diff_eq.N0 = 1.001E2;
  controller->diff_eq.N1 = -9.99E1;
  controller->diff_eq.D1 = -1.000000E0;

  controller->diff_eq.minOutputValue = 0;
  //The topheater element melts down if we diserpate more power than this.
  //controller->diff_eq.maxOutputValue = (PWM_UPPER_LIMIT * 55) / 100;
  controller->diff_eq.maxOutputValue = (PWM_UPPER_LIMIT * 100) / 100;
}

void lid_heater_init_rate_limiter(rateLimiter_t * rateLimiter)
{
  rateLimiter->slewRateRising = 0;
  rateLimiter->slewRateFaling = -0;
}

void lid_heater_setpoint(lidHeater_t * lidHeater, int16_t value)
{
  lidHeater->setPoint = lidHeater->controller.setPoint = temp_to_adc(&lidHeater->ntcCoef, value);
  lidHeater->setPointLow = temp_to_adc(&lidHeater->ntcCoef, (value-(int16_t)((double)value*0.05)));
  //lidHeater->setPointLow_1 = temp_to_adc(&lidHeater->ntcCoef, (value-(int16_t)((double)value*0.08)));
}

void lid_heater_init_ntc_coef(ntcCoef_t * ntcCoef)
{ 
  //ntcCoef->beta = 3940;
  //ntcCoef->r_s_ohm = 470;

  //ntcCoef->beta = 3984;
  //ntcCoef->r_s_ohm = 10000;
}

int16_t lid_heater_power(lidHeater_t *lidHeater)
{
  return lidHeater->controller.diff_eq.output/PWM_UPPER_LIMIT*97; //Watt*10
}

int16_t mid_heater_power(lidHeater_t *lidHeater)
{
  return lidHeater->controller.diff_eq.output/PWM_UPPER_LIMIT*74; //Watt*10
}

void lid_heater_controller(lidHeater_t *lidHeater)
{
  uint16_t ctr_out = 0;
  //int8_t i;

  lidHeater->adcValFilt = median_filter(&lidHeater->medianFilter, *lidHeater->io.adcVal);

  switch (lidHeater->state) {
    case CTR_STOP_STATE:
    {
      *lidHeater->io.ctrVal = 0;
    } 
    break;
    case CTR_INIT:
    {
      reset_controller(&lidHeater->controller);
      reset_rateLimiter(&lidHeater->rateLimiter, *lidHeater->io.adcVal);
      lidHeater->state = CTR_OPEN_LOOP_STATE;
    }
    break;
    case CTR_MANUAL_STATE:
    {
      ctr_out = (uint16_t)lidHeater->controller.setPoint;
    } 
    break;
    case CTR_OPEN_LOOP_STATE:
    {
      ctr_out = PWM_UPPER_LIMIT;
      if (*lidHeater->io.adcVal > lidHeater->setPointLow)
      {
      	lidHeater->controller.diff_eq.output = lidHeater->controller.diff_eq.input = ctr_out;
		lidHeater->state = CTR_CLOSED_LOOP_STATE;
      }
    }
    break;
    case CTR_CLOSED_LOOP_STATE:
    {

      ctr_out = (uint16_t)feedback_controller_pos(&lidHeater->controller, lidHeater->adcValFilt);

//int16_t temp=adc_to_temp(&lidHeater->ntcCoef,lidHeater->adcValFilt);
//PRINTF("CTR%d: %d %d %f %d %d\n",
//       lidHeater->lidHeaterID,ctr_out,lidHeater->adcValFilt,
//       lidHeater->controller.setPoint,temp,
//       temp_to_adc(&lidHeater->ntcCoef,temp));
      /*
      if (*lidHeater->io.adcVal < lidHeater->setPointLow_1)
			{
				lidHeater->state = CTR_OPEN_LOOP_STATE;
			}
      */
    }
    break;
    default:
    break;
  }

  if (ctr_out > lidHeater->controller.diff_eq.maxOutputValue)
  {
    ctr_out = lidHeater->controller.diff_eq.maxOutputValue;
  }
  else if (ctr_out < lidHeater->controller.diff_eq.minOutputValue)
  {
    ctr_out = lidHeater->controller.diff_eq.minOutputValue;
  }

#if 0
  if (lidHeater->state == CTR_CLOSED_LOOP_STATE)
  {
		if (lidHeater->adcValFilt > lidHeater->max_adc || lidHeater->adcValFilt < lidHeater->min_adc)
		{
			/*
			for (i=0;i<16; i++)
			{
				stop_tube_seq(i+1);
			}
			*/
			lidHeater->state = CTR_STOP_STATE;
			lidHeater->error = TRUE;
		}
  }
#endif
  *lidHeater->io.ctrVal = ctr_out;
}
