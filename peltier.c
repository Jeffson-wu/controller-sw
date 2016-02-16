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
#if (_MSC_VER)
#include "dll_dummy.h"
#else
#include "sequencer.h"
#endif
//#include	"gdi.h"

/* ---------------------------------------------------------------------------*/
/* Peltier handling */
/* ---------------------------------------------------------------------------*/
void init_peltier(peltier_t * peltier)
{
  peltier_init_feedback_ctr(&peltier->controller);
  peltier_init_rate_limiter(&peltier->rateLimiter);
  init_median_filter(&peltier->medianFilter);
  peltier_init_adc_to_temp(&peltier->ntcCoef);
  init_estfilter(&peltier->filter.lp_filter[0]);
  peltier->controller.setPoint = peltier->setPoint = MAX_DAC_VALUE;
  peltier->error = FALSE;
}


void peltier_init_feedback_ctr(controller_t * controller)
{
/*
  controller->diff_eq.N0 = 1.005000E1;
  controller->diff_eq.N1 = -9.950000E0;
  controller->diff_eq.D1 = -1.000000E0;
*/
  controller->diff_eq.N0 = 3.0005000E1;
  controller->diff_eq.N1 = -2.999500E1;
  controller->diff_eq.D1 = -1.000000E0;

  controller->diff_eq.minOutputValue = 0;
  controller->diff_eq.maxOutputValue = MAX_DAC_VALUE;
}

void peltier_init_rate_limiter(rateLimiter_t * rateLimiter)
{
  rateLimiter->slewRateRising = 0.5; //4;
  rateLimiter->slewRateFaling = -0.5; //-4;
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

void init_estfilter(lp_filter_t * lp_filter)
{
  lp_filter->kc = 1.0;
  lp_filter->wc = 0.1;
  init_lp_filter(lp_filter);
}

void init_lp_filter(lp_filter_t * lp_filter)
{
  double ticks = 10; //Hz
  lp_filter->diff_eq.N0 = lp_filter->kc*lp_filter->wc/(lp_filter->wc+(double)ticks);
  lp_filter->diff_eq.D1 = -(double)ticks/(lp_filter->wc+(double)ticks);
}

/*
#include <stdio.h>
#include "debug.h"
#define PRINTF(fmt, args...)      sprintf(dbgbuf, fmt, ## args);  send_msg_on_monitor(dbgbuf);
*/

void th_estmator(peltier_t *peltier, uint16_t ctr_out)
{
  double G = 0.291;
  double N = 127.0;
  double alpha = 0.0001012;
  double Kelvin = 273.15;
  //double Tc = ((adc_to_temp(&peltier->ntcCoef, peltier->adcValFilt)) / 10.0) + Kelvin;
  double Tc;
  Tc = peltier[0].temp/10.0 + Kelvin;
  double I = (double)(8.36/4095.00) * (double)ctr_out; //8.36 max Amp
  double phi_a = 4.15841584158E-06;
  double phi_b = -0.000215247524752;

  double t_hot_est = (((peltier->voltage/10.0)-(2*N*I*phi_b)/G+alpha*Tc*2*N) / ((2*N*I*phi_a)/G+alpha*2*N) - Kelvin) * 10; //10 deci oC
  if (t_hot_est <= 0)
  {
  	t_hot_est = 0;
  }
  peltier->t_hot_est = lp_filter(&peltier->filter.lp_filter[0], (double)(t_hot_est));

  //PRINTF("FAN: %d, %d\n", (int16_t)t_hot_est, (int16_t)peltier->t_hot_est);
}

void peltier_controller(peltier_t *peltier)
{
  uint16_t ctr_out = 0;
  peltier->adcValFilt = median_filter(&peltier->medianFilter, *peltier->io.adcVal);

  switch (peltier->state) {
    case CTR_STOP_STATE:
    {
      *peltier->io.ctrVal = 0;
      ctr_out = 0;
    }
    break;
    case CTR_INIT:
    {
    	//peltier->filter.lp_filter[0].diff_eq.input = peltier->filter.lp_filter[0].diff_eq.output = 0;
      //reset_lp_filter(&peltier->filter.lp_filter[0]);
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

  /*
  if (ctr_out > peltier->controller.diff_eq.maxOutputValue)
  {
    ctr_out = peltier->controller.diff_eq.maxOutputValue;
  }
  else if (ctr_out < peltier->controller.diff_eq.minOutputValue)
  {
    ctr_out = peltier->controller.diff_eq.minOutputValue;
  }
  */


  //double dT = 0;


  //dT = (double)(2*peltier->voltage) + (-0.04*(double)ctr_out-9.56);
  //peltier->controller.diff_eq.maxOutputValue = 22.9*dT+3160;

  //peltier->t_hot_est = adc_to_temp(&peltier->ntcCoef, peltier->adcValFilt) + (dT*10.0);

  th_estmator(peltier, ctr_out);
  
  int8_t i;
  if (peltier->adcValFilt > peltier->max_adc)
  {
	  //setCLStatusReg(0x0001);
	  for (i=0;i<16; i++)
	  {
		  stop_tube_seq(i+1);
		  //send_led_cmd(21, i+1); //SET_LED_OFF
	  }
	  peltier->state = CTR_STOP_STATE;
	  peltier->error = TRUE;
  }

  /*
  if (peltier->voltage < 100) //peltier voltage
  {
  	peltier->controller.diff_eq.output_old = ctr_out;
  }
  else
  {
  	ctr_out = peltier->controller.diff_eq.output = peltier->controller.diff_eq.output_old;
  }
  */

  *peltier->io.ctrVal = ctr_out;
}
