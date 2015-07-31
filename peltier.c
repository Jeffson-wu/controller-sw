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

//#define TEMPORARY

/* ---------------------------------------------------------------------------*/
/* Peltier handling */
/* ---------------------------------------------------------------------------*/
void init_peltier(peltier_t * peltier)
{
	peltier_init_feedback_ctr(&peltier->controller);
}

void peltier_init_feedback_ctr(controller_t * controller)
{
  controller->diff_eq.N0 = 6.001500E0;
  controller->diff_eq.N1 = -5.998500E0;
  controller->diff_eq.D1 = -1.000000E0;
}

void peltier_controller(peltier_t *peltier)
{

  int64_t ctr_out = 0;

  switch (peltier->state) {
    case CTR_STOP_STATE:
    {
      *peltier->io.ctrVal = 0;
      peltier->state = CTR_CLOSED_LOOP_STATE; //Starts when power on
      //peltier->controller.diff_eq.input = 0;
    }
    break;
    case CTR_MANUAL_STATE:
    {
      return; // No action in manuel state
    } 
    break;
    case CTR_OPEN_LOOP_STATE:
    {
    	ctr_out = (double)peltier->setPoint;
    } 
    break;
    case CTR_CLOSED_LOOP_STATE:
    {
    	peltier->setPoint = 1600; // move!!
      peltier->controller.setPoint = rate_limiter(&peltier->rateLimiter, (double)peltier->setPoint);
      
      ctr_out = feedback_controller(&peltier->controller, *peltier->io.adcVal);
    }
    break;
    default:
    break;
  }
#if 1
  if (ctr_out > DAC_UPPER_LIMIT)
    { ctr_out = DAC_UPPER_LIMIT; }
  if (ctr_out < 0)
    { ctr_out = 0; }
#endif
  *peltier->io.ctrVal = ctr_out;

#ifdef TEMPORARY
  *peltier->io.ctrVal = 1100; //ToDo: remove.
#endif
}
