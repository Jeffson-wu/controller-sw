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

#define TEMPORARY

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
  peltier->controller.setPoint = 1600; //??oC  //<< ToDo: read from calib
  int64_t ctr_out = 0;

  switch (peltier->state) {
    case STOP_STATE:
    {
      *peltier->io.ctrVal = 0;
      peltier->state = CTRL_CLOSED_LOOP_STATE; //Starts when power on
    }
    break;
    case MANUAL_STATE:
    {
      return; // No action in manuel state
    } 
    break;
    case CTRL_OPEN_LOOP_STATE:
    {
      peltier->state = CTRL_CLOSED_LOOP_STATE;
    } 
    break;
    case CTRL_CLOSED_LOOP_STATE:
    {
      ctr_out = feedback_controller(&peltier->controller, *peltier->io.adcVal);
    }
    break;
    default:
    break;
  }

  if (ctr_out > DAC_UPPER_LIMIT)
    { ctr_out = DAC_UPPER_LIMIT; }
  if (ctr_out < 0)
    { ctr_out = 0; }

  *peltier->io.ctrVal = ctr_out;

#ifdef TEMPORARY
  *peltier->io.ctrVal = 1100; //ToDo: remove.
#endif
}
