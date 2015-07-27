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

#define TEMPORARY

/* ---------------------------------------------------------------------------*/
/* Fan handling */
/* ---------------------------------------------------------------------------*/
void init_fan(fan_t * fan)
{
	fan_init_feedback_ctr(&fan->controller);
}

void fan_init_feedback_ctr(controller_t * controller)
{
  controller->diff_eq.N0 = 6.001500E0;
  controller->diff_eq.N1 = -5.998500E0;
  controller->diff_eq.D1 = -1.000000E0;
}

void fan_controller(fan_t *fan)
{
  fan->controller.setPoint = 2600; //<< ToDo: read from calib
  int64_t ctr_out = 0;

  switch (fan->state) {
    case STOP_STATE:
    {
      *fan->io.ctrVal = 0;
      fan->state = CTRL_CLOSED_LOOP_STATE; //Starts when power on
    }
    break;
    case MANUAL_STATE:
    {
      return; // No action in manuel state
    } 
    break;
    case CTRL_OPEN_LOOP_STATE:
    {
      fan->state = CTRL_CLOSED_LOOP_STATE;
    } 
    break;
    case CTRL_CLOSED_LOOP_STATE:
    {
    	ctr_out = feedback_controller(&fan->controller, *fan->io.adcVal);
    }
    break;
    default:
    break;
  }

  if (ctr_out > PWM_UPPER_LIMIT)
    { ctr_out = PWM_UPPER_LIMIT; }
  if (ctr_out < 15000)
    { ctr_out = 15000; }
  *fan->io.ctrVal = ctr_out;

#ifdef TEMPORARY
  *fan->io.ctrVal = 17000; //ToDo: remove.
#endif
}
