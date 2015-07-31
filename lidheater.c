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

#define TEMPORARY

/* ---------------------------------------------------------------------------*/
/* Lid handling */
/* ---------------------------------------------------------------------------*/
void init_lid_heater(lidHeater_t * lidHeater)
{
	lid_heater_init_feedback_ctr(&lidHeater->controller);
}

void lid_heater_init_feedback_ctr(controller_t * controller)
{
  controller->diff_eq.N0 = 6.001500E0;
  controller->diff_eq.N1 = -5.998500E0;
  controller->diff_eq.D1 = -1.000000E0;
}

void lid_heater_controller(lidHeater_t *lidHeater)
{
  lidHeater->controller.setPoint = 1647; // 470ohm series resistor this should be 100oC  //<< ToDo: read from calib
  int64_t ctr_out = 0;
#ifdef TEMPORARY
  int16_t Kp = 45; //1.5;
#endif

  switch (lidHeater->state) {
    case STOP_STATE:
    {
      *lidHeater->io.ctrVal = 0;
      lidHeater->state = CTRL_CLOSED_LOOP_STATE; // << remove
    } 
    break;
    case MANUAL_STATE:
    {
      return; // No action in manuel state
    } 
    break;
    case CTRL_OPEN_LOOP_STATE:
    {
      ctr_out = 32767;

      if (*lidHeater->io.adcVal > (lidHeater->controller.setPoint-40)) //ca -10oC
      	lidHeater->state = CTRL_CLOSED_LOOP_STATE;
    }
    break;
    case CTRL_CLOSED_LOOP_STATE:
    {
#ifdef TEMPORARY
    	ctr_out = Kp*(lidHeater->controller.setPoint - *lidHeater->io.adcVal);
#else
    	ctr_out = feedback_controller(&lidHeater->controller, *lidHeater->io.adcVal);
#endif
    }
    break;
    default:
    break;
  }

  if (ctr_out > PWM_UPPER_LIMIT)
    { ctr_out = PWM_UPPER_LIMIT; }
  if (ctr_out)
    { ctr_out = 0; }
  *lidHeater->io.ctrVal = ctr_out;

#ifdef TEMPORARY
  *lidHeater->io.ctrVal = 1100; //ToDo: remove.
#endif
}
