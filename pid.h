/**
  ******************************************************************************
  * @file    pid.h
  * @author  Jeppe Soendergaard Larsen
  * @version V1.0.0
  * @date    24-Aug-2015
  * @brief   Controller
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
  *
  ******************************************************************************
  */ 

  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include <math.h>

#define T_R_KEL 298.15
#define R_R_OHM 10000.00
#define MAX_DAC_VALUE 4095.00
#define DAC_UPPER_LIMIT 4095.00 //3725
#define PWM_UPPER_LIMIT 32767

#define MEDIAN_LENGTH 15

typedef enum {
  CTR_STOP_STATE,
  CTR_INIT,
  CTR_MANUAL_STATE,
  CTR_OPEN_LOOP_STATE,
  CTR_CLOSED_LOOP_STATE,
  nCTR_STATES
} controllerState_t;

typedef struct {
  uint16_t            *ctrVal;
  int16_t             *adcVal;
} io_t;

typedef enum {
  CTR_SELF, //--> CTR_FB
  nCTR
} controllerID_t;

typedef struct {
  double              input;
  double              output;
  double							N0;
  double							N1;
  double							D1;
  double							minOutputValue;
  double							maxOutputValue;
} diff_eq_t;

typedef struct {
  double              output;
  double              slewRateRising;
  double              slewRateFaling;
  double							gain;
  double							max;
  double							min;
} rateLimiter_t;

typedef struct {
  int16_t             beta;
  int16_t             r_s_ohm;
} ntcCoef_t;

typedef struct {
  diff_eq_t           diff_eq;
} prefilter_t;

typedef struct {
  double              setPoint;
  diff_eq_t           diff_eq;
} controller_t;

typedef struct MEDIAN_FILTER {
  int16_t samples[MEDIAN_LENGTH];
  uint8_t sortIdx[MEDIAN_LENGTH];
  uint8_t samplesIdx;
} medianFilter_t;

typedef struct {
  double              kc;
  double              wc;
  diff_eq_t           diff_eq;
} lp_filter_t;

typedef struct {
  lp_filter_t         lp_filter[1];
} filter_t;

/* Private define ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
double feedback_controller_neg(controller_t *controller, int16_t processValue);
double feedback_controller_pos(controller_t *controller, int16_t processValue);
double imc_pid(controller_t *controller, double input);
double first_order_difference_equation(controller_t *controller, double input);
double rate_limiter(rateLimiter_t *rateLimiter, double input);
void reset_controller(controller_t *controller);
void reset_rateLimiter(rateLimiter_t *rateLimiter, int16_t adc);
void init_median_filter(medianFilter_t *medianFilter);
int16_t median_filter(medianFilter_t *medianFilter, int16_t sample);
int16_t adc_to_temp(ntcCoef_t *ntcCoef, int16_t adc);
int16_t temp_to_adc(ntcCoef_t *ntcCoef, int16_t temp);
void reset_lp_filter(lp_filter_t *lp_filter);
double lp_filter(lp_filter_t *lp_filter, double input);

#endif /* __PID_H */
  /************************ (C) COPYRIGHT Xtel *****END OF FILE****/
