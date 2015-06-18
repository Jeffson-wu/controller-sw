/*
 * pwm.h
 *
 *  Created on: Okt 3, 2013
 *      Author: Tommy Kristensen <tfk@xtel.dk>
 */

#ifndef PWM_H_
#define PWM_H_

#include "stm32f10x.h"
typedef enum {
  PWM0_TIM4CH3, /* PB8 Timer 4 Channel 3*/
  PWM1_TIM4CH4, /* PB9 Timer 4 Channel 4*/
  PWM2_TIM3CH1, /* PC6 Timer 3 Channel 1*/
  PWM3_TIM3CH2, /* PC7 Timer 3 Channel 2*/
  PWM4_TIM3CH3, /* PC8 Timer 3 Channel 3*/
  nPWMS
} PWMPort_t;

void PWM_Init(uint32_t pwm_freq_TIM3,uint32_t pwm_freq_TIM4);
void PWM_Stop(void);
void PWM_Set(uint16_t pwm_width,PWMPort_t pwm_port);
void PWM_PinConfig(void);

/* LIST4: TIM 1, 2, 3, 14, 15, 16 and 17 */
#define IS_TIM_PMW_RANGE_VALID(PERIPH) (((PERIPH) == PWM0_TIM4CH3) || \
                                       ((PERIPH) == PWM1_TIM4CH4) || \
                                       ((PERIPH) == PWM2_TIM3CH1) || \
                                       ((PERIPH) == PWM3_TIM3CH2) || \
                                       ((PERIPH) == PWM4_TIM3CH3) )

#endif /* PWM_H_ */
