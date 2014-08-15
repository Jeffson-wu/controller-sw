/*
 * pwm.h
 *
 *  Created on: Okt 3, 2013
 *      Author: Tommy Kristensen <tfk@xtel.dk>
 */

#ifndef PWM_H_
#define PWM_H_

#include "stm32f10x.h"
#ifdef REV_2
typedef enum {
	
	TopHeaterCtrl1PWM,/* PB8 TIM4_CH3*/
	FANctrlPWM,      /* PB9 TIM4_CH4*/
	PeltierCtrlPWM1, /* PC6 TIM3_CH1*/
	TopHeaterCtrl2PWM, /* PC7 TIM3_CH2*/
	PeltierCtrlPWM3, /* PC8 TIM3_CH3*/
	nPWMS
} PWMPort_t;
#else
typedef enum {
	
	TopHeaterCtrlPWM,/* PB8 TIM4_CH3*/
	FANctrlPWM,      /* PB9 TIM4_CH4*/
	PeltierCtrlPWM1, /* PC6 TIM3_CH1*/
	PeltierCtrlPWM2, /* PC7 TIM3_CH2*/
	PeltierCtrlPWM3, /* PC8 TIM3_CH3*/
	nPWMS
} PWMPort_t;
#endif
extern void PWM_Init(uint32_t pwm_freq_TIM3,uint32_t pwm_freq_TIM4);
extern void PWM_Set(uint16_t pwm_width,PWMPort_t pwm_port);
extern void PWM_Test(void);

/* LIST4: TIM 1, 2, 3, 14, 15, 16 and 17 */
#define IS_TIM_PMW_RANGE_VALID(PERIPH) (((PERIPH) == TopHeaterCtrlPWM) || \
                                     ((PERIPH) == FANctrlPWM) || \
                                     ((PERIPH) == PeltierCtrlPWM1) || \
									 ((PERIPH) == PeltierCtrlPWM2) || \
                                     ((PERIPH) == PeltierCtrlPWM3) )



#endif /* PWM_H_ */
