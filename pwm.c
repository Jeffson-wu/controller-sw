/*
 * serial.c
 *
 *  Created on: Okt 3, 2013
 *      Author: Tommy Kristensen <tfk@xtel.dk>
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
//#include "stm32f10x_misc.h"
#include "stm32f10x_tim.h"
#include "pwm.h"
#include "stm32f10x.h"


#define STM32F051
/* Private variables ---------------------------------------------------------*/
static TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
static TIM_OCInitTypeDef  TIM_OCInitStructure;
static uint16_t TimerPeriod_TIM3 = 0;
static uint16_t TimerPeriod_TIM4 = 0;

uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure the PWM module on TIM1 CH1-4 
  * @param  The wanted PWM freq that should be used for switching in Hz
  * @retval None
  */

void PWM_Init(uint32_t TIM3_pwm_freq, uint32_t TIM4_pwm_freq)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */

  /* TIM Configuration */
  
  /* TIM1 Configuration ---------------------------------------------------
   TIM1 input clock (TIM1CLK) is set to APB2 clock (PCLK2)    
    => TIM1CLK = PCLK2 = SystemCoreClock
   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   SystemCoreClock is set to 48 MHz for STM32F0xx devices
   
   The objective is to generate PWM signal at 500 KHz:
     - TIM1_Period = (SystemCoreClock / 17570) - 1
   Note: 
    SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
    Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
    function to update SystemCoreClock variable value. Otherwise, any configuration
    based on this variable will be incorrect. 
  ----------------------------------------------------------------------- */
  /* Compute the value to be set in ARR regiter to generate signal frequency at 500 Khz */
  TimerPeriod_TIM3 = (SystemCoreClock / TIM3_pwm_freq ) - 1;
  TimerPeriod_TIM4 = ((SystemCoreClock / 11) / TIM4_pwm_freq ) - 1; // TIM_Prescaler = 10 => divide SCC by 11

  /* TIM3 and TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod_TIM3;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 10;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod_TIM4;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Channel 1, 2, 3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_Pulse = 0;

  //TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

 // TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

 // TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

 // TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

 // TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  /* TIM3 counter enable */
  TIM_Cmd(TIM3, ENABLE);
  
  /* TIM4 counter enable */
  TIM_Cmd(TIM4, ENABLE);

}

void PWM_Stop(void)
{
  TIM_SetCompare3(TIM4,0);
  TIM_SetCompare4(TIM4,0);
  TIM_SetCompare1(TIM3,0);
  TIM_SetCompare2(TIM3,0);
  TIM_SetCompare3(TIM3,0);
}

/**
  * @brief  Set the dutycycle for a given PWM port in %
  * @param  None
  * @retval None
  */
void PWM_Set(uint16_t pwm_width,PWMPort_t pwm_port)
{
  uint16_t ChannelPulse;
  switch (pwm_port)
  {
    case PWM0_TIM4CH3:
      ChannelPulse = (uint16_t) (((uint32_t) pwm_width * (TimerPeriod_TIM4 - 1)) / 32768);
      TIM_SetCompare3(TIM4,ChannelPulse);
    break;
    case PWM1_TIM4CH4:
      ChannelPulse = (uint16_t) (((uint32_t) pwm_width * (TimerPeriod_TIM4 - 1)) / 32768);
      TIM_SetCompare4(TIM4,ChannelPulse);
    break;
    case PWM2_TIM3CH1:
      ChannelPulse = (uint16_t) (((uint32_t) pwm_width * (TimerPeriod_TIM3 - 1)) / 32768);
      TIM_SetCompare1(TIM3,ChannelPulse);
    break;
    case PWM3_TIM3CH2:
      ChannelPulse = (uint16_t) (((uint32_t) pwm_width * (TimerPeriod_TIM3 - 1)) / 32768);
      TIM_SetCompare2(TIM3,ChannelPulse);
    break;
    case PWM4_TIM3CH3:
      ChannelPulse = (uint16_t) (((uint32_t) pwm_width * (TimerPeriod_TIM3 - 1)) / 32768);
      TIM_SetCompare3(TIM3,ChannelPulse);
    break;	
    default:
      assert_param(IS_TIM_PMW_RANGE_VALID(pwm_port));
    break;
  }
}

/**
  * @brief  Configure the GPIO Pins to PWM using TIM1
  * @param  None
  * @retval None
  */

void PWM_PinConfig(void)
{
  /*Setup for PWM*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE); //AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_FULLREMAP;

  //RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC;
  /* GPIOB-GPIOC Clocks enable */
  RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM4EN, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  /* GPIOB Configuration: Channel 3 and 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* GPIOC Configuration: Channel 1, 2 and 3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = /*GPIO_Pin_6 | now PeltierCtrl_EN */ GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
    
}


/**
  * @}
  */

/*******END OF FILE****/

