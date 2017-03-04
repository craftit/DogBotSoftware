
#include <stdint.h>
#include "stm32_tim.h"
#include "hal.h"

#if STM32_HAS_TIM1

#endif



void InitPWM(void)
{
  rccEnableTIM1(false);
  rccResetTIM1();

  // Time Base configuration

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned2; // compare flag when upcounting
  TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)m_conf->foc_f_sw;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 1; // Only generate update event on underflow

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

}

void ShutdownPWM(void)
{
  rccDisableTIM1(false);

}
