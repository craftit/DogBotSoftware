
#include <stdint.h>

#include "hal.h"
#include "pwm.h"

#define SYSTEM_CORE_CLOCK               168000000


/**
 * @brief   Configures and activates the PWM peripheral.
 * @note    Starting a driver that is already in the @p PWM_READY state
 *          disables all the active channels.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
//#include <stm32.h>
int InitPWM(void)
{
  rccEnableTIM1(FALSE);
  rccResetTIM1();
  //nvicEnableVector(STM32_TIM1_UP_NUMBER, STM32_PWM_TIM1_IRQ_PRIORITY);
  //nvicEnableVector(STM32_TIM1_CC_NUMBER, STM32_PWM_TIM1_IRQ_PRIORITY);
  //pwmp->clock = STM32_TIMCLK2;

  stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;
  /* All channels configured in PWM1 mode with preload enabled and will
       stay that way until the driver is stopped.*/

  tim->CCMR1 =
      STM32_TIM_CCMR1_OC1M(6) |
      STM32_TIM_CCMR1_OC1PE |
      STM32_TIM_CCMR1_OC2M(6) |
      STM32_TIM_CCMR1_OC2PE;
  tim->CCMR2 =
      STM32_TIM_CCMR2_OC3M(6) |
      STM32_TIM_CCMR2_OC3PE |
      STM32_TIM_CCMR2_OC4M(6) |
      STM32_TIM_CCMR2_OC4PE;
  uint16_t psc = (SYSTEM_CORE_CLOCK / 10000000) - 1;
  tim->PSC  = psc;
  tim->ARR  = 10000 - 1;
  tim->CR2  = 0;

  /* Output enables and polarities setup.*/
  uint16_t ccer = 0;
  ccer |= STM32_TIM_CCER_CC1E; // Active high
  //ccer |= STM32_TIM_CCER_CC1P;
  ccer |= STM32_TIM_CCER_CC2E; // Active high
  //ccer |= STM32_TIM_CCER_CC2P;
  ccer |= STM32_TIM_CCER_CC3E;


  tim->CCER  = ccer;
  tim->EGR   = STM32_TIM_EGR_UG;      /* Update event.                */
  tim->SR    = 0;                     /* Clear pending IRQs.          */
  tim->DIER  = 0;
  tim->BDTR  = STM32_TIM_BDTR_MOE;

  /* Timer configured and started.*/
  tim->CR1   = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN | STM32_TIM_CR1_CMS(3);


  tim->CCR[0] = 7500;
  tim->CCR[1] = 5000;
  tim->CCR[2] = 2500;

  return 0;
}

