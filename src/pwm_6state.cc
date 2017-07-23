enum PinStateT {
  LOW,
  HIGH,
  PWM,
  NPWM
} ;

struct ComStateC  {
  enum PinStateT m_pinState[6];
  int m_ccer;
  int m_ccmr1;
  int m_ccmr2;
} ;

#if 1
struct ComStateC g_commutationSequence[] = {
    {{  PWM, NPWM, LOW, HIGH, LOW,  LOW }, 0,0,0 }, //  0 AB
    {{  PWM, NPWM, LOW, HIGH, PWM, NPWM }, 0,0,0 }, //  1 AB_CB
    {{  LOW,  LOW, LOW, HIGH, PWM, NPWM }, 0,0,0 }, //  2 CB
    {{  LOW, HIGH, LOW, HIGH, PWM, NPWM }, 0,0,0 }, //  3 CB_CA
    {{  LOW, HIGH, LOW,  LOW, PWM, NPWM }, 0,0,0 }, //  4 CA
    {{  LOW, HIGH, PWM, NPWM, PWM, NPWM }, 0,0,0 }, //  5 CA_BA
    {{  LOW, HIGH, PWM, NPWM, LOW,  LOW }, 0,0,0 }, //  6 BA
    {{  LOW, HIGH, PWM, NPWM, LOW, HIGH }, 0,0,0 }, //  7 BA_BC
    {{  LOW,  LOW, PWM, NPWM, LOW, HIGH }, 0,0,0 }, //  8 BC
    {{  PWM, NPWM, PWM, NPWM, LOW, HIGH }, 0,0,0 }, //  9 BC_AC
    {{  PWM, NPWM, LOW,  LOW, LOW, HIGH }, 0,0,0 }, // 10 AC
    {{  PWM, NPWM, LOW, HIGH, LOW, HIGH }, 0,0,0 }, // 11 AC_AB
    {{  LOW,  LOW, LOW,  LOW, LOW,  LOW }, 0,0,0 }, // 12 STOP
};
#else
struct ComStateC g_commutationSequence[] = {
    {{  PWM, NPWM, LOW, HIGH, LOW,  LOW }, 0,0,0 }, //  0 AB
    {{  LOW,  LOW, LOW, HIGH, PWM, NPWM }, 0,0,0 }, //  1 CB
    {{  LOW, HIGH, LOW,  LOW, PWM, NPWM }, 0,0,0 }, //  2 CA
    {{  LOW, HIGH, PWM, NPWM, LOW,  LOW }, 0,0,0 }, //  3 BA
    {{  LOW,  LOW, PWM, NPWM, LOW, HIGH }, 0,0,0 }, //  4 BC
    {{  PWM, NPWM, LOW,  LOW, LOW, HIGH }, 0,0,0 }, //  5 AC
    {{  LOW,  LOW, LOW,  LOW, LOW,  LOW }, 0,0,0 }, //  6 STOP
};
#endif

static int PWMMode(enum PinStateT ps)
{
  switch(ps)
  {
  case LOW:
  case HIGH:
    return 4;
  case PWM:
  case NPWM:
    return 6;
  }
  return 0;
}

int InitPWMSimple(void)
{
  g_drivePhase = 0;

  // Generate the register setup for each phase.

  for(int i = 0;i < 13;i++) {
    struct ComStateC *cs = &g_commutationSequence[i];

    cs->m_ccmr1 =
        STM32_TIM_CCMR1_OC1M(PWMMode(cs->m_pinState[0])) |
        STM32_TIM_CCMR1_OC1PE |
        STM32_TIM_CCMR1_OC2M(PWMMode(cs->m_pinState[2])) |
        STM32_TIM_CCMR1_OC2PE;
    cs->m_ccmr2 =
        STM32_TIM_CCMR2_OC3M(PWMMode(cs->m_pinState[4])) |
        STM32_TIM_CCMR2_OC3PE |
        STM32_TIM_CCMR2_OC4M(6) | // PWM Mode
        STM32_TIM_CCMR2_OC4PE;

    int ccer = 0;
    for(int i = 0;i < 6;i++) {
      int nv = 0;
      nv |= STM32_TIM_CCER_CC1E;
      if(i & 1) {
        if((cs->m_pinState[i] == NPWM || cs->m_pinState[i] == HIGH))
          nv |= STM32_TIM_CCER_CC1P;
      } else {
        if((cs->m_pinState[i] == PWM || cs->m_pinState[i] == LOW))
          nv |= STM32_TIM_CCER_CC1P;
      }
      ccer |= nv << (i * 2);
    }
    ccer |= STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC1P;
    cs->m_ccer = ccer;
  }


  rccEnableTIM1(FALSE);
  rccResetTIM1();
  //nvicEnableVector(STM32_TIM1_UP_NUMBER, STM32_PWM_TIM1_IRQ_PRIORITY);
  //nvicEnableVector(STM32_TIM1_CC_NUMBER, STM32_PWM_TIM1_IRQ_PRIORITY);
  //pwmp->clock = STM32_TIMCLK2;

  stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;

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

  uint16_t psc = 0; // (SYSTEM_CORE_CLOCK / 80000000) - 1;
  tim->PSC  = psc;
  tim->ARR  = TIM_1_8_PERIOD_CLOCKS; // This should give about 20KHz
  tim->CR2  = 0;
  /* Output enables and polarities setup.*/
  uint16_t ccer = 0;
#if 0
  ccer |= STM32_TIM_CCER_CC1E; // Active high
  //ccer |= STM32_TIM_CCER_CC1P;
  ccer |= STM32_TIM_CCER_CC2E; // Active high
  //ccer |= STM32_TIM_CCER_CC2P;
  ccer |= STM32_TIM_CCER_CC3E;
#endif

  tim->CCER  = ccer;
  tim->EGR   = STM32_TIM_EGR_UG;      /* Update event.                */
  tim->SR    = 0;                     /* Clear pending IRQs.          */
  tim->DIER  = 0;
  tim->BDTR  = STM32_TIM_BDTR_MOE;
  //| STM32_TIM_BDTR_OSSR;

  /* Timer configured and started.*/
  tim->CR1   = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN | STM32_TIM_CR1_CMS(3);

  tim->CCR[0] = 200;
  tim->CCR[1] = 200;
  tim->CCR[2] = 200;
  tim->CCR[3] = TIM_1_8_PERIOD_CLOCKS - 16;

  tim->CR2  = STM32_TIM_CR2_CCPC | STM32_TIM_CR2_MMS(7); // Use the COMG bit to update.

  palSetPad(GPIOC, GPIOC_PIN13); // Wake

  InitPWM_ADC();

#if 0
  // This is quick, so may as well do it every time.
  ShuntCalibration();

#endif
  //palSetPad(GPIOB, GPIOB_PIN12); // Turn on flag pin

  return 0;
}


int PWMCal(BaseSequentialStream *chp)
{
  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  for(int phase = 0;phase < 12;phase++) {
    PWMUpdateDrive(phase,200);
    chThdSleepMilliseconds(1000);

    // Sync to avoid reading variables when they're being updated.
    chBSemWait(&g_adcInjectedDataReady);

    g_phaseAngles[phase][0] = g_hall[0];
    g_phaseAngles[phase][1] = g_hall[1];
    g_phaseAngles[phase][2] = g_hall[2];

    chprintf(chp, "Cal %d : %04d %04d %04d   \r\n",phase,g_hall[0],g_hall[1],g_hall[2]);
  }

  PWMUpdateDrive(12,0);

  palClearPad(GPIOC, GPIOC_PIN14); // Gate disable
  DisplayAngle(chp);
  return 0;
}

void PWMUpdateDrive(int phase,int power)
{
#if 1
  if(phase < 0 || phase > 12)
    phase = 12;
  stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;
  struct ComStateC *cs = &g_commutationSequence[phase];
  tim->CCER  = cs->m_ccer;
  tim->CCMR1 = cs->m_ccmr1;
  tim->CCMR2 = cs->m_ccmr2;
  tim->CCR[0] = power;
  tim->CCR[1] = power;
  tim->CCR[2] = power;

  tim->EGR = STM32_TIM_EGR_COMG;
#endif
}


static THD_WORKING_AREA(waThreadSimplePWM, 128);


static THD_FUNCTION(ThreadSimplePWM, arg) {

  (void)arg;
  chRegSetThreadName("pwm");

  // Start pwm

  PWMUpdateDrivePhase(TIM_1_8_PERIOD_CLOCKS/2,TIM_1_8_PERIOD_CLOCKS/2,TIM_1_8_PERIOD_CLOCKS/2);

  // This is quick, so may as well do it every time.
  ShuntCalibration();

#if 0
  control_motor_loop();
  //scan_motor_loop(2.0,0.1);
#else
  //int phase = 0;
  while (g_pwmRun) {
#if 0
    PWMUpdateDrive(phase,200);
    phase += 1;
    if(phase >= 12) phase = 0;
#else
    //palClearPad(GPIOB, GPIOB_PIN12); // off
    chBSemWait(&g_adcInjectedDataReady);
    //palSetPad(GPIOB, GPIOB_PIN12); // on

    compute_state();

#if 1
#if 0
    angle += 5.5;
    if(angle < 0) angle += 24.0;
    if(angle > 24) angle -= 24.0;
    int phase = angle / 2;
    PWMUpdateDrive(phase,500);
#else
    float pangle = g_phaseAngle;
    pangle -= M_PI/2.0;
    float voltage_magnitude = 0.1;
    float v_alpha = voltage_magnitude * arm_cos_f32(pangle);
    float v_beta  = voltage_magnitude * arm_sin_f32(pangle);
    queue_modulation_timings(v_alpha, v_beta);
#endif
#else
    PWMUpdateDrivePhase(TIM_1_8_PERIOD_CLOCKS/2,TIM_1_8_PERIOD_CLOCKS/2,TIM_1_8_PERIOD_CLOCKS/2);
    //queue_modulation_timings(0.0, 0.0);
#endif

#endif
    if (!palReadPad(GPIOB, GPIOA_PIN2)) {
      break;
    }
  }
  PWMUpdateDrive(12,0);
#endif
  g_pwmThreadRunning = false;
}

