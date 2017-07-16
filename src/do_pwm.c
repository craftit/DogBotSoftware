
#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include <arm_math.h>

#include <stdint.h>
#include "hal.h"
#include "pwm.h"
#include <math.h>
#include "chprintf.h"
#include "drv8503.h"


#define SYSTEM_CORE_CLOCK               168000000
#define TIM_1_8_PERIOD_CLOCKS (2047)

#include "stm32f4xx_adc.h"

#if 0
#include "stm32f4xx_adc.h"
#include "ch.h"
#include "isr_vector_table.h"
#include "mc_interface.h"
#include "servo.h"
#include "hw.h"
#endif


#define TIM_1_8_CLOCK_HZ (SYSTEM_CORE_CLOCK/4)
#define CURRENT_MEAS_PERIOD ((float)(2*TIM_1_8_PERIOD_CLOCKS)/(float)TIM_1_8_CLOCK_HZ)

float g_shuntADCValue2Amps = 0.0;
float g_vbus_voltage = 12.0;
float g_currentZeroOffset[3] = { 0,0,0 } ;
float g_current[3] = { 0,0,0} ;
float g_phaseAngle = 0 ;
float g_current_Ibus = 0;
float g_motor_p_gain = 0.0025;
float g_motor_i_gain = 0.0;

int InitPWM_ADC(void)
{
#if 0
  ADC_ITConfig(ADC1,ADC_IT_JEOC,ENABLE);
  ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);

  palSetPad(GPIOB, GPIOB_PIN12); // on
#endif

  return 0;
}


static bool g_pwmThreadRunning = false;

static THD_WORKING_AREA(waThreadPWM, 128);

void PWMUpdateDrivePhase(int pa,int pb,int pc);

static const float one_by_sqrt3 = 0.57735026919f;
//static const float two_by_sqrt3 = 1.15470053838f;
static const float sqrt3_by_2 = 0.86602540378;

int g_phaseAngles[12][3];

static int g_drivePhase = 0;

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

inline int sqr(int val)
{
   return val * val;
}

inline float mysqrtf(float op1)
{
  if(op1 <= 0.f)
    return 0.f;

   float result;
   __ASM volatile ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (op1) );
   return (result);
}


/**
 * @brief   Configures and activates the PWM peripheral.
 * @note    Starting a driver that is already in the @p PWM_READY state
 *          disables all the active channels.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */

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

volatile bool g_pwmRun = true;

//--------------------------------
// Test functions
//--------------------------------
#include "svm.h"


static void queue_modulation_timings(float mod_alpha, float mod_beta) {
#if 1
    float tA = 0, tB = 0, tC = 0;
    SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
    uint16_t a = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
    uint16_t b = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
    uint16_t c = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
#else
    uint32_t a = 0,b = 0,c = 0;
    svm2(mod_alpha, mod_beta,TIM_1_8_PERIOD_CLOCKS,&a, &b, &c);
#endif
    PWMUpdateDrivePhase(a,b,c);
}

static void queue_voltage_timings(float v_alpha, float v_beta) {
    float vfactor = 1.0f / ((2.0f / 3.0f) * g_vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;
    queue_modulation_timings(mod_alpha, mod_beta);
}


static void scan_motor_loop(float omega, float voltage_magnitude) {
  while (g_pwmRun) {
#if 0
    PWMUpdateDrivePhase(
        TIM_1_8_PERIOD_CLOCKS/2,
        TIM_1_8_PERIOD_CLOCKS/2,
        TIM_1_8_PERIOD_CLOCKS/2
        );
    chThdSleepMicroseconds(CURRENT_MEAS_PERIOD*1000000);
#else
        for (float ph = 0.0f; ph < 2.0f * M_PI; ph += omega * CURRENT_MEAS_PERIOD) {
          chThdSleepMicroseconds(CURRENT_MEAS_PERIOD*1000000);
          //osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
          float v_alpha = voltage_magnitude * arm_cos_f32(ph);
          float v_beta  = voltage_magnitude * arm_sin_f32(ph);
          queue_modulation_timings(v_alpha, v_beta);
        }
#endif
        if (!palReadPad(GPIOB, GPIOA_PIN2)) {
          break;
        }
    }
}


#if 0
static float phase_current_from_adcval(Motor_t* motor, uint32_t ADCValue) {
    int adcval_bal = (int)ADCValue - (1<<11);
    float amp_out_volt = (3.3f/(float)(1<<12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * motor->phase_current_rev_gain;
    float current = shunt_volt * motor->shunt_conductance;
    return current;
}
#endif

void ShuntCalibration(void)
{
  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable
  // Enable shunt calibration mode.

  Drv8503SetRegister(DRV8503_REG_SHUNT_AMPLIFIER_CONTROL,
      DRV8503_DC_CAL_CH1 |
      DRV8503_DC_CAL_CH2 |
      DRV8503_DC_CAL_CH3
      );

  float ampGain = 10.0; // V/V gain
  float shuntResistance = 0.001;

  g_shuntADCValue2Amps  = (3.3f/((float)(1<<12) * ampGain * shuntResistance));

  // Wait a little for things to settle. Shouldn't need more than one,
  // but paranoia rules.
  for(int i = 0;i < 128;i++) {
    chBSemWait(&g_adcInjectedDataReady);
  }

  // Make
  float sums[3];
  for(int j = 0;j < 3;j++) {
    g_currentZeroOffset[j] = 0.0f;
    sums[j] = 0.0f;
  }

  // Sample values.

  int samples = 32;
  for(int i = 0;i < samples;i++) {
    chBSemWait(&g_adcInjectedDataReady);
    for(int j = 0;j < 3;j++)
      sums[j] += ((float) g_currentADCValue[j]) * g_shuntADCValue2Amps ;
    chThdSleepMicroseconds(5);
  }

  for(int j = 0;j < 3;j++)
    g_currentZeroOffset[j] = sums[j] / (float) samples;

  // Disable calibration mode.
  Drv8503SetRegister(DRV8503_REG_SHUNT_AMPLIFIER_CONTROL,
      0
      );

}


static bool FOC_current(float Id_des, float Iq_des) {
  //Current_control_t* ictrl = &motor->current_control;

  // Clarke transform
  // float Ialpha = -motor->current_meas.phB - motor->current_meas.phC;
  //  float Ibeta = one_by_sqrt3 * (motor->current_meas.phB - motor->current_meas.phC);

  float Ialpha = -g_current[1] - g_current[2];
  float Ibeta = one_by_sqrt3 * (g_current[1] - g_current[2]);

  // Park transform
  float c = arm_cos_f32(g_phaseAngle);
  float s = arm_sin_f32(g_phaseAngle);
  float Id = c*Ialpha + s*Ibeta;
  float Iq = c*Ibeta  - s*Ialpha;

  // Current error
  float Ierr_d = Id_des - Id;
  float Ierr_q = Iq_des - Iq;

  static float v_current_control_integral_d = 0;
  static float v_current_control_integral_q = 0;
  // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
  // Apply PI control

  float Vd = v_current_control_integral_d + Ierr_d * g_motor_p_gain;
  float Vq = v_current_control_integral_q + Ierr_q * g_motor_p_gain;

  float vfactor = 1.0f / ((2.0f / 3.0f) * g_vbus_voltage);
  float mod_d = vfactor * Vd;
  float mod_q = vfactor * Vq;

  // Vector modulation saturation, lock integrator if saturated
  // TODO make maximum modulation configurable
  float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f/mysqrtf(mod_d*mod_d + mod_q*mod_q);
  if (mod_scalefactor < 1.0f)
  {
    mod_d *= mod_scalefactor;
    mod_q *= mod_scalefactor;
    // TODO make decayfactor configurable
    v_current_control_integral_d *= 0.99f;
    v_current_control_integral_q *= 0.99f;
  } else {
    v_current_control_integral_d += Ierr_d * (g_motor_i_gain * CURRENT_MEAS_PERIOD);
    v_current_control_integral_q += Ierr_q * (g_motor_i_gain * CURRENT_MEAS_PERIOD);
  }

  // Compute estimated bus current
  g_current_Ibus = mod_d * Id + mod_q * Iq;

  // Inverse park transform
  float mod_alpha = c*mod_d - s*mod_q;
  float mod_beta  = c*mod_q + s*mod_d;

  // Apply SVM
  queue_modulation_timings(mod_alpha, mod_beta);

  return true;
}

static void compute_state(void)
{
#if 0
  static bool g_pinToggle = false;
  if(g_pinToggle) {
    palSetPad(GPIOB, GPIOB_PIN12); // on
    g_pinToggle = false;
  } else {
    palClearPad(GPIOB, GPIOB_PIN12); // off
    g_pinToggle = true;
  }
#endif

  // Compute motor currents;
  for(int i = 0;i < 3;i++)
    g_current[i] = ((float) g_currentADCValue[i] * g_shuntADCValue2Amps) - g_currentZeroOffset[i];

  // Compute current phase angle
  g_phaseAngle = hallToAngle(g_hall);


}

static void control_motor_loop() {

    while (g_pwmRun) {
      palClearPad(GPIOB, GPIOB_PIN12); // off
      chBSemWait(&g_adcInjectedDataReady);
      palSetPad(GPIOB, GPIOB_PIN12); // on

      compute_state();

      float vel_des = 5.0; // Radians a second.
#if 0
      // Position control
      // TODO Decide if we want to use encoder or pll position here
      float vel_des = motor->vel_setpoint;
      if (motor->control_mode >= CTRL_MODE_POSITION_CONTROL) {
          float pos_err = motor->pos_setpoint - motor->rotor.pll_pos;
          vel_des += motor->pos_gain * pos_err;
      }
#endif

      // Velocity limiting
      float vel_lim = 10;
      if (vel_des >  vel_lim) vel_des =  vel_lim;
      if (vel_des < -vel_lim) vel_des = -vel_lim;

      // Velocity control
      float Iq = 0.2;
#if 0
      float v_err = vel_des - motor->rotor.pll_vel;
      if (motor->control_mode >=  CTRL_MODE_VELOCITY_CONTROL) {
          Iq += motor->vel_gain * v_err;
      }

      // Velocity integral action before limiting
      Iq += motor->vel_integrator_current;

      // Apply motor direction correction
      Iq *= motor->rotor.motor_dir;

      // Current limiting
      float Ilim = motor->current_control.current_lim;
      bool limited = false;
      if (Iq > Ilim) {
          limited = true;
          Iq = Ilim;
      }
      if (Iq < -Ilim) {
          limited = true;
          Iq = -Ilim;
      }

      // Velocity integrator (behaviour dependent on limiting)
      if (motor->control_mode < CTRL_MODE_VELOCITY_CONTROL ) {
          // reset integral if not in use
          motor->vel_integrator_current = 0.0f;
      } else {
          if (limited) {
              // TODO make decayfactor configurable
              motor->vel_integrator_current *= 0.99f;
          } else {
              motor->vel_integrator_current += (motor->vel_integrator_gain * CURRENT_MEAS_PERIOD) * v_err;
          }
      }
#endif

      // Execute current command
      if(!FOC_current(0.0f, Iq)){
          break; // in case of error exit loop, motor->error has been set by FOC_current
      }
    }
}


static THD_FUNCTION(ThreadPWM, arg) {

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


int InitPWM(void)
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

void PWMUpdateDrivePhase(int pa,int pb,int pc)
{
  stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;
#if 1
  tim->CCER  =
      (STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC1NE ) |
      (STM32_TIM_CCER_CC2E | STM32_TIM_CCER_CC2NE ) |
      (STM32_TIM_CCER_CC3E | STM32_TIM_CCER_CC3NE );
  tim->CCMR1 = STM32_TIM_CCMR1_OC1M(6) |
      STM32_TIM_CCMR1_OC1PE |
      STM32_TIM_CCMR1_OC2M(6) |
      STM32_TIM_CCMR1_OC2PE;
  tim->CCMR2 =
    STM32_TIM_CCMR2_OC3M(6) |
    STM32_TIM_CCMR2_OC3PE |
    STM32_TIM_CCMR2_OC4M(6) |
    STM32_TIM_CCMR2_OC4PE;
  tim->CCR[0] = pa;
  tim->CCR[1] = pb;
  tim->CCR[2] = pc;

  tim->EGR = STM32_TIM_EGR_COMG;
#endif
}

int PWMRun()
{
  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable
  g_pwmRun = true;
  if(!g_pwmThreadRunning) {
    g_pwmThreadRunning = true;
    chThdCreateStatic(waThreadPWM, sizeof(waThreadPWM), NORMALPRIO, ThreadPWM, NULL);
  }
  return 0;
}

int PWMStop()
{
  g_pwmRun = false;
  palClearPad(GPIOC, GPIOC_PIN14); // Gate disable
  return 0;
}

float hallToAngleDebug(uint16_t *sensors,int *nearest,int *n0,int *n1,int *n2);

int PWMSVMScan(BaseSequentialStream *chp)
{
  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  while(true) {
    float voltage_magnitude = 0.12;
    float omega = 1.0;

    for (float ph = 0.0f; ph < 2.0f * M_PI; ph += omega * CURRENT_MEAS_PERIOD) {
      chThdSleepMicroseconds(5);
      //osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
      float v_alpha = voltage_magnitude * arm_cos_f32(ph);
      float v_beta  = voltage_magnitude * arm_sin_f32(ph);

#if 1
      float tA = 0, tB = 0, tC = 0;
      SVM(v_alpha, v_beta, &tA, &tB, &tC);
      uint16_t a = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
      uint16_t b = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
      uint16_t c = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
#else
      uint32_t a = 0,b = 0,c = 0;
      svm2(v_alpha,v_beta,TIM_1_8_PERIOD_CLOCKS,&a,&b,&c);
#endif

      chprintf(chp, "PWM: %d %d  ->   %d %d %d   \r\n",(int)(v_alpha*1000.0),(int)(v_beta*1000.0),
          (int)a,(int)b,(int)c);

      queue_modulation_timings(v_alpha, v_beta);

      if (!palReadPad(GPIOB, GPIOA_PIN2)) {
        break;
      }
    }
    if (!palReadPad(GPIOB, GPIOA_PIN2)) {
      break;
    }
  }


  palClearPad(GPIOC, GPIOC_PIN14); // Gate disable
  return 0;

}

void DisplayAngle(BaseSequentialStream *chp);


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

int PWMCalSVM(BaseSequentialStream *chp)
{
  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  float voltage_magnitude = 0.12;

  for(int phase = 0;phase < 12*7;phase++) {
    int phaseStep = phase % 12;
    float phaseAngle = (float) phase * M_PI * 2.0 / 12.0;
    float v_alpha = voltage_magnitude * arm_cos_f32(phaseAngle);
    float v_beta  = voltage_magnitude * arm_sin_f32(phaseAngle);

    queue_modulation_timings(v_alpha, v_beta);

    // Wait for position to settle
    chThdSleepMilliseconds(1000);

    // Sync to avoid reading variables when they're being updated.
    chBSemWait(&g_adcInjectedDataReady);

    g_phaseAngles[phaseStep][0] += g_hall[0];
    g_phaseAngles[phaseStep][1] += g_hall[1];
    g_phaseAngles[phaseStep][2] += g_hall[2];

    chprintf(chp, "Cal %d : %04d %04d %04d   \r\n",phase,g_hall[0],g_hall[1],g_hall[2]);
  }

  for(int i = 0;i < 12;i++) {
    g_phaseAngles[i][0] /= 7;
    g_phaseAngles[i][1] /= 7;
    g_phaseAngles[i][2] /= 7;
    chprintf(chp, "Cal %d : %04d %04d %04d   \r\n",
        i,g_phaseAngles[i][0],g_phaseAngles[i][1],g_phaseAngles[i][2]);
  }
  PWMUpdateDrive(12,0);

  palClearPad(GPIOC, GPIOC_PIN14); // Gate disable
  //DisplayAngle(chp);
  return 0;
}

void DisplayAngle(BaseSequentialStream *chp)
{
  while(true) {
    int nearest,n0,n1,n2;
    float angle = hallToAngleDebug(g_hall,&nearest,&n0,&n1,&n2);

    int phase = angle * 10;

    chprintf(chp, "Read %4d : %04d %04d %04d  Ne:%d n0:%d n1:%d n2:%d \r\n",
        phase,g_hall[0],g_hall[1],g_hall[2],nearest,n0,n1,n2);
    chThdSleepMilliseconds(50);

    if (!palReadPad(GPIOB, GPIOA_PIN2)) {
      break;
    }
  }

}


float hallToAngle(uint16_t *sensors)
{
  int distTable[12];
  int phase = 0;
  int minDist = sqr(g_phaseAngles[0][0] - sensors[0]) +
                sqr(g_phaseAngles[0][1] - sensors[1]) +
                sqr(g_phaseAngles[0][2] - sensors[2]);
  distTable[0] = minDist;

  for(int i = 1;i < 12;i++) {
    int dist = sqr(g_phaseAngles[i][0] - sensors[0]) +
                  sqr(g_phaseAngles[i][1] - sensors[1]) +
                  sqr(g_phaseAngles[i][2] - sensors[2]);
    distTable[i] = dist;
    if(dist < minDist) {
      phase = i;
      minDist = dist;
    }
  }
  int last = phase - 1;
  if(last < 0) last = 11;
  int next = phase + 1;
  if(last > 11) last = 0;
  int lastDist2 = distTable[last];
  int nextDist2 = distTable[next];
  float nearDist = mysqrtf(minDist);
  float angle = phase * 2.0;
  if(lastDist2 < nextDist2) {
    float lastDist = mysqrtf(lastDist2);
    angle -= nearDist / (lastDist + nearDist);
  } else {
    float nextDist = mysqrtf(nextDist2);
    angle += nearDist / (nextDist + nearDist);
  }
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return angle * M_PI * 2.0 / 24.0;
}

float hallToAngleDebug(uint16_t *sensors,int *nearest,int *n0,int *n1,int *n2)
{
  int distTable[12];
  int phase = 0;
  int minDist = sqr(g_phaseAngles[0][0] - sensors[0]) +
                sqr(g_phaseAngles[0][1] - sensors[1]) +
                sqr(g_phaseAngles[0][2] - sensors[2]);
  distTable[0] = minDist;

  for(int i = 1;i < 12;i++) {
    int dist = sqr(g_phaseAngles[i][0] - sensors[0]) +
                  sqr(g_phaseAngles[i][1] - sensors[1]) +
                  sqr(g_phaseAngles[i][2] - sensors[2]);
    distTable[i] = dist;
    if(dist < minDist) {
      phase = i;
      minDist = dist;
    }
  }
  *nearest = phase;
  int last = phase - 1;
  if(last < 0) last = 11;
  int next = phase + 1;
  if(last > 11) last = 0;
  int lastDist2 = distTable[last];
  int nextDist2 = distTable[next];
  *n0 = minDist;
  *n1 = lastDist2;
  *n2 = nextDist2;
  float nearDist = mysqrtf(minDist);
  float angle = phase * 2.0;
  if(lastDist2 < nextDist2) {
    float lastDist = mysqrtf(lastDist2);
    angle -= nearDist / (lastDist + nearDist);
  } else {
    float nextDist = mysqrtf(nextDist2);
    angle += nearDist / (nextDist + nearDist);
  }
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return angle * M_PI * 2.0 / 24.0;
}





