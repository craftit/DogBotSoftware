

#include <stdint.h>
#include "hal.h"
#include "pwm.h"
#include "mathfunc.h"
#include "chprintf.h"
#include "drv8503.h"
#include "svm.h"

#include "coms.h"


#define SYSTEM_CORE_CLOCK     168000000

#include "stm32f4xx_adc.h"


#define TIM_1_8_CLOCK_HZ (SYSTEM_CORE_CLOCK/4)
#define CURRENT_MEAS_PERIOD ((float)(2*TIM_1_8_PERIOD_CLOCKS)/(float)TIM_1_8_CLOCK_HZ)
#define TIM_1_8_PERIOD_CLOCKS (2047)

float g_shuntADCValue2Amps = 0.0;
float g_vbus_voltage = 12.0;
float g_currentZeroOffset[3] = { 0,0,0 } ;
float g_current[3] = { 0,0,0} ;
float g_phaseAngle = 0 ;

float g_current_Ibus = 0;
float g_motor_p_gain = 1.2;
float g_motor_i_gain = 0.0;

int g_phaseAngles[12][3];
float g_phaseDistance[12];

static bool g_pwmThreadRunning = false;

static THD_WORKING_AREA(waThreadPWM, 128);

void PWMUpdateDrivePhase(int pa,int pb,int pc);

static int g_drivePhase = 0;

volatile bool g_pwmRun = true;


static void queue_modulation_timings(float mod_alpha, float mod_beta) {
  float tA = 0, tB = 0, tC = 0;
  SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
  uint16_t a = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
  uint16_t b = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
  uint16_t c = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
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
    for (float ph = 0.0f; ph < 2.0f * M_PI; ph += omega * CURRENT_MEAS_PERIOD) {
      chThdSleepMicroseconds(CURRENT_MEAS_PERIOD*1000000);
      //osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
      float v_alpha = voltage_magnitude * arm_cos_f32(ph);
      float v_beta  = voltage_magnitude * arm_sin_f32(ph);
      queue_modulation_timings(v_alpha, v_beta);
    }
    if (!palReadPad(GPIOB, GPIOA_PIN2)) {
      break;
    }
  }
}

void ShuntCalibration(void)
{
  // Enable shunt calibration mode.
  int gainMode = DRV8503_GAIN_CS1_40 | DRV8503_GAIN_CS2_40 | DRV8503_GAIN_CS3_40;

  float ampGain = 40.0; // V/V gain
  float shuntResistance = 0.001;

  Drv8503SetRegister(DRV8503_REG_SHUNT_AMPLIFIER_CONTROL,
      DRV8503_DC_CAL_CH1 |
      DRV8503_DC_CAL_CH2 |
      DRV8503_DC_CAL_CH3 |
      gainMode
      );


  g_shuntADCValue2Amps  = (3.3f/((float)(1<<12) * ampGain * shuntResistance));

  // Wait a little for things to settle. Shouldn't need more than one,
  // but paranoia rules.
  for(int i = 0;i < 128;i++) {
    chBSemWait(&g_adcInjectedDataReady);
  }

  // Make
  float sums[3];
  for(int j = 0;j < 3;j++) {
    sums[j] = 0.0f;
  }

  // Sample values.

  int samples = 32;
  for(int i = 0;i < samples;i++) {
    chBSemWait(&g_adcInjectedDataReady);
    for(int j = 0;j < 3;j++)
      sums[j] += ((float) g_currentADCValue[j]) * g_shuntADCValue2Amps ;
  }

  for(int j = 0;j < 3;j++)
    g_currentZeroOffset[j] = sums[j] / (float) samples;

  // Disable calibration mode.
  Drv8503SetRegister(DRV8503_REG_SHUNT_AMPLIFIER_CONTROL,
      gainMode
      );

}

float g_Ierr_d;
float g_Ierr_q;

bool g_breakMode = false;
// The following function is based on that from the ODrive project.

static bool FOC_current(float Id_des, float Iq_des) {
  //Current_control_t* ictrl = &motor->current_control;

  // Clarke transform
  float Ialpha = -g_current[1] - g_current[2];
  float Ibeta = one_by_sqrt3 * (g_current[1] - g_current[2]);

  // Park transform
  float c = arm_cos_f32(g_phaseAngle);
  float s = arm_sin_f32(g_phaseAngle);
  g_Id = c*Ialpha + s*Ibeta;
  g_Iq = c*Ibeta  - s*Ialpha;

  // Current error
  g_Ierr_d = Id_des - g_Id;
  g_Ierr_q = Iq_des - g_Iq;

  static float g_current_control_integral_d = 0;
  static float g_current_control_integral_q = 0;
  // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
  // Apply PI control

  float Vd = g_current_control_integral_d + g_Ierr_d * g_motor_p_gain;
  float Vq = g_current_control_integral_q + g_Ierr_q * g_motor_p_gain;

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
    // TODO make decay factor configurable
    g_current_control_integral_d *= 0.99f;
    g_current_control_integral_q *= 0.99f;
  } else {
    g_current_control_integral_d += g_Ierr_d * (g_motor_i_gain * CURRENT_MEAS_PERIOD);
    g_current_control_integral_q += g_Ierr_q * (g_motor_i_gain * CURRENT_MEAS_PERIOD);
  }

  // Compute estimated bus current
  g_current_Ibus = mod_d * g_Id + mod_q * g_Iq;
#if 0
  if(g_current_Ibus < 0) {
    // Breaking...
    if(!g_breakMode) {
      g_breakMode = true;
      SetModeBreak();
    }
  } else {
    if(g_breakMode) {
      g_breakMode = false;
      SetModeFOC();
    }
  }
#endif

  // Inverse park transform
  float mod_alpha = c*mod_d - s*mod_q;
  float mod_beta  = c*mod_q + s*mod_d;

  // Apply SVM
  queue_modulation_timings(mod_alpha, mod_beta);

  return true;
}

float g_demandPhasePosition = 0;
float g_demandPhaseVelocity = 0;
float g_demandTorque = 0;

int g_phaseRotationCount = 0;
float g_currentPhasePosition = 0;
float g_currentPhaseVelocity = 0;
float g_velocityGain = 0.01;
float g_velocityFilter = 16.0;
float g_positionGain = 1.0;
float g_torqueLimit = 5.0;
float g_positionIGain = 0.1;
float g_positionIClamp = 5.0;
float g_positionISum = 0.0;
float g_Id = 0.0;
float g_Iq = 0.0;

enum ControlModeT g_controlMode = CM_Idle;

static void ComputeState(void)
{
  // Compute current phase angle
  float lastAngle = g_phaseAngle;

  // This returns an angle between 0 and 2 pi
  g_phaseAngle = hallToAngle(g_hall);

  float angleDiff = lastAngle - g_phaseAngle;
  // If the change is large we have wrapped around.
  if(angleDiff > M_PI) {
    angleDiff -= 2*M_PI;
    g_phaseRotationCount++;
  }
  if(angleDiff < -M_PI) {
    angleDiff += 2*M_PI;
    g_phaseRotationCount--;
  }

  // If we just sum up difference things will drift.
  g_currentPhasePosition = (float) g_phaseRotationCount * 2 * M_PI + g_phaseAngle;

  //
  // Velocity estimate, filtered a little.
  g_currentPhaseVelocity = (g_currentPhaseVelocity * (g_velocityFilter-1.0) + angleDiff / CURRENT_MEAS_PERIOD )/g_velocityFilter;

  // Compute motor currents;
  // Make sure they sum to zero
  float sum = 0;
  float tmpCurrent[3];
  for(int i = 0;i < 3;i++) {
    float c = ((float) g_currentADCValue[i] * g_shuntADCValue2Amps) - g_currentZeroOffset[i];
    tmpCurrent[i] = c;
    sum += c;
  }
  sum /= 3.0f;
  for(int i = 0;i < 3;i++) {
    g_current[i] = tmpCurrent[i] - sum;
  }

}

static void SetTorque(float torque) {

#if 1
  if(torque > g_torqueLimit)
    torque = g_torqueLimit;
  if(torque < -g_torqueLimit)
    torque = -g_torqueLimit;

  FOC_current(0,-torque);
#else

  float voltage_magnitude = torque;

  if(voltage_magnitude > g_torqueLimit)
    voltage_magnitude = g_torqueLimit;
  if(voltage_magnitude < -g_torqueLimit)
    voltage_magnitude = -g_torqueLimit;

  // Apply velocity limits ?

  float pangle = g_phaseAngle;
  pangle -= M_PI/2.0;
  float v_alpha = voltage_magnitude * arm_cos_f32(pangle);
  float v_beta  = voltage_magnitude * arm_sin_f32(pangle);
  queue_modulation_timings(v_alpha, v_beta);
#endif
}

static void MotorControlLoop(void) {

    while (g_pwmRun) {
      palClearPad(GPIOB, GPIOB_PIN12); // Turn output off to measure timing
      chBSemWait(&g_adcInjectedDataReady);

      ComputeState();

      float torque = g_demandTorque;
      //float targetVelocity = g_demandPhaseVelocity;
      float targetPosition = g_demandPhasePosition;

      switch(g_controlMode)
      {
        case CM_Idle: // Maybe turn off the MOSFETS ?
          SetTorque(0);
          break;
        case CM_Break:
          // Just turn everything off, this will passively break the motor
          PWMUpdateDrivePhase(
              TIM_1_8_PERIOD_CLOCKS/2,
              TIM_1_8_PERIOD_CLOCKS/2,
              TIM_1_8_PERIOD_CLOCKS/2
              );
          break;
        case CM_Velocity: {
          g_demandPhasePosition += g_demandPhaseVelocity * CURRENT_MEAS_PERIOD;
          targetPosition = g_demandPhasePosition;
        }
        /* no break */
        case CM_Position: {
          float positionError = targetPosition - g_currentPhasePosition;
#if 0
          const float deadBand = M_PI/30;
          if(positionError > 0) {
            if(positionError < deadBand)
              positionError = 0;
            else
              positionError -= deadBand;
          } else {
            if(positionError > -deadBand)
              positionError = 0;
            else
              positionError += deadBand;
          }
#endif
          g_positionISum += positionError * CURRENT_MEAS_PERIOD;
          if(g_positionISum > g_positionIClamp)  g_positionISum = g_positionIClamp;
          if(g_positionISum < -g_positionIClamp) g_positionISum = -g_positionIClamp;
          torque = -positionError * g_positionGain + -g_positionISum * g_positionIGain;
          SetTorque(torque);
        } break;
        case CM_Torque: {
          SetTorque(torque);
        } break;
      }

    }
}


static THD_FUNCTION(ThreadPWM, arg) {

  (void)arg;
  chRegSetThreadName("pwm");

  PWMUpdateDrivePhase(TIM_1_8_PERIOD_CLOCKS/2,TIM_1_8_PERIOD_CLOCKS/2,TIM_1_8_PERIOD_CLOCKS/2);

  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  //! Make sure controller is setup.
  Drv8503Init();

  // This is quick, so may as well do it every time.
  ShuntCalibration();

  MotorControlLoop();

  // Make sure motor isn't being driven.

  PWMUpdateDrivePhase(TIM_1_8_PERIOD_CLOCKS/2,TIM_1_8_PERIOD_CLOCKS/2,TIM_1_8_PERIOD_CLOCKS/2);

  palClearPad(GPIOC, GPIOC_PIN14); // Gate disable

  g_pwmThreadRunning = false;
}


void SetModeFOC(void)
{
  stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;
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
}

// Not implemented yet.
void SetModeBreak(void)
{
  stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;
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
}

int InitPWM(void)
{
  g_drivePhase = 0;

  rccEnableTIM1(FALSE);
  rccResetTIM1();

  stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;

  uint16_t psc = 0; // (SYSTEM_CORE_CLOCK / 80000000) - 1;
  tim->PSC  = psc;
  tim->ARR  = TIM_1_8_PERIOD_CLOCKS; // This should give about 20KHz
  tim->CR2  = 0;

  SetModeFOC();

  tim->EGR   = STM32_TIM_EGR_UG;      /* Update event.                */
  tim->SR    = 0;                     /* Clear pending IRQs.          */
  tim->DIER  = 0;
  tim->BDTR  = STM32_TIM_BDTR_MOE;
  //| STM32_TIM_BDTR_OSSR;

  /* Timer configured and started.*/
  tim->CR1   = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN | STM32_TIM_CR1_CMS(3);

  tim->CCR[0] = TIM_1_8_PERIOD_CLOCKS/2;
  tim->CCR[1] = TIM_1_8_PERIOD_CLOCKS/2;
  tim->CCR[2] = TIM_1_8_PERIOD_CLOCKS/2;
  tim->CCR[3] = TIM_1_8_PERIOD_CLOCKS - 1;

  tim->CR2  = STM32_TIM_CR2_CCPC | STM32_TIM_CR2_MMS(7); // Use the COMG bit to update.


  //palSetPad(GPIOB, GPIOB_PIN12); // Turn on flag pin

  // Pre-compute the distance between this position and the last.
  int lastIndex = 11;
  for(int i = 0;i < 12;i++) {
    int sum = 0;
    for(int k = 0;k < 3;k++) {
      int diff = g_phaseAngles[i][k] - g_phaseAngles[lastIndex][k];
      sum += diff * diff;
    }
    g_phaseDistance[i] = mysqrtf((float) sum);
    lastIndex = i;
  }

  palSetPad(GPIOC, GPIOC_PIN13); // Wake
  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  return 0;
}


void PWMUpdateDrivePhase(int pa,int pb,int pc)
{
  // Make sure we don't overflow around.
  if(pa < 0) pa = 0;
  if(pb < 0) pb = 0;
  if(pc < 0) pc = 0;
  if(pa > TIM_1_8_PERIOD_CLOCKS) pa = TIM_1_8_PERIOD_CLOCKS;
  if(pb > TIM_1_8_PERIOD_CLOCKS) pb = TIM_1_8_PERIOD_CLOCKS;
  if(pc > TIM_1_8_PERIOD_CLOCKS) pc = TIM_1_8_PERIOD_CLOCKS;

  stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;
#if 1
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
    chThdCreateStatic(waThreadPWM, sizeof(waThreadPWM), NORMALPRIO+8, ThreadPWM, NULL);
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

      float tA = 0, tB = 0, tC = 0;
      SVM(v_alpha, v_beta, &tA, &tB, &tC);
      uint16_t a = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
      uint16_t b = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
      uint16_t c = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);

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

    for(int i = 0;i < 3;i++) {
      g_phaseAngles[phaseStep][i] += g_hall[i];
    }

    chprintf(chp, "Cal %d : %04d %04d %04d   \r\n",phase,g_hall[0],g_hall[1],g_hall[2]);
  }
  for(int i = 0;i < 12;i++) {
    g_phaseAngles[i][0] /= 7;
    g_phaseAngles[i][1] /= 7;
    g_phaseAngles[i][2] /= 7;
    chprintf(chp, "Cal %d : %04d %04d %04d   \r\n",
        i,g_phaseAngles[i][0],g_phaseAngles[i][1],g_phaseAngles[i][2]);
  }

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

// This returns an angle between 0 and 2 pi

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
  float angle = phase * 2.0;
  float lastDist = mysqrtf(lastDist2) / g_phaseDistance[phase];
  float nextDist = mysqrtf(nextDist2) / g_phaseDistance[next];
  angle -= (nextDist-lastDist)/(nextDist + lastDist);
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return angle * M_PI * 2.0 / 24.0;
}






