

#include <stdint.h>
#include "hal.h"
#include "pwm.h"
#include "mathfunc.h"
#include "chprintf.h"
#include "drv8503.h"
#include "svm.h"

#include "coms.h"
#include "dogbot/protocol.h"
#include "storedconf.h"

#include "motion.h"

#define SYSTEM_CORE_CLOCK     168000000

#include "stm32f4xx_adc.h"

float g_maxSupplyVoltage = 40.0;
float g_maxOperatingTemperature = 75.0;

#define TIM_1_8_CLOCK_HZ (SYSTEM_CORE_CLOCK/4)
#define TIM_1_8_PERIOD_CLOCKS (2047)
#define CURRENT_MEAS_PERIOD ((float)(TIM_1_8_PERIOD_CLOCKS)/(float)TIM_1_8_CLOCK_HZ)

int g_motorReportSampleRate = 1.0 / (100.0 * CURRENT_MEAS_PERIOD);  //CURRENT_MEAS_PERIOD/100; // The target rate is 100Hz

BSEMAPHORE_DECL(g_reportSampleReady,0); // 100Hz report loop

float g_shuntADCValue2Amps = 0.0;
float g_vbus_voltage = 12.0;
float g_currentZeroOffset[3] = { 0,0,0 } ;
float g_current[3] = { 0,0,0} ;
float g_phaseAngle = 0 ;

float g_driveTemperature = 0.0;

float g_current_Ibus = 0;
float g_motor_p_gain = 1.2;  // 1.2
float g_motor_i_gain = 0.0;  // 0.0

int g_phaseAngles[12][3];
float g_phaseDistance[12];

bool g_pwmThreadRunning = false;
volatile bool g_pwmRun = true;

int g_pwmTimeoutCount = 0 ;
bool g_pwmFullReport = false;
bool g_motorControlLoopReady = true;
bool g_lastLimitState[3];

static THD_WORKING_AREA(waThreadPWM, 512);

void PWMUpdateDrivePhase(int pa,int pb,int pc);


int PWMSetPosition(uint16_t position,uint16_t torque)
{
  g_torqueLimit = ((float) torque) * g_absoluteMaxTorque / (65535.0);
  g_demandPhasePosition = ((float) position) * 7.0 * 21.0 * M_PI * 2.0/ 65535.0;
  return 0;
}

int CheckHallInRange(void) {
  // Are sensor readings outside the expected range ?
  for(int i = 0;i < 3;i++) {
    if(g_hall[i] < 1500 || g_hall[i] > 2800)
      return FC_NoSensor;
  }
  return FC_Ok;
}


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
float g_torqueAverage = 0.0;
float g_positionIGain = 0.1;
float g_positionIClamp = 5.0;
float g_positionISum = 0.0;
float g_Id = 0.0;
float g_Iq = 0.0;

enum PWMControlModeT g_controlMode = CM_Break;

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

  g_torqueAverage = (g_torqueAverage * 30.0 + torque)/31.0;

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

  int loopCount = 0;
  g_motorControlLoopReady = true;

  while (g_pwmRun) {
    palClearPad(GPIOB, GPIOB_PIN12); // Turn output off to measure timing
    if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
      g_pwmTimeoutCount++;
      continue;
    }

    ComputeState();

    float torque = g_demandTorque;
    //float targetVelocity = g_demandPhaseVelocity;
    float targetPosition = g_demandPhasePosition;

    switch(g_controlMode)
    {
      case CM_Idle: // Maybe turn off the MOSFETS ?
        PWMUpdateDrivePhase(
            TIM_1_8_PERIOD_CLOCKS/2,
            TIM_1_8_PERIOD_CLOCKS/2,
            TIM_1_8_PERIOD_CLOCKS/2
            );
        break;
      case CM_Fault:
      case CM_Final:
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

    // Check limit switches.
    {
#if 0
      bool es1 = palReadPad(GPIOC, GPIOC_PIN6);
      if(es1 != g_lastLimitState[0]) {
        g_lastLimitState[0] = es1;
        MotionUpdateEndStop(0,es1,g_currentPhasePosition,g_currentPhaseVelocity);
      }
      bool es2 = palReadPad(GPIOC, GPIOC_PIN7);
      if(es2 != g_lastLimitState[1]) {
        g_lastLimitState[1] = es2;
        MotionUpdateEndStop(1,es2,g_currentPhasePosition,g_currentPhaseVelocity);
      }
#endif
      bool es3 = palReadPad(GPIOC, GPIOC_PIN8);
      if(es3 != g_lastLimitState[2]) {
        g_lastLimitState[2] = es3;
        MotionUpdateEndStop(2,es3,g_currentPhasePosition,g_currentPhaseVelocity);
      }
    }


    // Flag motion control update if needed.
    if(++loopCount >= g_motorReportSampleRate) {
      loopCount = 0;
      chBSemSignal(&g_reportSampleReady);
    }

    // Do some sanity checks
    int errCode;
    if((errCode = CheckHallInRange()) != FC_Ok) {
      FaultDetected(errCode);
    }

    // Last send report if needed.
    if(g_pwmFullReport) {
      struct PacketT *pkt;
      if((pkt = GetEmptyPacket(TIME_IMMEDIATE)) != 0) {
        struct PacketPWMStateC *ps = (struct PacketPWMStateC *)&(pkt->m_data);
        pkt->m_len = sizeof(struct PacketPWMStateC);
        ps->m_packetType = CPT_PWMState;
        ps->m_tick = g_adcTickCount;
        for(int i = 0;i < 3;i++)
          ps->m_curr[i] = g_currentADCValue[i];
        for(int i = 0;i < 3;i++)
          ps->m_hall[i] = g_hall[i];
        ps->m_angle = g_phaseAngle * 65535.0 / (2.0 * M_PI);
        PostPacket(pkt);
      }
    }

  }
}



static THD_FUNCTION(ThreadPWM, arg) {

  (void)arg;
  chRegSetThreadName("pwm");

  palSetPad(GPIOC, GPIOC_PIN13); // Wake

  //! Wait for powerup to complete
  // TODO: Add a timeout and report and error
  while (!palReadPad(GPIOD, GPIOD_PIN2)) {
    chThdSleepMilliseconds(100);
  }

  //! Read initial state of endstop switches
  g_lastLimitState[0] = palReadPad(GPIOC, GPIOC_PIN6); // Endstop 1
  g_lastLimitState[1] = palReadPad(GPIOC, GPIOC_PIN7); // Endstop 2
  g_lastLimitState[2] = palReadPad(GPIOC, GPIOC_PIN8); // Index

  //! Make sure controller is setup.
  Drv8503Init();

  //! Wait a bit more
  chThdSleepMilliseconds(100);

  // Setup PWM
  InitPWM();

  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  // This is quick, so may as well do it every time.
  ShuntCalibration();

  // Reset calibration state.
  MotionResetCalibration();

  // Do main control loop
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

void InitHall2Angle(void);

int InitPWM(void)
{

  InitHall2Angle();

  rccEnableTIM1(FALSE);
  rccResetTIM1();

  stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;

  uint16_t psc = 0; // (SYSTEM_CORE_CLOCK / 80000000) - 1;
  tim->PSC  = psc;
  tim->ARR  = TIM_1_8_PERIOD_CLOCKS; // This should give about 20KHz
  tim->CR2  = 0;

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
  tim->CCR[3] = TIM_1_8_PERIOD_CLOCKS - 2;

  tim->CR2  = STM32_TIM_CR2_CCPC | STM32_TIM_CR2_MMS(7); // Use the COMG bit to update. 7=Tim4 3=Update event  =


  //palSetPad(GPIOB, GPIOB_PIN12); // Turn on flag pin


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

  tim->CCR[0] = pa;
  tim->CCR[1] = pb;
  tim->CCR[2] = pc;

  tim->EGR = STM32_TIM_EGR_COMG;
}

int PWMRun()
{
  if(g_pwmRun)
    return 0;

  g_pwmRun = true;
  if(!g_pwmThreadRunning) {
    g_motorControlLoopReady = false;
    g_pwmThreadRunning = true;
    chThdCreateStatic(waThreadPWM, sizeof(waThreadPWM), NORMALPRIO+8, ThreadPWM, NULL);
    while(!g_motorControlLoopReady) {
      chThdSleepMilliseconds(10);
    }
  }
  return 0;
}

int PWMStop()
{
  if(!g_pwmRun)
    return 0;
  g_pwmRun = false;
  // Wait for thread to shutdown to avoid race conditions.
  while(g_pwmThreadRunning) {
    chThdSleepMilliseconds(10);
  }

  // Don't claim to be calibrated.
  MotionResetCalibration();

  //palClearPad(GPIOC, GPIOC_PIN14); // Gate disable
  return 0;
}

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


enum FaultCodeT PWMSelfTest()
{

  float rail5V = Read5VRailVoltage();
  if(rail5V < 4.5 || rail5V > 5.5)
    return FC_Internal5VRailOutOfRange;

  g_vbus_voltage = ReadSupplyVoltage();
  if(g_vbus_voltage > g_maxSupplyVoltage)
    return FC_OverVoltage;

  g_driveTemperature = ReadDriveTemperature();
  if(g_driveTemperature > g_maxOperatingTemperature)
    return FC_OverTemprature;

  InitPWM();
  if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
    return FC_Internal; // Not sure what is going on.
  }

  // Check hall sensors.

  // If pins are floating, pull them into a fixed state.

  palSetPadMode(GPIOC,GPIOC_PIN0,PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC,GPIOC_PIN1,PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC,GPIOC_PIN2,PAL_MODE_INPUT_PULLUP);

  chThdSleepMilliseconds(100);

  // Change them back to analog inputs.

  palSetPadMode(GPIOC,GPIOC_PIN0,PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOC,GPIOC_PIN1,PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOC,GPIOC_PIN2,PAL_MODE_INPUT_ANALOG);

  // Wait for next measurement

  if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
    return FC_Internal; // Not sure what is going on.
  }
  if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
    return FC_Internal; // Not sure what is going on.
  }

  int errCode = FC_Ok;
  if((errCode = CheckHallInRange()) != FC_Ok)
    return errCode;

  // Are all readings very similar ?
  // Sensor may not be near motor. This isn't full proof but better than nothing.
  int diff1 = g_hall[1] - g_hall[0];
  int diff2 = g_hall[1] - g_hall[2];
  if(diff1 < 0) diff1 *= -1;
  if(diff2 < 0) diff2 *= -1;
  if(diff1 < 20 && diff2 < 20)
    return FC_NoSensor;

  return FC_Ok;
}

enum FaultCodeT PWMFactoryCal()
{
  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable


  InitPWM();


  for(int phase = 0;phase < 12*7;phase++) {
    g_vbus_voltage = ReadSupplyVoltage();
    if(g_vbus_voltage < 12.0)
      return FC_UnderVoltage;
    float voltage_magnitude = (0.12 * 12.0) / g_vbus_voltage;

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
  }
  for(int i = 0;i < 12;i++) {
    g_phaseAngles[i][0] /= 7;
    g_phaseAngles[i][1] /= 7;
    g_phaseAngles[i][2] /= 7;
  }

  palClearPad(GPIOC, GPIOC_PIN14); // Gate disable

  if(!g_eeInitDone) {
    StoredConf_Init();
    g_eeInitDone = true;
  }
  g_storedConfig.configState = 1;
  g_storedConfig.controllerId = 2;
  for(int i = 0;i < 12;i++) {
    g_storedConfig.phaseAngles[i][0] = g_phaseAngles[i][0];
    g_storedConfig.phaseAngles[i][1] = g_phaseAngles[i][1];
    g_storedConfig.phaseAngles[i][2] = g_phaseAngles[i][2];
  }
  if(!StoredConf_Save(&g_storedConfig)) {
    return FC_InternalStoreFailed;
  }

  //DisplayAngle(chp);
  return FC_Ok;
}


int PWMCalSVM(BaseSequentialStream *chp)
{
  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  float voltage_magnitude = 0.06;

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
    if (!palReadPad(GPIOB, GPIOA_PIN2)) {
      return 0;
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

// This returns an angle between 0 and 2 pi


float hallToAngleRef(uint16_t *sensors)
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
  if(next > 11) next = 0;
  int lastDist2 = distTable[last];
  int nextDist2 = distTable[next];
  float angle = phase * 2.0;
  float lastDist = mysqrtf(lastDist2) / g_phaseDistance[phase];
  float nextDist = mysqrtf(nextDist2) / g_phaseDistance[next];
  angle -= (nextDist-lastDist)/(nextDist + lastDist);
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return (angle * M_PI * 2.0 / 24.0) + 1.04;
}

float g_hallToAngleOriginOffset = -2000;
float g_phaseAnglesNormOrg[12][3];

void InitHall2Angle(void)
{
  {
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
  }

  for(int i = 0;i < 12;i++) {
    float sumMag = 0;

    for(int k = 0;k < 3;k++) {
      sumMag += sqr(g_phaseAngles[i][k] - g_hallToAngleOriginOffset);
    }

    sumMag = mysqrtf(sumMag);
    for(int k = 0;k < 3;k++) {
      g_phaseAnglesNormOrg[i][k] = (g_phaseAngles[i][k]-g_hallToAngleOriginOffset) / sumMag;
    }
  }
}



float hallToAngleDot2(uint16_t *sensors)
{
  float distTable[12];
  int phase = 0;
  float norm[3];
  norm[0] = (float) sensors[0] - g_hallToAngleOriginOffset;
  norm[1] = (float) sensors[1] - g_hallToAngleOriginOffset;
  norm[2] = (float) sensors[2] - g_hallToAngleOriginOffset;
  float mag = mysqrtf(sqr(norm[0]) + sqr(norm[1]) + sqr(norm[2]));
  for(int j = 0;j < 3;j++)
    norm[j] /= mag;

  //RavlDebug("Vec: %f %f %f",norm[0],norm[1],norm[2]);

  //mag = mysqrtf(sqr(g_phaseAngles[0][0]) + sqr(g_phaseAngles[0][1]) + sqr(g_phaseAngles[0][2]));

  float maxCorr = ((g_phaseAnglesNormOrg[0][0]) * norm[0]) +
                  ((g_phaseAnglesNormOrg[0][1]) * norm[1]) +
                  ((g_phaseAnglesNormOrg[0][2]) * norm[2]);

  distTable[0] = maxCorr;

  for(int i = 1;i < 12;i++) {
    //mag = mysqrtf(sqr(g_phaseAngles[i][0]) + sqr(g_phaseAngles[i][1]) + sqr(g_phaseAngles[i][2]));
    float corr = ((g_phaseAnglesNormOrg[i][0]) * norm[0]) +
                  ((g_phaseAnglesNormOrg[i][1]) * norm[1]) +
                  ((g_phaseAnglesNormOrg[i][2]) * norm[2]);
    distTable[i] = corr;
    //RavlDebug("Corr:%f ",corr);
    if(corr > maxCorr) {
      phase = i;
      maxCorr = corr;
    }
  }
  int last = phase - 1;
  if(last < 0) last = 11;
  int next = phase + 1;
  if(next > 11) next = 0;
  float lastDist2 = distTable[last];
  float nextDist2 = distTable[next];
  float angle = phase * 2.0;
  float lastDist = maxCorr - lastDist2;
  float nextDist = maxCorr - nextDist2;
  //Average error:0.007239  Abs:0.207922 Mag:0.263140
  float corr = (nextDist-lastDist)/(nextDist + lastDist);
  angle -= corr;
  //RavlDebug("Last:%f  Max:%f Next:%f Corr:%f ",lastDist,maxCorr,nextDist,corr);
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return (angle * M_PI * 2.0 / 24.0);
  //return angle;
}


float hallToAngle(uint16_t *sensors)
{
  return hallToAngleDot2(sensors);
}



