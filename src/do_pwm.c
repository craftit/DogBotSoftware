

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

float g_phaseResistance = 0.002;
float g_phaseOffsetVoltage = 0.1;
float g_phaseInductance = 1e-9;

#define TIM_1_8_CLOCK_HZ (SYSTEM_CORE_CLOCK/4)
//#define TIM_1_8_PERIOD_CLOCKS (2100)   // 20 KHz
#define TIM_1_8_PERIOD_CLOCKS (2333)   // 18 KHz
//#define TIM_1_8_PERIOD_CLOCKS (2625)  // 16 KHz
//#define TIM_1_8_PERIOD_CLOCKS (4095)  // 10 KHz
#define CURRENT_MEAS_PERIOD ((float)(TIM_1_8_PERIOD_CLOCKS)/(float)TIM_1_8_CLOCK_HZ)

int g_motorReportSampleRate = 1.0 / (100.0 * CURRENT_MEAS_PERIOD);  // The target rate is 100Hz

BSEMAPHORE_DECL(g_reportSampleReady,0); // 100Hz report loop

float g_shuntADCValue2Amps = 0.0;
float g_vbus_voltage = 12.0;
float g_currentZeroOffset[3] = { 0,0,0 } ;
float g_current[3] = { 0,0,0} ;
float g_phaseAngle = 0 ;

float g_driveTemperature = 0.0;
float g_motorTemperature = 0.0;

float g_current_Ibus = 0;
float g_motor_p_gain = 1.2;  // 1.2
float g_motor_i_gain = 0.0;  // 0.0

int g_phaseAngles[g_calibrationPointCount][3];
float g_phaseDistance[g_calibrationPointCount];

bool g_pwmThreadRunning = false;
volatile bool g_pwmRun = true;

int g_pwmTimeoutCount = 0 ;
bool g_pwmFullReport = false;
bool g_motorControlLoopReady = true;
bool g_lastLimitState = false;

float g_current_control_integral_d = 0;
float g_current_control_integral_q = 0;

static THD_WORKING_AREA(waThreadPWM, 512);

void PWMUpdateDrivePhase(int pa,int pb,int pc);

void SetupMotorCurrentPID(void);

static float sqrf(float v)
{ return v * v; }

int CheckHallInRange(void) {
  // Are sensor readings outside the expected range ?
  for(int i = 0;i < 3;i++) {
    if(g_hall[i] < 1000 || g_hall[i] > 3000)
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



void ShuntCalibration(void)
{
  // Enable shunt calibration mode.
  int gainMode = DRV8503_GAIN_CS1_40 | DRV8503_GAIN_CS2_40 | DRV8503_GAIN_CS3_40 |
      DRV8503_CS_BLANK_2_5US ;

  // The following can deal with +- 40 Amps or so

  float ampGain = 40.0; // V/V gain
  float shuntResistance = 0.001;

  // 90% of half the range of possible values.
  g_maxCurrentSense = (3.3 * 0.5 * 0.9) / (ampGain/shuntResistance);

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

float g_Ierr_d = 0;
float g_Ierr_q = 0;

// The following function is based on that from the ODrive project.

static bool FOC_current(float phaseAngle,float Id_des, float Iq_des) {


  // Clarke transform
  float Ialpha = -g_current[1] - g_current[2];
  float Ibeta = one_by_sqrt3 * (g_current[1] - g_current[2]);

  // Park transform
  float c = arm_cos_f32(phaseAngle);
  float s = arm_sin_f32(phaseAngle);
  g_Id = c*Ialpha + s*Ibeta;
  g_Iq = c*Ibeta  - s*Ialpha;

  g_torqueAverage = (g_torqueAverage * 30.0 - g_Iq)/31.0;

  // Current error
  g_Ierr_d = Id_des - g_Id;
  g_Ierr_q = Iq_des - g_Iq;

  // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
  // Apply PI control

  float Vd = g_current_control_integral_d + g_Ierr_d * g_motor_p_gain + Id_des * g_phaseResistance;
  float Vq = g_current_control_integral_q + g_Ierr_q * g_motor_p_gain + Iq_des * g_phaseResistance;

  float mod_to_V = (2.0f / 3.0f) * g_vbus_voltage;
  float V_to_mod = 1.0f / mod_to_V;

  float mod_d = V_to_mod * Vd;
  float mod_q = V_to_mod * Vq;

  // Vector modulation saturation, lock integrator if saturated
  // TODO make maximum modulation configurable
  float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f/sqrtf(mod_d*mod_d + mod_q*mod_q);
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

  // Dead time compensation.


  // Inverse park transform
  float mod_alpha = c*mod_d - s*mod_q;
  float mod_beta  = c*mod_q + s*mod_d;

  queue_modulation_timings(mod_alpha, mod_beta);

  return true;
}

float g_demandPhasePosition = 0;
float g_demandPhaseVelocity = 0;
float g_demandTorque = 0;

int g_phaseRotationCount = 0;
float g_currentPhasePosition = 0;
float g_currentPhaseVelocity = 0;
float g_velocityLimit = 4000.0; // Phase is radians a second.
float g_velocityPGain = 0.03;
float g_velocityIGain = 3.0;
float g_velocityISum = 0.0;
float g_velocityFilter = 2.0;
float g_positionGain = 5.0;
float g_currentLimit = 5.0;
float g_maxCurrentSense = 20.0;
float g_torqueAverage = 0.0;
float g_positionIGain = 0.0;
float g_positionIClamp = 5.0;
float g_positionISum = 0.0;
float g_Id = 0.0;
float g_Iq = 0.0;

bool g_gateDriverWarning = false;
bool g_gateDriverFault = false;

enum PWMControlDynamicT g_controlMode = CM_Brake;

static void UpdateCurrentMeasurementsFromADCValues(void) {
  // Compute motor currents;
  // Make sure they sum to zero
#if 1
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
#else
  //static float shuntFilter[3] = { 0.0,0.0,0.0 };
  for(int i = 0;i < 3;i++) {
    float newValue = ((float) g_currentADCValue[i] * g_shuntADCValue2Amps) - g_currentZeroOffset[i];
    g_current[i] = newValue;
  }
#endif
}

static float wrapAngle(float theta) {
    while (theta >= M_PI) theta -= (2.0f * M_PI);
    while (theta < -M_PI) theta += (2.0f * M_PI);
    return theta;
}


static void ComputeState(void)
{
  float rawPhase = hallToAngle(g_hall);

  // Compute current phase angle
  float lastAngle = g_phaseAngle;

  // PLL based position / velocity tracker.
  {
    const float pllBandwidth = 1000.0f; // [rad/s]

    static const float pllKp = 2.0f * pllBandwidth;
    static const float pllKi = 0.25f * pllKp * pllKp; // Critically damped
    static float pllPhase = 0;
    static float pllVel = 0;

    // predict PLL phase with velocity
    pllPhase = wrapAngle(pllPhase + CURRENT_MEAS_PERIOD * pllVel);
    float phaseError = wrapAngle(rawPhase - pllPhase);
    pllPhase = wrapAngle(pllPhase + CURRENT_MEAS_PERIOD * pllKp * phaseError);

    // update PLL velocity
    pllVel += CURRENT_MEAS_PERIOD * pllKi * phaseError;

    g_currentPhaseVelocity = pllVel;

    //g_phaseAngle = rawPhase; //pll_pos;
    g_phaseAngle = pllPhase;
  }

  // Update continuous angle.
  float angleDiff = lastAngle - g_phaseAngle;
  // If the change is large we have wrapped around.
  if(angleDiff > M_PI) {
    g_phaseRotationCount++;
  }
  if(angleDiff < -M_PI) {
    g_phaseRotationCount--;
  }

  // If we just sum up difference things will drift.
  g_currentPhasePosition = (float) g_phaseRotationCount * 2 * M_PI + g_phaseAngle;

  // Update currents
  UpdateCurrentMeasurementsFromADCValues();
}

static void SetCurrent(float current)
{

  if(current > g_currentLimit)
    current = g_currentLimit;
  if(current < -g_currentLimit)
    current = -g_currentLimit;

  //g_torqueAverage = (g_torqueAverage * 30.0 + torque)/31.0;
  FOC_current(g_phaseAngle,0,current);
}



static void MotorControlLoop(void)
{

  int loopCount = 0;
  g_motorControlLoopReady = true;

  int faultTimer = 0;

  while (g_pwmRun) {
    //palClearPad(GPIOB, GPIOB_PIN12); // Turn output off to measure timing
    if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
      g_pwmTimeoutCount++;
      continue;
    }

    ComputeState();

    if(!palReadPad(GPIOC, GPIOC_PIN15)) { // Fault pin
      faultTimer++;
      if(faultTimer > 1) {
        g_controlMode = CM_Fault;
        g_gateDriverFault = true;
      } else {
        // Signal warning.
        g_gateDriverWarning = true;
      }
    } else {
      g_gateDriverFault = false;
      faultTimer = 0;
    }

    float demandCurrent = g_demandTorque;
    float targetPosition = g_demandPhasePosition;

    switch(g_controlMode)
    {
      case CM_Off: // Maybe turn off the MOSFETS ?
        PWMUpdateDrivePhase(
            TIM_1_8_PERIOD_CLOCKS/2,
            TIM_1_8_PERIOD_CLOCKS/2,
            TIM_1_8_PERIOD_CLOCKS/2
            );
        g_torqueAverage = 0;
        break;
      case CM_Fault:
      case CM_Final:
        // Just turn everything off, this should mildly passively brake the motor
        PWMUpdateDrivePhase(
            TIM_1_8_PERIOD_CLOCKS/2,
            TIM_1_8_PERIOD_CLOCKS/2,
            TIM_1_8_PERIOD_CLOCKS/2
            );
        break;
      case CM_Brake:
        // Just turn everything off, this should passively brake the motor
        PWMUpdateDrivePhase(
            0,
            0,
            0
            );
        break;
      case CM_Position: {
        float positionError = (targetPosition - g_currentPhasePosition);
        float targetVelocity =  positionError * g_positionGain;

        if(targetVelocity > g_velocityLimit)
          targetVelocity = g_velocityLimit;
        if(targetVelocity < -g_velocityLimit)
          targetVelocity = -g_velocityLimit;

        g_demandPhaseVelocity = targetVelocity;
      }
      // no break
      case CM_Velocity: {
        //g_demandPhasePosition += g_demandPhaseVelocity * CURRENT_MEAS_PERIOD;
        //targetPosition = g_demandPhasePosition;

        float err = g_demandPhaseVelocity - g_currentPhaseVelocity;

        // Add a small deadzone.
        const float deadZone = M_PI/8.0f;
        if(err < 0) {
          if(err > -deadZone)
            err = 0;
          else
            err += deadZone;
        } else {
          if(err < deadZone)
            err = 0;
          else
            err -= deadZone;
        }

        g_velocityISum += err * CURRENT_MEAS_PERIOD * g_velocityIGain;
        if(g_velocityISum > g_velocityLimit)
          g_velocityISum = g_velocityLimit;
        if(g_velocityISum < -g_velocityLimit)
          g_velocityISum = -g_velocityLimit;

        demandCurrent = err * g_velocityPGain + g_velocityISum;

        SetCurrent(demandCurrent);

      } break;
      case CM_Torque: {
        SetCurrent(demandCurrent);
      } break;
    }

    // Check limit switches.
    {
      bool es3 = palReadPad(GPIOC, GPIOC_PIN8);
      if(es3 != g_lastLimitState) {
        g_lastLimitState = es3;
        MotionUpdateIndex(0,es3,g_currentPhasePosition,g_currentPhaseVelocity);
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
      if((pkt = USBGetEmptyPacket(TIME_IMMEDIATE)) != 0) {
        struct PacketPWMStateC *ps = (struct PacketPWMStateC *)&(pkt->m_data);
        pkt->m_len = sizeof(struct PacketPWMStateC);
        ps->m_packetType = CPT_PWMState;
        ps->m_tick = g_adcTickCount;
        for(int i = 0;i < 3;i++)
          ps->m_curr[i] = g_currentADCValue[i];
        for(int i = 0;i < 3;i++)
          ps->m_hall[i] = g_hall[i];
        ps->m_angle = g_phaseAngle * 65535.0 / (2.0 * M_PI);
        USBPostPacket(pkt);
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
  while (!palReadPad(GPIOD, GPIOD_PIN2) && g_pwmRun) {
    chThdSleepMilliseconds(100);
  }

  //! Read initial state of endstop switches
  g_lastLimitState = palReadPad(GPIOC, GPIOC_PIN8); // Index

  //! Make sure controller is setup.
  InitDrv8503();

  //! Wait a bit more
  chThdSleepMilliseconds(100);

  //! Check status.
  uint16_t status = Drv8503ReadRegister(DRV8503_REG_WARNING);

  // Check for any faults.
  if(status & DRV8503_WARN_FAULT) {
    palClearPad(GPIOC, GPIOC_PIN14); // Paranoid gate disable
    g_gateDriverWarning = true;
    g_gateDriverFault = true;
    g_pwmRun = false;
    g_pwmThreadRunning = false;
    return ;
  }

  // Double check with the fault bit.
  if(!palReadPad(GPIOC, GPIOC_PIN15)) { // Fault pin
    g_gateDriverWarning = true;
    g_gateDriverFault = false; // Should only be a warning.
  } else {
    //! Reset fault flags.
    g_gateDriverFault = false;
    g_gateDriverWarning = false;
  }

  // Setup PWM
  InitPWM();

  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  // Reset calibration state.
  MotionResetCalibration(MHS_Measuring);

  // This is quick, so may as well do it every time.
  ShuntCalibration();

  // Setup motor PID.
  SetupMotorCurrentPID();

  g_velocityISum = 0; // Reset velocity integral
  g_phaseRotationCount = 0; // Reset the rotation count to zero.

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
void SetModeBrake(void)
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

  // Make sure integrals are reset.
  g_current_control_integral_d = 0;
  g_current_control_integral_q = 0;

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
    return g_pwmThreadRunning;

  g_pwmRun = true;
  if(!g_pwmThreadRunning) {
    g_motorControlLoopReady = false;
    g_pwmThreadRunning = true;
    chThdCreateStatic(waThreadPWM, sizeof(waThreadPWM), NORMALPRIO+8, ThreadPWM, NULL);
    while(!g_motorControlLoopReady && g_pwmThreadRunning) {
      chThdSleepMilliseconds(10);
    }
  }
  return g_pwmThreadRunning;
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
  MotionResetCalibration(MHS_Lost);

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


#define CHECK_OVERCURRENT 1

enum FaultCodeT PWMSelfTest()
{
  EnableSensorPower(true);

  float rail5V = Read5VRailVoltage();
  if(rail5V < 4.5 || rail5V > 5.5)
    return FC_Internal5VRailOutOfRange;

  g_vbus_voltage = ReadSupplyVoltage();
  if(g_vbus_voltage > g_maxSupplyVoltage)
    return FC_OverVoltage;

#if CHECK_OVERCURRENT
  if(!palReadPad(GPIOB, GPIOB_PIN11)) { // Sensor over current fault
    EnableSensorPower(false);
    return FC_SensorOverCurrent;
  }
#endif

  //if(g_vbus_voltage < 8.0) return FC_UnderVoltage;

  {
    float sum = 0;
    for(int i = 0;i < 10;i++)
      sum += ReadDriveTemperature();
    g_driveTemperature = sum/10.0f;
  }
  if(g_driveTemperature > g_maxOperatingTemperature)
    return FC_DriverOverTemperature;

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
  int errCode = CheckHallInRange();
  if(errCode != FC_Ok)
    return errCode;

#if CHECK_OVERCURRENT
  // Check fan
  bool fanState = palReadPad(GPIOA, GPIOA_PIN7); // Read fan power.
  EnableFanPower(true);
  for(int i = 0;i < 10;i++) {
    if(!palReadPad(GPIOB, GPIOB_PIN11)) { // Fan fault
      EnableFanPower(false);
      return FC_FanOverCurrent;
    }
    chThdSleepMilliseconds(10);
  }
  // Restore fan state.
  EnableFanPower(fanState);

  // Double check sensor over current fault
  if(!palReadPad(GPIOB, GPIOB_PIN11)) {
    EnableSensorPower(false);
    return FC_SensorOverCurrent;
  }
#endif

  return FC_Ok;
}

enum FaultCodeT PWMMotorCalResistance(void);
enum FaultCodeT PWMMotorCalInductance(void);
enum FaultCodeT PWMMotorPhaseCal(void);

enum FaultCodeT PWMMotorCal()
{
  enum FaultCodeT ret = FC_Ok;

  palSetPad(GPIOC, GPIOC_PIN13); // Wake

  //! Wait for powerup to complete
  // TODO: Add a timeout and report and error
  while (!palReadPad(GPIOD, GPIOD_PIN2)) {
    chThdSleepMilliseconds(100);
  }

  // Make sure PWM is running.
  InitPWM();

  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable


  // Calibrate shunts.
  ShuntCalibration();

  if((ret = PWMMotorCalResistance()) != FC_Ok) {
    palClearPad(GPIOC, GPIOC_PIN14); // Gate disable
    return ret;
  }
  SendParamUpdate(CPI_MotorResistance);

  if((ret = PWMMotorCalInductance()) != FC_Ok) {
    palClearPad(GPIOC, GPIOC_PIN14); // Gate disable
    return ret;
  }
  SendParamUpdate(CPI_MotorInductance);

  // Setup motor controller parameters
  SetupMotorCurrentPID();
  SendParamUpdate(CPI_MotorPGain);

  if((ret = PWMMotorPhaseCal()) != FC_Ok) {
    palClearPad(GPIOC, GPIOC_PIN14); // Gate disable
    return ret;
  }
  palClearPad(GPIOC, GPIOC_PIN14); // Gate disable

  // Restore igain

  SendParamUpdate(CPI_MotorIGain);

  SaveSetup();

  return ret;
}

void SetupMotorCurrentPID()
{
  // Calculate current control gains
  float current_control_bandwidth = 1000.0f; // [rad/s]
  g_motor_p_gain = current_control_bandwidth * g_phaseInductance;

#if 1
  float plant_pole = g_phaseResistance / g_phaseInductance;
  g_motor_i_gain = plant_pole * g_motor_p_gain;
#else
  g_motor_i_gain = 0;
#endif
}


enum FaultCodeT PWMMotorCalResistance()
{
  enum FaultCodeT ret = FC_Ok;
  static const float kI = 10.0f; //[(V/s)/A]

  int cyclesPerSecond = 1.0 / (1.0 * CURRENT_MEAS_PERIOD);

  float targetCurrent = 5.0;
  float maxVoltage = 3.0;

  float testVoltage = 0;
  float actualCurrent = 0;//targetCurrent;

  g_phaseOffsetVoltage = 0;

#if 1
  for(int i = 0;i < cyclesPerSecond * 3;i++) {
    g_vbus_voltage = ReadSupplyVoltage();
    if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
      ret = FC_InternalTiming;
      break;
    }
    // Update in case we're monitoring currents from another thread.
    UpdateCurrentMeasurementsFromADCValues();

    float Ialpha = -(g_current[1] + g_current[2]);
    actualCurrent = actualCurrent * 0.99 + 0.01 * Ialpha;
    testVoltage += (kI * CURRENT_MEAS_PERIOD) * (targetCurrent - Ialpha);

    if (testVoltage > maxVoltage) testVoltage = maxVoltage;
    if (testVoltage < -maxVoltage) testVoltage = -maxVoltage;

    queue_voltage_timings(testVoltage, 0.0f);
  }

  if(ret == FC_Ok) {
    if(testVoltage >= maxVoltage || testVoltage <= -maxVoltage) {
      ret = FC_MotorResistanceOutOfRange;
      //return ret;
    }

    g_phaseResistance = testVoltage / actualCurrent;
    //g_phaseInductance = off;
  }

#else
  const int samples = 4;

  float voltage[samples];
  float current[samples];

//  g_vbus_voltage = ReadSupplyVoltage();
  for(int l = 0;l < samples;l++) {
    targetCurrent = 1.0 + l * 1.0;
    current[l] = targetCurrent;
    for(int i = 0;i < cyclesPerSecond * 1;i++) {
      g_vbus_voltage = ReadSupplyVoltage();
      if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
        ret = FC_InternalTiming;
        break;
      }
      UpdateCurrentMeasurementsFromADCValues();

      float Ialpha = -(g_current[1] + g_current[2]);
      actualCurrent = actualCurrent * 0.99 + 0.01 * Ialpha;
      testVoltage += (kI * CURRENT_MEAS_PERIOD) * (targetCurrent - Ialpha);

      if (testVoltage > maxVoltage) testVoltage = maxVoltage;
      if (testVoltage < -maxVoltage) testVoltage = -maxVoltage;

      queue_voltage_timings(testVoltage, 0.0f);
    }
    voltage[l] = testVoltage;
  }

  // De-energize motor
  queue_voltage_timings(0.0f, 0.0f);

  // x = i;
  // y = v;
  // y = a  + r * i
  float meanV = 0;
  float meanI = 0;
  for(int i = 0;i < samples;i++) {
    meanV += voltage[i];
    meanI += current[i];
  }
  meanV /= samples;
  meanI /= samples;

  float sumI2 = 0;
  float sumIV = 0;
  for(int i = 0;i < samples;i++) {
    sumI2 += sqrf(current[i] - meanI);
    sumIV += (current[i] - meanI) * (voltage[i] - meanV);
  }

  float r = sumIV / sumI2;
  float off = meanV - r * meanI;

  g_phaseResistance = r;// / targetCurrent;
  g_phaseOffsetVoltage = off;

  if(testVoltage >= maxVoltage || testVoltage <= -maxVoltage) {
    ret = FC_MotorResistanceOutOfRange;
  }
#endif
  return ret;
}

enum FaultCodeT PWMMotorCalInductance()
{
  enum FaultCodeT ret = FC_Ok;

  float voltage_low = -1;
  float voltage_high = 1;
  float Ialphas[2] = {0.0f,0.0f};
  static const int num_cycles = 5000;

  for (int t = 0; t < num_cycles; ++t) {
    for (int i = 0; i < 2; ++i) {
      if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
        ret = FC_InternalTiming;
        break;
      }
      UpdateCurrentMeasurementsFromADCValues();
      Ialphas[i] += -(g_current[1] + g_current[2]);

      // Test voltage along phase A
      queue_voltage_timings( i != 0 ? (voltage_low - g_phaseOffsetVoltage): (voltage_high+g_phaseOffsetVoltage), 0.0f);
    }
  }

  // De-energize motor
  queue_voltage_timings(0.0f, 0.0f);

  float v_L = 0.5f * (voltage_high - voltage_low);
  // Note: A more correct formula would also take into account that there is a finite timestep.
  // However, the discretisation in the current control loop inverts the same discrepancy
  float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (CURRENT_MEAS_PERIOD * (float)num_cycles);
  float L = v_L / dI_by_dt;

  g_phaseInductance = L;

  // TODO arbitrary values set for now
  if (L < 1e-6f || L > 1e-3f) {
    ret = FC_MotorInducetanceOutOfRange;
    return ret;
  }

  return ret;
}

enum FaultCodeT PWMMotorPhaseCal()
{
  enum FaultCodeT ret = FC_Ok;

  // This assumes the motor is powered up, and the current shunts are calibrated.

  // Make sure PWM is running.
  InitPWM();

  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  int phaseRotations = 7;
  int numberOfReadings = 8;

  int cyclesPerSecond = 1.0 / (1.0 * CURRENT_MEAS_PERIOD);

  float torqueValue = 3.0;
  float lastAngle = 0;

  g_vbus_voltage = ReadSupplyVoltage();

  // Turn up torque slowly until we're at the initial angle

  for(int i = 0;i < cyclesPerSecond;i++) {
    if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
      ret = FC_InternalTiming;
      break;
    }
    UpdateCurrentMeasurementsFromADCValues();
    float tv = (float) i * torqueValue / (float) cyclesPerSecond;
    FOC_current(lastAngle,tv,0);
  }

  for(int phase = 0;phase < g_calibrationPointCount*phaseRotations && ret == FC_Ok;phase++) {
    g_vbus_voltage = ReadSupplyVoltage();

    int phaseStep = phase % g_calibrationPointCount;
    float phaseAngle = (float) phase * M_PI * 2.0 / ((float) g_calibrationPointCount) ;

    int shiftPeriod = cyclesPerSecond / 12; // 8 is ok

    // Move to position slowly to reduce vibration

    if(phaseAngle != lastAngle) {
      for(int i = 0;i < shiftPeriod;i++) {
        if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
          ret = FC_InternalTiming;
          break;
        }
        UpdateCurrentMeasurementsFromADCValues();
        float fract= (float) i / (float) shiftPeriod;
        float targetAngle = lastAngle * (1.0 - fract) + phaseAngle * fract;
        FOC_current(targetAngle,torqueValue,0);
      }
    }
    lastAngle = phaseAngle;

    g_vbus_voltage = ReadSupplyVoltage();
    // Settle

    for(int i = 0;i < shiftPeriod;i++) {
      if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
        ret = FC_InternalTiming;
        break;
      }
      UpdateCurrentMeasurementsFromADCValues();

      FOC_current(phaseAngle,torqueValue,0);
    }

    g_vbus_voltage = ReadSupplyVoltage();
    // Take some readings.

    for(int i = 0;i < numberOfReadings;i++) {
      if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
        ret = FC_InternalTiming;
        break;
      }
      UpdateCurrentMeasurementsFromADCValues();

      FOC_current(phaseAngle,torqueValue,0);

      for(int i = 0;i < 3;i++) {
        g_phaseAngles[phaseStep][i] += g_hall[i];
      }
    }
  }

  if(ret == FC_Ok) {
    for(int i = 0;i < g_calibrationPointCount;i++) {
      g_phaseAngles[i][0] /= phaseRotations * numberOfReadings;
      g_phaseAngles[i][1] /= phaseRotations * numberOfReadings;
      g_phaseAngles[i][2] /= phaseRotations * numberOfReadings;
    }
  }

  palClearPad(GPIOC, GPIOC_PIN14); // Gate disable

  //DisplayAngle(chp);
  return ret;
}




int PWMCalSVM(BaseSequentialStream *chp)
{
  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  float voltage_magnitude = 0.06;

  for(int phase = 0;phase <  g_calibrationPointCount*7;phase++) {
    int phaseStep = phase % g_calibrationPointCount;
    float phaseAngle = (float) phase * M_PI * 2.0 / ((float) g_calibrationPointCount);
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
  for(int i = 0;i < g_calibrationPointCount;i++) {
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
  int distTable[g_calibrationPointCount];
  int phase = 0;

  int minDist = sqr(g_phaseAngles[0][0] - sensors[0]) +
                sqr(g_phaseAngles[0][1] - sensors[1]) +
                sqr(g_phaseAngles[0][2] - sensors[2]);
  distTable[0] = minDist;

  for(int i = 1;i < g_calibrationPointCount;i++) {
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
  if(last < 0) last = g_calibrationPointCount;
  int next = phase + 1;
  if(next >= g_calibrationPointCount) next = 0;
  int lastDist2 = distTable[last];
  int nextDist2 = distTable[next];
  float angle = phase * 2.0;
  float lastDist = mysqrtf(lastDist2) / g_phaseDistance[phase];
  float nextDist = mysqrtf(nextDist2) / g_phaseDistance[next];
  angle -= (nextDist-lastDist)/(nextDist + lastDist);
  const float calibRange = g_calibrationPointCount*2.0f;
  if(angle < 0.0) angle += calibRange;
  if(angle > calibRange) angle -= calibRange;
  return (angle * M_PI * 2.0 / calibRange) + 1.04;
}

float g_hallToAngleOriginOffset = -2000;
float g_phaseAnglesNormOrg[g_calibrationPointCount][3];

void InitHall2Angle(void)
{
  {
    // Pre-compute the distance between this position and the last.
    int lastIndex = g_calibrationPointCount-1;
    for(int i = 0;i < g_calibrationPointCount;i++) {
      int sum = 0;
      for(int k = 0;k < 3;k++) {
        int diff = g_phaseAngles[i][k] - g_phaseAngles[lastIndex][k];
        sum += diff * diff;
      }
      g_phaseDistance[i] = mysqrtf((float) sum);
      lastIndex = i;
    }
  }

  for(int i = 0;i < g_calibrationPointCount;i++) {
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
  float distTable[g_calibrationPointCount];
  float norm[3];
  norm[0] = (float) sensors[0] - g_hallToAngleOriginOffset;
  norm[1] = (float) sensors[1] - g_hallToAngleOriginOffset;
  norm[2] = (float) sensors[2] - g_hallToAngleOriginOffset;
  float mag = mysqrtf(sqr(norm[0]) + sqr(norm[1]) + sqr(norm[2]));
  // This shouldn't happen, but just in case of some extreme noise, avoid returning a NAN.
  if(mag == 0) {
    // Log an error?
    return g_phaseAngle;
  }

  for(int j = 0;j < 3;j++)
    norm[j] /= mag;

  //RavlDebug("Vec: %f %f %f",norm[0],norm[1],norm[2]);

  //mag = mysqrtf(sqr(g_phaseAngles[0][0]) + sqr(g_phaseAngles[0][1]) + sqr(g_phaseAngles[0][2]));

  float maxCorr = ((g_phaseAnglesNormOrg[0][0]) * norm[0]) +
                  ((g_phaseAnglesNormOrg[0][1]) * norm[1]) +
                  ((g_phaseAnglesNormOrg[0][2]) * norm[2]);

  distTable[0] = maxCorr;
  int phase = 0;

  for(int i = 1;i < g_calibrationPointCount;i++) {
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
  if(last < 0) last = g_calibrationPointCount-1;
  int next = phase + 1;
  if(next >= g_calibrationPointCount) next = 0;
  float lastDist2 = distTable[last];
  float nextDist2 = distTable[next];
  float angle = phase * 2.0;
  float lastDist = maxCorr - lastDist2;
  float nextDist = maxCorr - nextDist2;
  //Average error:0.007239  Abs:0.207922 Mag:0.263140
  float corr = (nextDist-lastDist)/(nextDist + lastDist);
  angle -= corr;
  //RavlDebug("Last:%f  Max:%f Next:%f Corr:%f ",lastDist,maxCorr,nextDist,corr);
  const float calibRange = g_calibrationPointCount*2.0f;
  //if(angle < -g_calibrationPointCount) angle += calibRange;
  if(angle > g_calibrationPointCount) angle -= calibRange;
  return (angle * M_PI * 2.0 / calibRange);
}


float hallToAngle(uint16_t *sensors)
{
  return hallToAngleDot2(sensors);
}



