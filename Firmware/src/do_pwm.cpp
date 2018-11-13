

#include <stdint.h>
#include "hal.h"
#include "pwm.h"
#include "mathfunc.h"
#include "chprintf.h"
#include "svm.h"

#include "coms.h"
#include "dogbot/protocol.h"
#include "drv8320.h"

#include "storedconf.h"

#include "motion.h"


#define SYSTEM_CORE_CLOCK     168000000

#include "stm32f4xx_adc.h"

const float g_pllBandwidth = 1000.0f; // [rad/s]

float g_debugValue = 0;

float g_maxSupplyVoltage = 40.0;
float g_maxOperatingTemperature = 75.0;

float g_phaseResistance = 0.002;
float g_phaseOffsetVoltage = 0.1;
float g_phaseInductance = 1e-9;

#define TIM_1_8_CLOCK_HZ (SYSTEM_CORE_CLOCK/4)
//#define TIM_1_8_PERIOD_CLOCKS (2100)   // 20 KHz
#define TIM_1_8_PERIOD_CLOCKS (2333)    // 18 KHz
//#define TIM_1_8_PERIOD_CLOCKS (2625)  // 16 KHz
//#define TIM_1_8_PERIOD_CLOCKS (4095)  // 10 KHz
#define CURRENT_MEAS_PERIOD ((float)(TIM_1_8_PERIOD_CLOCKS)/(float)TIM_1_8_CLOCK_HZ)

const float g_defaultReportRate = 250.0;

int g_motorReportSampleCount = (1.0 / (g_defaultReportRate * CURRENT_MEAS_PERIOD)) + 0.5f;
float g_PWMFrequency = 1.0/CURRENT_MEAS_PERIOD;

BSEMAPHORE_DECL(g_reportSampleReady,0); // Synchronise motion report loop (Normally 100Hz )

float g_shuntADCValue2Amps = 0.0;
float g_vbus_voltage = 3.0;
float g_currentZeroOffset[3] = { 0,0,0 } ;
float g_current[3] = { 0,0,0} ;
float g_phaseAngle = 0 ;

float g_driveTemperature = 0.0;
float g_motorTemperature = 0.0;

float g_current_Ibus = 0;
float g_motor_p_gain = 1.2;  // 1.2
float g_motor_i_gain = 0.0;  // 0.0

float g_lastVoltageAlpha = 0;
float g_lastVoltageBeta = 0;

int g_phaseAngles[g_calibrationPointCount][3];
float g_phaseDistance[g_calibrationPointCount];

bool g_pwmThreadRunning = false;
volatile bool g_pwmRun = false;

int g_pwmTimeoutCount = 0 ;
bool g_pwmFullReport = false;
bool g_motorControlLoopReady = true;
bool g_lastLimitState = false;

bool g_hitLimitVelocity = false;
bool g_hitLimitTorque = false;
bool g_hitLimitPosition = false;

float g_current_control_integral_d = 0;
float g_current_control_integral_q = 0;

#if ENABLE_ANGLESTATS
bool g_enableAngleStats = false;
float g_angleStats[g_angleTableSize][2];
#endif

static THD_WORKING_AREA(waThreadPWM, 512);

void EnableGateDriver(bool enable)
{
  if(enable) {
    palSetPad(DRIVE_GATE_ENABLE_GPIO_Port, DRIVE_GATE_ENABLE_Pin);
  } else {
    palClearPad(DRIVE_GATE_ENABLE_GPIO_Port, DRIVE_GATE_ENABLE_Pin);
  }
}

void EnableGateOutputs(bool enable)
{
  if(enable) {
#if USE_PWM3
    palSetPad(DRIVE_INLA_GPIO_Port,DRIVE_INLA_Pin);
    palSetPad(DRIVE_INHB_GPIO_Port,DRIVE_INHB_Pin);
    palSetPad(DRIVE_INLC_GPIO_Port,DRIVE_INLC_Pin);
#else
    uint16_t driveControl = Drv8320ReadRegister(DRV8320_REG_DRIVE_CONTROL);
    Drv8320SetRegister(DRV8320_REG_DRIVE_CONTROL,driveControl & ~DRV8320_DC_COAST);
#endif
  } else {
#if USE_PWM3
    palClearPad(DRIVE_INLA_GPIO_Port,DRIVE_INLA_Pin);
    palClearPad(DRIVE_INHB_GPIO_Port,DRIVE_INHB_Pin);
    palClearPad(DRIVE_INLC_GPIO_Port,DRIVE_INLC_Pin);
#else
    uint16_t driveControl = Drv8320ReadRegister(DRV8320_REG_DRIVE_CONTROL);
    Drv8320SetRegister(DRV8320_REG_DRIVE_CONTROL,driveControl | DRV8320_DC_COAST);
#endif
  }
}

bool IsGateDriverFault()
{
  return palReadPad(DRIVE_FAULT_GPIO_Port,DRIVE_FAULT_Pin) == 0;
}



void PWMUpdateDrivePhase(int pa,int pb,int pc);

static bool SensorlessEstimatorUpdate(float *eta);

void SetupMotorCurrentPID(void);

// based on https://math.stackexchange.com/a/1105038/81278
#define MACRO_MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MACRO_MIN(x, y) (((x) < (y)) ? (x) : (y))

static float fast_atan2(float y, float x)
{
  // a := min (|x|, |y|) / max (|x|, |y|)
  float abs_y = fabsf(y);
  float abs_x = fabsf(x);
  float a = MACRO_MIN(abs_x, abs_y) / MACRO_MAX(abs_x, abs_y);
  //s := a * a
  float s = a * a;
  //r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
  float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  //if |y| > |x| then r := 1.57079637 - r
  if (abs_y > abs_x)
      r = 1.57079637f - r;
  // if x < 0 then r := 3.14159274 - r
  if (x < 0.0f)
      r = 3.14159274f - r;
  // if y < 0 then r := -r
  if (y < 0.0f)
      r = -r;

  return r;
}

// Set the report rate.
// It must be a integer multiple of
bool SetServoReportRate(float rate)
{
  if(rate == 0)
    return false;
  float newValue = (1.0f/(rate * CURRENT_MEAS_PERIOD));
  if(newValue < 1 || newValue > ((int32_t)1<<30))
    return false;
  g_motorReportSampleCount = newValue + 0.5f; // Round to nearest integer value.
  return true;
}

// Compute the current report rate.

float GetServoReportRate(void)
{
  if(g_motorReportSampleCount == 0)
    return 0;
  return 1.0f/((float) g_motorReportSampleCount * CURRENT_MEAS_PERIOD);
}


FaultCodeT CheckHallInRange(void) {
  // Are sensor readings outside the expected range ?
  for(int i = 0;i < 3;i++) {
    if(g_hall[i] < 1000 || g_hall[i] > 3000)
      return FC_NoSensor;
  }
  return FC_Ok;
}


static void queue_modulation_timings(float mod_alpha, float mod_beta,bool unsymetricSwitching) {
  float tA = 0, tB = 0, tC = 0;
  SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
  uint16_t a = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
  uint16_t b = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
  uint16_t c = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);

#if 1
  if(unsymetricSwitching) {
    uint16_t min = a;
    if(b < min) min = b;
    if(c < min) min = c;
    a -= min;
    b -= min;
    c -= min;

    // If the pulse is too short it won't actually switch the MOSFETS
    // so it's better to do nothing.
    // FIXME:- Measure the switching time and use that.
    if(a < 50) a = 0;
    if(b < 50) b = 0;
    if(c < 50) c = 0;
  }
#endif

  PWMUpdateDrivePhase(a,b,c);
}

static void queue_voltage_timings(float v_alpha, float v_beta,bool unsymetricSwitching) {
  float vfactor = 1.0f / ((2.0f / 3.0f) * g_vbus_voltage);
  float mod_alpha = vfactor * v_alpha;
  float mod_beta = vfactor * v_beta;
  queue_modulation_timings(mod_alpha, mod_beta,unsymetricSwitching);
}



void ShuntCalibration(void)
{

  float ampGain = 20.0; // V/V gain
  float shuntResistance = 0.0005;

  // 95% of half the range of possible values.
  g_maxCurrentSense = (3.3 * 0.5 * 0.95)/ (shuntResistance * ampGain);
  SendParamUpdate(CPI_MaxCurrentSense);

  // Disable outputs
  // FIXME:- SetRegister should return the old values, so two calls
  // are not needed.  Need to double check though
  uint16_t driveControl = Drv8320ReadRegister(DRV8320_REG_DRIVE_CONTROL);
  Drv8320SetRegister(DRV8320_REG_DRIVE_CONTROL,driveControl | DRV8320_DC_COAST);

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

  int samples = 64;
  for(int i = 0;i < samples;i++) {
    chBSemWait(&g_adcInjectedDataReady);
    for(int j = 0;j < 3;j++)
      sums[j] += ((float) g_currentADCValue[j]) * g_shuntADCValue2Amps ;
  }

  for(int j = 0;j < 3;j++)
    g_currentZeroOffset[j] = sums[j] / (float) samples;

  // Restore control state.
  Drv8320SetRegister(DRV8320_REG_DRIVE_CONTROL,driveControl);

}

float g_Ierr_d = 0;
float g_Ierr_q = 0;

// Monitor currents when we're not driving the motor directly, normally during breaking

static bool Monitor_FOC_Current(float phaseAngle)
{
  // Clarke transform
  float Ialpha = -g_current[1] - g_current[2];
  float Ibeta = one_by_sqrt3 * (g_current[1] - g_current[2]);

  // Park transform
  float c = arm_cos_f32(phaseAngle);
  float s = arm_sin_f32(phaseAngle);
  g_Id = c*Ialpha + s*Ibeta;
  g_Iq = c*Ibeta  - s*Ialpha;

  g_torqueAverage += (g_Iq - g_torqueAverage)* CURRENT_MEAS_PERIOD * g_defaultReportRate;

  g_Ierr_d = 0;
  g_Ierr_q = 0;

  return true;
}

// The following function is based on that from the ODrive project.

static bool FOC_current(float phaseAngle,float Id_des, float Iq_des)
{
  // Clarke transform
  float Ialpha = -g_current[1] - g_current[2];
  float Ibeta = one_by_sqrt3 * (g_current[1] - g_current[2]);

  // Park transform
  float c = arm_cos_f32(phaseAngle);
  float s = arm_sin_f32(phaseAngle);
  g_Id = c*Ialpha + s*Ibeta;
  g_Iq = c*Ibeta  - s*Ialpha;

  g_torqueAverage += (g_Iq - g_torqueAverage)* CURRENT_MEAS_PERIOD * g_defaultReportRate;

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
  // TBD

  // Inverse park transform
  float mod_alpha = c*mod_d - s*mod_q;
  float mod_beta  = c*mod_q + s*mod_d;

#if 0
  // In general it is more efficient to reduce switching keeping the current running through the low
  // side MOSFETs reduces switching losses and makes things more efficient.   The down side of this
  // is that all the heat is dissipated in in the component which may over heat.  To mitigate this
  // we change to symmetric switching when the current gets too high for one MOSFET to handle.

  static bool lastSwitchState = true;

  // Add some hysteresis as the two methods offer a different average resistance.
  float current = fabs(Id_des) + fabs(Iq_des);
  bool useUnsymetricSwitching;
  if(lastSwitchState) {
    useUnsymetricSwitching = current < 6.0;
  } else {
    useUnsymetricSwitching = current < 5.0;
  }
  lastSwitchState = useUnsymetricSwitching;

  queue_modulation_timings(mod_alpha, mod_beta,useUnsymetricSwitching);
#else
  queue_modulation_timings(mod_alpha, mod_beta,false);
#endif

  // Report final applied voltage in stationary frame (for sensorless estimator)

  g_lastVoltageAlpha = mod_to_V * mod_alpha;
  g_lastVoltageBeta = mod_to_V * mod_beta;

  return true;
}

MUTEX_DECL(g_demandMutex);
float g_demandPhasePosition = 0;
float g_demandPhaseVelocity = 0;
float g_demandTorque = 0;

bool g_endStopEnable = false;
float g_endStopMin = 0;
float g_endStopMax = 0;
float g_endStopTargetBreakCurrent = 3;
float g_endStopMaxBreakCurrent = 6;
float g_endStopPhaseMin = 0;
float g_endStopPhaseMax = 0;


int g_phaseRotationCount = 0;
float g_currentPhasePosition = 0;
float g_currentPhaseVelocity = 0;
float g_phaseVelocityFilter = CURRENT_MEAS_PERIOD * g_defaultReportRate;
float g_filteredPhaseVelocity = 0;
float g_velocityLimit = 4000.0; // Phase velocity in radians a second.
float g_velocityPGain = 0.12;
float g_velocityIGain = 12.0;
float g_velocityISum = 0.0;
float g_velocityFilter = 2.0;
float g_positionGain = 5.0;
float g_currentLimit = 5.0;    // Current limit from last request
float g_userCurrentLimit = 5.0; // Current limit set as parameter
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

// Do some sanity checks on the setup.
// FIXME:- we should include a way of reporting source of the fault.

bool CheckMotorSetup(void)
{

  if(g_endStopEnable) {
    if(g_endStopMax <= g_endStopMin) {
      return false;
    }
    if(g_endStopMaxBreakCurrent < g_endStopTargetBreakCurrent) {
      return false;
    }
    if(g_endStopMaxBreakCurrent > g_maxCurrentSense) {
      return false;
    }
    if(g_endStopTargetBreakCurrent > g_maxCurrentSense) {
      return false;
    }
  }
  if(g_velocityLimit > 6000 || g_velocityLimit < 0) {
    return false;
  }

  if(g_currentLimit  > g_maxCurrentSense) {
    return false;
  }

  return true;
}



static void UpdateCurrentMeasurementsFromADCValues(void) {
#if 1
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

static void FastSinCos(float x,float &sin,float &cos)
{
  if (x < -3.14159265)
      x += 6.28318531;
  else
  if (x >  3.14159265)
      x -= 6.28318531;

  //compute sine
  if (x < 0)
  {
      sin = 1.27323954 * x + .405284735 * x * x;

      if (sin < 0)
          sin = .225 * (sin *-sin - sin) + sin;
      else
          sin = .225 * (sin * sin - sin) + sin;
  }
  else
  {
      sin = 1.27323954 * x - 0.405284735 * x * x;

      if (sin < 0)
          sin = .225 * (sin *-sin - sin) + sin;
      else
          sin = .225 * (sin * sin - sin) + sin;
  }

  //compute cosine: sin(x + PI/2) = cos(x)
  x += 1.57079632;
  if (x >  3.14159265)
      x -= 6.28318531;

  if (x < 0)
  {
      cos = 1.27323954 * x + 0.405284735 * x * x;

      if (cos < 0)
          cos = .225 * (cos *-cos - cos) + cos;
      else
          cos = .225 * (cos * cos - cos) + cos;
  }
  else
  {
      cos = 1.27323954 * x - 0.405284735 * x * x;

      if (cos < 0)
          cos = .225 * (cos *-cos - cos) + cos;
      else
          cos = .225 * (cos * cos - cos) + cos;
  }
}


static void ComputeState(void)
{
  float rawPhase = hallToAngle(g_hall);

  // Compute current phase angle
  float lastAngle = g_phaseAngle;

  // PLL based position / velocity tracker.
  {
    static const float pllKp = 2.0f * g_pllBandwidth;
    static const float pllKi = 0.25f * pllKp * pllKp; // Critically damped
    static float pllPhase = 0;
    static float pllVel = 0;

#if ENABLE_ANGLESTATS
    float eta[2];
    if(SensorlessEstimatorUpdate(eta)) {

#if 0
      static bool useSenorless = false;
      if(useSenorless && fabs(g_currentPhaseVelocity) < 20) {
        useSenorless = false;
      } else if(fabs(g_currentPhaseVelocity) > 40) {
        useSenorless = true;
      }
      if(g_debugValue > 0 && useSenorless) {
        const float phaseError = 0.65;
        // Average the senseless feedback with hall data, the faster we go
        // the larger the eta values are and the more reliable they are.

        // FIXME :- We need a scaling factor that better reflects the confidence of the
        // motor position.
        // Scaling factor based on the magnitude of eta[] when the motor is stationary.
        float scale = g_debugValue/0.00188590826924;

        float psin,pcos;
        FastSinCos(rawPhase+phaseError,psin,pcos);
        rawPhase = fast_atan2(eta[1] * scale+ psin, eta[0] * scale + pcos)-phaseError;
      }
#endif
    }
#endif

    // predict PLL phase with velocity
    pllPhase = wrapAngle(pllPhase + CURRENT_MEAS_PERIOD * pllVel);
    float phaseError = wrapAngle(rawPhase - pllPhase);
    pllPhase = wrapAngle(pllPhase + CURRENT_MEAS_PERIOD * pllKp * phaseError);

    // update PLL velocity
    pllVel += CURRENT_MEAS_PERIOD * pllKi * phaseError;

    g_currentPhaseVelocity = pllVel;
    g_filteredPhaseVelocity += (g_currentPhaseVelocity - g_filteredPhaseVelocity) * g_phaseVelocityFilter;
    g_phaseAngle = pllPhase;

#if ENABLE_ANGLESTATS
    if(fabs(g_currentPhaseVelocity) > 100.0 && g_enableAngleStats) {
      int angleBin = ((g_phaseAngle + M_PI) * (float) g_angleTableSize / (2.0f*M_PI));
      if(angleBin < 0) angleBin = 0;
      if(angleBin >= g_angleTableSize) angleBin = g_angleTableSize-1;
      g_angleStats[angleBin][0] += (eta[0] - g_angleStats[angleBin][0]) * 0.001;
      g_angleStats[angleBin][1] += (eta[1] - g_angleStats[angleBin][1]) * 0.001;
    }
#endif


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

  // If we just sum up difference things will drift, so we compute position.
  g_currentPhasePosition = (float) g_phaseRotationCount * 2 * M_PI + g_phaseAngle;

  // Update currents
  UpdateCurrentMeasurementsFromADCValues();
}



static bool SetCurrent(float current)
{
  bool ret = false;
  if(current > g_currentLimit) {
    g_hitLimitTorque = true;
    ret = true;
    current = g_currentLimit;
  }
  if(current < -g_currentLimit) {
    g_hitLimitTorque = true;
    ret = true;
    current = -g_currentLimit;
  }

  //g_torqueAverage = (g_torqueAverage * 30.0 + torque)/31.0;
  FOC_current(g_phaseAngle,0,current);
  return ret;
}

static float max(float v1,float v2)
{ return v1 > v2 ? v1 : v2; }

static float min(float v1,float v2)
{ return v1 < v2 ? v1 : v2; }

// Check endstop limits, return true if everything is ok.

static bool MotorCheckEndStop(float demandCurrent)
{
  // Check virtual end stops.

  // If we're not homed or endStops are disabled then do nothing.
  if(!g_endStopEnable || g_motionHomedState != MHS_Homed) {
    return true;
  }

  // We could do something fancy here, but it is better to keep it simple to reduce the chance of unexpected behaviour.
  if(g_currentPhasePosition < g_endStopPhaseMin) {
    g_hitLimitPosition = true;
    if(g_currentPhaseVelocity < 0) {
      if(demandCurrent > g_currentLimit)  // If we're using the demand, respect the currentLimit
        demandCurrent = g_currentLimit;
      // Set current directly to bypass limits, if limit is less than the breaking current.
      FOC_current(g_phaseAngle,0,max(g_endStopTargetBreakCurrent,demandCurrent));
      return false;
    }
  }
  if(g_currentPhasePosition > g_endStopPhaseMax) {
    g_hitLimitPosition = true;
    if(g_currentPhaseVelocity > 0) {
      if(demandCurrent < -g_currentLimit) // If we're using the demand, respect the currentLimit
        demandCurrent = -g_currentLimit;
      // Set current directly to bypass limits, if limit is less than the breaking current.
      FOC_current(g_phaseAngle,0,min(-g_endStopTargetBreakCurrent,demandCurrent));
      return false;
    }
  }
  return true;
}

static void MotorControlLoop(void)
{

  int loopCount = 0;
  g_motorControlLoopReady = true;

  //int faultTimer = 0;

  while (g_pwmRun) {
    //palClearPad(GPIOB, GPIOB_PIN12); // Turn output off to measure timing
    if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
      g_pwmTimeoutCount++;
      continue;
    }

    ComputeState();

#if 0
    // The pulse length of 58us is very close to the PWM timing.
    // This means warnings can look like errors due to aliasing with the
    // warning pulses. We poll the register at 250Hz, and it is not
    // clear that responding faster to errors brings any benefit as
    // the chip shuts down itself in the case of a fault.
    if(!IsGateDriverFault()) { // Check the fault pin
      faultTimer++;
      if(faultTimer > 3) {
        SetMotorControlMode(CM_Brake);
        g_gateDriverFault = true;
      } else {
        // Signal warning.
        g_gateDriverWarning = true;
      }
    } else {
      g_gateDriverFault = false;
      faultTimer = 0;
    }
#endif


    static float localDemandTorque = 0;
    static float localDemandPosition = 0;
    static float localDemandVelocity = 0;
    static int localMotionTime = 0;


    if(g_motionPositionTime < 100000) // Protect against wrap around
      g_motionPositionTime++; // Used for working out the frequency of motion requests
    if(localMotionTime < 100000) // Protect against wrap around
      localMotionTime++;

    // Try and lock the mutex to update state variables, if we fail just use the
    // same values from the last pwm cycle and pick up changes next time around.
    // This avoid blocking in this loop which we definitely don't want to do.
    if(chMtxTryLock(&g_demandMutex)) {
      localDemandTorque = g_demandTorque;
      localDemandPosition = g_demandPhasePosition;
      localDemandVelocity = g_demandPhaseVelocity;
      localMotionTime = g_motionPositionTime;
      chMtxUnlock(&g_demandMutex);

    }

    float demandCurrent = localDemandTorque;
    float targetPosition = localDemandPosition;
    float targetVelocity = localDemandVelocity;

    switch(g_controlMode)
    {
      case CM_Off: // Maybe turn off the MOSFETS ?
        // FIXME:- This doesn't actually turn the MOSFETs off, but the switching losses
        // stop too much breaking.

#if 0
        // Zero out and drift in motor current readings.
        if(fabs(g_currentPhaseVelocity) < 15) {
          for(int i = 0;i < 3;i++) {
            g_currentZeroOffset[i] = (((float) g_currentADCValue[i] * g_shuntADCValue2Amps) - g_currentZeroOffset[i]) * 0.005;
          }
        }
#endif

        PWMUpdateDrivePhase(
            TIM_1_8_PERIOD_CLOCKS/2,
            TIM_1_8_PERIOD_CLOCKS/2,
            TIM_1_8_PERIOD_CLOCKS/2
            );
        // Monitor the currents but don't drive them
        Monitor_FOC_Current(g_phaseAngle);
        break;
      case CM_Fault:
      case CM_Final:
        // Just turn on the low side MOSFETs, this should passively brake the motor
        PWMUpdateDrivePhase(
            0,
            0,
            0
            );
        g_torqueAverage = 0;
        break;
      case CM_Brake:
        // Just turn on the low side MOSFETs, this should passively brake the motor
        // as there are no switching losses this applies more breaking force.
        PWMUpdateDrivePhase(
            0,
            0,
            0
            );
        // Monitor the currents but don't drive them
        Monitor_FOC_Current(g_phaseAngle);
        break;
      case CM_Position: {
        if(localMotionTime > g_motionTimeOut) {
          localMotionTime = g_motionTimeOut;
          localDemandVelocity = 0;
        }
        if(g_motionUpdatePeriod != 0 && localDemandVelocity != 0)
          targetPosition += (localDemandVelocity * (float) localMotionTime) * CURRENT_MEAS_PERIOD;
        float positionError = (targetPosition - g_currentPhasePosition);
        targetVelocity = localDemandVelocity + positionError * g_positionGain;
      }
      // no break
      case CM_Velocity: {

        // Apply velocity limits
        if(targetVelocity > g_velocityLimit)
          targetVelocity = g_velocityLimit;
        if(targetVelocity < -g_velocityLimit)
          targetVelocity = -g_velocityLimit;

        float err = targetVelocity - g_currentPhaseVelocity;

        // Add a small dead zone.  This reduces noise and small oscillations
        // coupled through from the sensors
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

        demandCurrent = err * g_velocityPGain + g_velocityISum;

        // Do this before integration so we don't fight it.
        if(!MotorCheckEndStop(demandCurrent))
          break;

        if(!SetCurrent(demandCurrent)) {
          // We're not saturating, so do the integration.
          g_velocityISum += err * CURRENT_MEAS_PERIOD * g_velocityIGain;
        } else {
          // Control is saturating, so just slowly unwind things until it isn't.
          g_velocityISum *= 0.99f;
        }

      } break;
      case CM_Torque: {
        if(!MotorCheckEndStop(demandCurrent))
          break;

        if(demandCurrent > g_currentLimit) {
          g_hitLimitTorque = true;
          demandCurrent = g_currentLimit;
        }
        if(demandCurrent < -g_currentLimit) {
          g_hitLimitTorque = true;
          demandCurrent = -g_currentLimit;
        }

        // Enforce the the velocity limit.
        if(g_currentPhaseVelocity > g_velocityLimit) {
          g_hitLimitVelocity = true;
          float breakingForce = (1.0 - (g_currentPhaseVelocity / g_velocityLimit)) * g_endStopTargetBreakCurrent;
          if(breakingForce > g_endStopTargetBreakCurrent)
            breakingForce = g_endStopTargetBreakCurrent;
          // If the user is requesting more current to slow down don't argue
          if(demandCurrent > breakingForce) {
            breakingForce =  demandCurrent;
          }
          // Set current directly to bypass any limits set by user
          FOC_current(g_phaseAngle,0,breakingForce);
          break;
        }
        if(g_currentPhaseVelocity < -g_velocityLimit) {
          g_hitLimitVelocity = true;
          float breakingForce = (1.0 + (g_currentPhaseVelocity / g_velocityLimit)) * g_endStopTargetBreakCurrent;
          if(breakingForce < -g_endStopTargetBreakCurrent)
            breakingForce = -g_endStopTargetBreakCurrent;
          // If the user is requesting more current to slow down don't argue
          if(demandCurrent < breakingForce) {
            breakingForce =  demandCurrent;
          }
          // Set current directly to bypass any limits set by user
          FOC_current(g_phaseAngle,0,breakingForce);
          break;
        }

        FOC_current(g_phaseAngle,0,demandCurrent);
      } break;
    }

    // Check home index switch.
    {
      bool es3 = palReadPad(POSITION_INDEX_GPIO_Port, POSITION_INDEX_Pin);
      if(es3 != g_lastLimitState) {
        g_lastLimitState = es3;
        MotionUpdateIndex(es3,g_currentPhasePosition,g_currentPhaseVelocity);
      }
    }


    // Flag motion control update if needed.
    if(++loopCount >= g_motorReportSampleCount) {
      loopCount = 0;
      chBSemSignal(&g_reportSampleReady);
    }

    // Do some sanity checks
    FaultCodeT errCode;
    if((errCode = CheckHallInRange()) != FC_Ok) {
      FaultDetected(errCode);
    }

    // Last send report if needed.
    static int g_pwmReportDownSample = 0;
    if(g_pwmFullReport && g_pwmReportDownSample++ > 2) {
      g_pwmReportDownSample = 0;
      struct PacketT *pkt;
      if((pkt = USBGetEmptyPacket(TIME_IMMEDIATE)) != 0) {
        struct PacketPWMStateC *ps = (struct PacketPWMStateC *)&(pkt->m_data);
        pkt->m_len = sizeof(struct PacketPWMStateC);
        ps->m_packetType = CPT_PWMState;
        ps->m_deviceId = g_deviceId;
        //ps->m_tick = g_adcTickCount;
        //for(int i = 0;i < 3;i++)
        //  ps->m_curr[i] = g_currentADCValue[i];
        for(int i = 0;i < 3;i++)
          ps->m_hall[i] = g_hall[i];
        ps->m_angle = g_phaseAngle * DOGBOT_PACKETSERVO_FLOATSCALE / (2.0 * M_PI);
        USBPostPacket(pkt);
      }
    }

  }
}

//! Check the state of the DRV8320 driver chip

bool CheckDriverStatus(void)
{
  // Check the state of the gate driver.
  uint16_t gateDriveStatus = Drv8320ReadRegister(DRV8320_REG_FAULT1);

  // Update gate status
  static uint16_t lastGateStatus = 0;
  if(lastGateStatus != gateDriveStatus || g_gateDriverWarning) {
    lastGateStatus = gateDriveStatus;
    if(g_gateDriverWarning)
      SendError(CET_MotorDriverWarning,0,0);
    SendParamData(CPI_DRV8305_00,&gateDriveStatus,sizeof(gateDriveStatus));
    g_gateDriverWarning = false;
    SendParamUpdate(CPI_DRV8305_01);
  }

  if(gateDriveStatus & DRV8320_FAULT1_FAULT) {
    FaultDetected(FC_DriverFault);
    return false;
  }

  return true;
}


static THD_FUNCTION(ThreadPWM, arg) {

  (void)arg;
  chRegSetThreadName("pwm");

  // Set initial motor mode
  SetMotorControlMode(CM_Brake);

  // Setup PWM.
  // Make sure output pins are in a sensible state before
  // starting anything
  InitPWM();

#if USE_PWM3
  // Depending if this works via SPI, then it won't do anything before
  // the driver chip is powered up
  EnableGateOutputs(false);
#endif

  EnableGateDriver(true);
  //! Wait for power up to complete
  int timeOut = 10;
  while (IsGateDriverFault() && g_pwmRun ) {
    if(timeOut-- < 0) {
      EnableGateDriver(false);
      g_gateDriverFault = true;
      g_pwmRun = false;
      g_pwmThreadRunning = false;
      return ;
    }
    chThdSleepMilliseconds(100);
  }

  //! Make sure the driver is setup.
  InitDrv8320();

  //! Read initial state of index switch
  g_lastLimitState = palReadPad(POSITION_INDEX_GPIO_Port, POSITION_INDEX_Pin); // Index

  //! Wait a bit more
  chThdSleepMilliseconds(100);

  EnableGateOutputs(true);

  //! Check the driver is happy
  if(!CheckDriverStatus()) {
    EnableGateDriver(false);
    g_gateDriverWarning = true;
    g_gateDriverFault = true;
    g_pwmRun = false;
    g_pwmThreadRunning = false;
    return ;
  }

  // Double check with the fault bit.
  if(IsGateDriverFault()) { // Fault pin
    g_gateDriverWarning = true;
    g_gateDriverFault = false; // Should only be a warning.
  } else {
    //! Reset fault flags.
    g_gateDriverFault = false;
    g_gateDriverWarning = false;
  }

  // Make sure state is reset to something sensible
  g_velocityISum = 0; // Reset velocity integral
  g_phaseRotationCount = 0; // Reset the rotation count to zero.
  g_demandTorque = 0;
  g_demandPhasePosition = 0;
  g_demandPhaseVelocity = 0;

#if ENABLE_ANGLESTATS
  {
    for(int i = 0;i < g_angleTableSize;i++){
      g_angleStats[i][0] = 0.0f;
      g_angleStats[i][1] = 0.0f;
    }
  }
#endif

  // Reset calibration state.
  MotionResetCalibration(MHS_Measuring);

  // This is quick, so may as well do it every time.
  ShuntCalibration();

  // Setup motor PID.
  SetupMotorCurrentPID();

  // Enable gate outputs
  EnableGateOutputs(true);

  // Do main control loop
  MotorControlLoop();

  // Make sure motor isn't being driven.
  PWMUpdateDrivePhase(TIM_1_8_PERIOD_CLOCKS/2,TIM_1_8_PERIOD_CLOCKS/2,TIM_1_8_PERIOD_CLOCKS/2);

  EnableGateOutputs(false);

  // Disable the PWM timer to save some power
  rccEnableTIM1(FALSE);

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
  EnableGateOutputs(false);

  InitHall2Angle();

  // Make sure current integrals are reset.
  g_current_control_integral_d = 0;
  g_current_control_integral_q = 0;

  rccEnableTIM1(FALSE);
  rccResetTIM1();

  stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;

  uint16_t psc = 0; // (SYSTEM_CORE_CLOCK / 80000000) - 1;
  tim->PSC  = psc;
  tim->ARR  = TIM_1_8_PERIOD_CLOCKS; // This should give about 20KHz
  tim->CR2  = 0;

#if USE_PWM3
  tim->CCER  =
      (STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC1NE ) |
      (STM32_TIM_CCER_CC2E | STM32_TIM_CCER_CC2NE | STM32_TIM_CCER_CC2NP) |
      (STM32_TIM_CCER_CC3E | STM32_TIM_CCER_CC3NE );
#else
  tim->CCER  =
      (STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC1NE ) |
      (STM32_TIM_CCER_CC2E | STM32_TIM_CCER_CC2NE ) |
      (STM32_TIM_CCER_CC3E | STM32_TIM_CCER_CC3NE );
#endif
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

#if 0
  tim->CCR[0] = pa;
  tim->CCR[1] = pb;
  tim->CCR[2] = pc;
#else
  tim->CCR[0] = pc;
  tim->CCR[1] = pb;
  tim->CCR[2] = pa;
#endif

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

  EnableGateDriver(false);

  return 0;
}

int PWMSVMScan(BaseSequentialStream *chp)
{

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

      queue_modulation_timings(v_alpha, v_beta,false);

      if (!palReadPad(BUTTON1_GPIO_Port, BUTTON1_Pin)) {
        break;
      }
    }
    if (!palReadPad(BUTTON1_GPIO_Port, BUTTON1_Pin)) {
      break;
    }
  }

  return 0;
}

void DisplayAngle(BaseSequentialStream *chp);


#define CHECK_OVERCURRENT 1

enum FaultCodeT PWMSelfTest()
{

  float rail5V = Read5VRailVoltage();
  if(rail5V < 4.5 || rail5V > 5.5)
    return FC_Internal5VRailOutOfRange;

  //g_vbus_voltage = ReadSupplyVoltage();
  if(g_vbus_voltage > g_maxSupplyVoltage)
    return FC_OverVoltage;
  if(g_vbus_voltage < g_minSupplyVoltage)
    return FC_UnderVoltage;

#if CHECK_OVERCURRENT && 0
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

  FaultCodeT errCode = FC_Ok;
  // Check hall sensors.
  for(int i = 0;i < 3;i++) {
    // If pins are floating, pull them into a fixed state.

    palSetPadMode(HALL_A_GPIO_Port,HALL_A_Pin,PAL_MODE_INPUT_PULLUP);
    palSetPadMode(HALL_B_GPIO_Port,HALL_B_Pin,PAL_MODE_INPUT_PULLUP);
    palSetPadMode(HALL_C_GPIO_Port,HALL_C_Pin,PAL_MODE_INPUT_PULLUP);

    chThdSleepMilliseconds(100);

    // Change them back to analog inputs.

    palSetPadMode(HALL_A_GPIO_Port,HALL_A_Pin,PAL_MODE_INPUT_ANALOG);
    palSetPadMode(HALL_B_GPIO_Port,HALL_B_Pin,PAL_MODE_INPUT_ANALOG);
    palSetPadMode(HALL_C_GPIO_Port,HALL_C_Pin,PAL_MODE_INPUT_ANALOG);

    // Wait for next measurement

    if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
      return FC_Internal; // Not sure what is going on.
    }
    if(chBSemWaitTimeout(&g_adcInjectedDataReady,5) != MSG_OK) {
      return FC_Internal; // Not sure what is going on.
    }
    errCode = CheckHallInRange();
    if(errCode == FC_Ok)
      break;
    // Retry 3 times, the failure may be caused by noise.
  }
  if(errCode != FC_Ok)
    return errCode;


#if CHECK_OVERCURRENT && 0
  EnableGateOutputs(true);

  // FIXME:- The over-current pin is now on the ethercat chip/
  // Check fan
  bool fanState = !palReadPad(GPIOA, GPIOA_PIN7); // Read fan power.
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

  //! Wait for power up to complete
  {
    int timeOut = 20;
    while (IsGateDriverFault() && timeOut-- > 0) {
      chThdSleepMilliseconds(100);
    }
    if(timeOut <= 0)
      return FC_DriverFault;
  }

  // Make sure PWM is running.
  InitPWM();

  EnableGateDriver(true);
  //! Wait for power up to complete
  int timeOut = 10;
  while (IsGateDriverFault()) {
    if(timeOut-- < 0) {
      EnableGateDriver(false);
      g_gateDriverFault = true;
      return FC_DriverFault;
    }
    chThdSleepMilliseconds(100);
  }

  // Calibrate shunts.
  ShuntCalibration();

  if((ret = PWMMotorCalResistance()) != FC_Ok) {
    EnableGateDriver(false);
    EnableGateOutputs(false);
    return ret;
  }
  SendParamUpdate(CPI_MotorResistance);

  if((ret = PWMMotorCalInductance()) != FC_Ok) {
    EnableGateDriver(false);
    EnableGateOutputs(false);
    return ret;
  }
  SendParamUpdate(CPI_MotorInductance);

  // Setup motor controller parameters
  SetupMotorCurrentPID();
  SendParamUpdate(CPI_MotorPGain);

  if((ret = PWMMotorPhaseCal()) != FC_Ok) {
    return ret;
  }

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

//static float sqrf(float v) { return v * v; }

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
    UpdateCurrentMeasurementsFromADCValues();

    float Ialpha = -(g_current[1] + g_current[2]);
    actualCurrent = actualCurrent * 0.99 + 0.01 * Ialpha;
    testVoltage += (kI * CURRENT_MEAS_PERIOD) * (targetCurrent - Ialpha);

    if (testVoltage > maxVoltage) testVoltage = maxVoltage;
    if (testVoltage < -maxVoltage) testVoltage = -maxVoltage;

    queue_voltage_timings(testVoltage, 0.0f,false);
  }

  // Turn off motor
  queue_voltage_timings(0.0, 0.0f,false);

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
      queue_voltage_timings( i != 0 ? (voltage_low - g_phaseOffsetVoltage): (voltage_high+g_phaseOffsetVoltage), 0.0f,false);
    }
  }

  // De-energize motor
  queue_voltage_timings(0.0f, 0.0f,false);

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

enum FaultCodeT PWMMotorPhaseCalReading(int phase,int cyclesPerSecond,int numberOfReadings,float torqueValue,float &lastAngle,int *readingCount)
{
  enum FaultCodeT ret = FC_Ok;

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
    readingCount[phaseStep]++;
  }

  return ret;
}

enum FaultCodeT PWMMotorPhaseCal()
{
  enum FaultCodeT ret = FC_Ok;

  // This assumes the motor is powered up, and the current shunts are calibrated.

  // Make sure PWM is running.
  InitPWM();

  int phaseRotations = 7;
  int numberOfReadings = 8;
  float torqueValue = 12.0;

  int cyclesPerSecond = 1.0 / (1.0 * CURRENT_MEAS_PERIOD);

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
  int readingCount[g_calibrationPointCount];
  for(int i = 0;i < g_calibrationPointCount;i++)
    readingCount[i] = 0;

  int phase = 0;
  for(;phase < g_calibrationPointCount*phaseRotations && ret == FC_Ok;phase++) {
    ret = PWMMotorPhaseCalReading(phase,cyclesPerSecond,numberOfReadings,torqueValue,lastAngle,readingCount);
  }
  phase--;
  for(;phase >= 0 && ret == FC_Ok;phase--) {
    ret = PWMMotorPhaseCalReading(phase,cyclesPerSecond,numberOfReadings,torqueValue,lastAngle,readingCount);
  }

  if(ret == FC_Ok) {
    for(int i = 0;i < g_calibrationPointCount;i++) {
      g_phaseAngles[i][0] /= readingCount[i];
      g_phaseAngles[i][1] /= readingCount[i];
      g_phaseAngles[i][2] /= readingCount[i];
    }
  }

  //DisplayAngle(chp);
  return ret;
}




int PWMCalSVM(BaseSequentialStream *chp)
{
  EnableGateOutputs(true);

  float voltage_magnitude = 0.06; // FIXME:- SHould change depending on supply voltage

  for(int phase = 0;phase <  g_calibrationPointCount*7;phase++) {
    int phaseStep = phase % g_calibrationPointCount;
    float phaseAngle = (float) phase * M_PI * 2.0 / ((float) g_calibrationPointCount);
    float v_alpha = voltage_magnitude * arm_cos_f32(phaseAngle);
    float v_beta  = voltage_magnitude * arm_sin_f32(phaseAngle);

    queue_modulation_timings(v_alpha, v_beta,false);

    // Wait for position to settle
    chThdSleepMilliseconds(1000);

    // Sync to avoid reading variables when they're being updated.
    chBSemWait(&g_adcInjectedDataReady);

    for(int i = 0;i < 3;i++) {
      g_phaseAngles[phaseStep][i] += g_hall[i];
    }
    if (!palReadPad(BUTTON1_GPIO_Port, BUTTON1_Pin)) {
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

  EnableGateOutputs(false);

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
  float phaseAngle = (angle * M_PI * 2.0 / calibRange);
  return wrapAngle(phaseAngle);
}


float hallToAngle(uint16_t *sensors)
{
  return hallToAngleDot2(sensors);
  //return hallToAngleRef(sensors);
}



static bool SensorlessEstimatorUpdate(float *eta_out)
{
  static const float pm_flux_linkage_ = 1.58e-3f;          // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
  static float flux_state_[2] = {0,0};
  static float observer_gain_ = 1000.0f;             // [rad/s]

  // Algorithm based on paper: Sensorless Control of Surface-Mount Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer
  // http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
  // In particular, equation 8 (and by extension eqn 4 and 6).

  // Clarke transform
  float I_alpha_beta[2] = {
      -g_current[1] - g_current[2],
      one_by_sqrt3 * (g_current[1] - g_current[2])
  };

  float V_alpha_beta[2] = {g_lastVoltageAlpha,g_lastVoltageBeta};

  // alpha-beta vector operations
  float eta[2];
  for (int i = 0; i <= 1; ++i) {
    // y is the total flux-driving voltage (see paper eqn 4)
    float y = -g_phaseResistance * I_alpha_beta[i] + V_alpha_beta[i];
    // flux dynamics (prediction)
    float x_dot = y;
    // integrate prediction to current timestep
    flux_state_[i] += x_dot * CURRENT_MEAS_PERIOD;

    // eta is the estimated permanent magnet flux (see paper eqn 6)
    eta[i] = flux_state_[i] - g_phaseInductance * I_alpha_beta[i];
  }

  // Non-linear observer (see paper eqn 8):
  const float pm_flux_sqr = pm_flux_linkage_ * pm_flux_linkage_;
  const float bandwidth_factor = 1.0f / pm_flux_sqr;
  float est_pm_flux_sqr = eta[0] * eta[0] + eta[1] * eta[1];
  float eta_factor = 0.5f * (observer_gain_ * bandwidth_factor) * (pm_flux_sqr - est_pm_flux_sqr);

  // alpha-beta vector operations
  for (int i = 0; i <= 1; ++i) {
    // add observer action to flux estimate dynamics
    float x_dot = eta_factor * eta[i];
    // convert action to discrete-time
    flux_state_[i] += x_dot * CURRENT_MEAS_PERIOD;
    // update new eta
    eta[i] = flux_state_[i] - g_phaseInductance * I_alpha_beta[i];
  }

  eta_out[0] = eta[0];
  eta_out[1] = eta[1];

  //*phase = fast_atan2(eta[1], eta[0]);
  return true;
};



