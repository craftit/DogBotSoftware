#ifndef PWM_HEADER
#define PWM_HEADER 1

#include "ch.h"

int InitPWM(void);

int PWMRun(void);
int PWMStop(void);
int PWMCal(BaseSequentialStream *chp);
int PWMCalSVM(BaseSequentialStream *chp);
int PWMSVMScan(BaseSequentialStream *chp);

void PWMUpdateDrive(int phase,int power);

extern uint16_t *ReadADCs(void);

float hallToAngle(uint16_t *sensors);

extern binary_semaphore_t g_adcInjectedDataReady;
extern int16_t g_currentADCValue[3];
extern float g_currentZeroOffset[3];
extern uint16_t g_hall[3];
extern int g_adcInjCount;

extern int g_phaseAngles[12][3];

extern float g_current[3];
extern float g_phaseAngle;
extern float g_current_Ibus;


extern float g_demandPhasePosition;
extern float g_demandPhaseVelocity;
extern float g_demandTorque;

enum ControlModeT {
  CM_Idle,
  CM_Break,
  CM_Torque,
  CM_Velocity,
  CM_Position
} ;

extern enum ControlModeT g_controlMode;

#endif
