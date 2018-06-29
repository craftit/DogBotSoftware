
#include "pwm.h"

#include <hal.h>

static bool g_emergencyButtonConfigured = false;
static bool g_currentOutputState = false;
static int g_timeSinceLastComsOk = 5;
static int g_masterEmergencyStopDevice = 0;


void InitEmergencyStop(void)
{
  g_currentOutputState = false;
  palSetPadMode(GPIOC, GPIOC_PIN7,PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC, GPIOC_PIN6,PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(GPIOC, GPIOC_PIN6);

  g_emergencyButtonConfigured = true;
}

void ResetEmergencyStop(void)
{
  if(!g_emergencyButtonConfigured && g_safetyMode == SM_MasterEmergencyStop)
    InitEmergencyStop();
  g_masterEmergencyStopDevice = 0;
  g_timeSinceLastComsOk = 0;
}

bool IsEmergencyStopButtonSetup(void)
{
  return g_emergencyButtonConfigured;
}


bool IsEmergencyStopButtonSetToSafe(void)
{
  bool ret = false;
  if(g_currentOutputState) {
    if(palReadPad(GPIOC, GPIOC_PIN7) != 0)
      ret = true;
    g_currentOutputState = false;
    palClearPad(GPIOC, GPIOC_PIN6);
    palSetPadMode(GPIOC, GPIOC_PIN7,PAL_MODE_INPUT_PULLUP);
  } else {
    if(palReadPad(GPIOC, GPIOC_PIN7) == 0)
      ret = true;
    g_currentOutputState = true;
    palSetPad(GPIOC, GPIOC_PIN6);
    palSetPadMode(GPIOC, GPIOC_PIN7,PAL_MODE_INPUT_PULLDOWN);
  }
  return ret;
}

// This is called from the motor control loop at 100Hz

void EmergencyStopTick(void)
{
  if(g_safetyMode == SM_GlobalEmergencyStop) {
    if(g_timeSinceLastComsOk <= 0) {
      ChangeControlState(CS_EmergencyStop,SCS_EStopLostComs);
    } else {
      g_timeSinceLastComsOk--;
    }
  }
}

void EmergencyStopReceivedSafeFlag(int fromDeviceId)
{
  if(fromDeviceId == 0)
    return ;
  if(g_masterEmergencyStopDevice == 0) {
    g_masterEmergencyStopDevice = fromDeviceId;
  }
  if(fromDeviceId == g_masterEmergencyStopDevice)
    g_timeSinceLastComsOk = 4;
}

bool EmergencyStopHaveReceivedSafeFlag()
{
  return (g_masterEmergencyStopDevice > 0);
}

