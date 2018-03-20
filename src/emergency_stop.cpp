
#include "pwm.h"

#include <hal.h>

static bool g_emergencyButtonConfigured = false;
static bool g_currentOutputState = false;
static int g_timeSinceLastComsOk = 5;
static int g_masterEmergencyStopDevice = 0;


void InitEmergencyStop(void)
{
  palSetPadMode(GPIOC, GPIOC_PIN6,PAL_MODE_OUTPUT_PUSHPULL);
  g_currentOutputState = false;
  palClearPad(GPIOC, GPIOC_PIN6);
  palSetPadMode(GPIOC, GPIOC_PIN7,PAL_MODE_INPUT_PULLUP);

  g_emergencyButtonConfigured = true;
}

void ResetEmergencyStop(void)
{
  if(!g_emergencyButtonConfigured)
    InitEmergencyStop();
  g_masterEmergencyStopDevice = 0;
}

bool IsEmergencyStopButtonSetup(void)
{
  return g_emergencyButtonConfigured;
}


bool IsEmergencyStopButtonSetToSafe(void)
{
  if(!g_emergencyButtonConfigured)
    return false;
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

void EmergencyStopTick(void)
{
  if(g_safetyMode == SM_GlobalEmergencyStop) {
    g_timeSinceLastComsOk--;
    if(g_timeSinceLastComsOk <= 0) {
      ChangeControlState(CS_EmergencyStop,SCS_Internal);
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
  return (g_masterEmergencyStopDevice != 0);
}

