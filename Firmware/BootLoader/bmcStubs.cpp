
#include "motion.h"
#include "bmc.h"


bool MotionSetDemand(uint8_t mode,uint8_t timestamp,int16_t demand,int16_t torque)
{
  (void) demand;
  (void) mode;
  (void) torque;
  (void) timestamp;
  return false;
}

bool MotionCalZero(void)
{
  return false;
}

enum FaultCodeT LoadSetup()
{
  return FC_Internal;
}

enum FaultCodeT SaveSetup()
{
  return FC_Internal;
}

int ChangeControlState(enum ControlStateT newState,enum StateChangeSourceT /*change source*/)
{
  switch(newState)
  {
    case CS_StartUp:
    case CS_BootLoader:
      break;
    default:
    return 0;
  }
  g_controlState = newState;
  return 0;
}


bool MotionSyncTime(void)
{
  return false;
}

void EmergencyStopReceivedSafeFlag(int /*fromDeviceId*/)
{

}
