
#include "motion.h"
#include "bmc.h"


bool MotionSetPosition(uint8_t mode,int16_t position,uint16_t torqueLimit)
{
  (void) position;
  (void) mode;
  (void) torqueLimit;
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

uint8_t g_otherJointId = 0xff;

bool MotionOtherJointUpdate(int16_t position,int16_t torque,uint8_t mode)
{
  (void) position;
  (void) torque;
  (void) mode;
  return false;
}


void EmergencyStopReceivedSafeFlag(int /*fromDeviceId*/)
{

}
