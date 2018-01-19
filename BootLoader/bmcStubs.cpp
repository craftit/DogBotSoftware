
#include "motion.h"
#include "bmc.h"


bool MotionSetPosition(uint8_t mode,int16_t position,uint16_t torqueLimit)
{
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

int ChangeControlState(enum ControlStateT newState)
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
