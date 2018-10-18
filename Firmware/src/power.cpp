

#include <stdint.h>

#include "bmc.h"
#include "hal.h"
#include "coms.h"
#include "canbus.h"
#include "dogbot/protocol.h"
#include "chmboxes.h"
#include "pwm.h"
#include "mathfunc.h"
#include "motion.h"
#include "hal_channels.h"
#include "lan9252.h"
#include <string.h>

#include "drv8320.h"

static int16_t g_ioStatus = 0;
static int16_t g_ioOutput = 0;

static int16_t g_ioBitSensorPower = 0x0001;
static int16_t g_ioBitFanPower    = 0x0002;
static int16_t g_ioBitCANPower    = 0x0004;
static int16_t g_ioBitUSARTPower  = 0x0008;
static int16_t g_ioBitUSBPower    = 0x0010;
static int16_t g_ioBitCANStandby  = 0x20;

void PollIOStatus()
{
  SendParamUpdate(CPI_LAN9252);
  //g_ioStatus = Lan9252ReadRegister16(0x0f18);
}

void EnableFanPower(bool enable)
{
  if(enable) {
    palSetPad(FAN_POWER_GPIO_Port, FAN_POWER_Pin); // Turn off fan PWM power
    g_ioOutput = g_ioOutput | g_ioBitFanPower;
  }
  else
  {
    palClearPad(FAN_POWER_GPIO_Port, FAN_POWER_Pin); // Turn on fan PWM power
    g_ioOutput = g_ioOutput & (~g_ioBitFanPower);
  }
  // Update output
  //Lan9252SetRegister16(0x0F10,g_ioOutput);
}

bool HasSensorPower()
{
  return (g_ioStatus & g_ioBitSensorPower) != 0;
}

void EnableSensorPower(bool enable)
{
  if(enable) {
    g_ioOutput = g_ioOutput | g_ioBitSensorPower;
  } else {
    g_ioOutput = g_ioOutput & (~g_ioBitSensorPower);
  }
  // Update output
  //Lan9252SetRegister16(0x0F10,g_ioOutput);
}

