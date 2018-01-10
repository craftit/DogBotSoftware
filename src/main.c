

#include "ch.h"
#include "hal.h"

#include "coms_serial.h"

#include "serial_usbcfg.h"
#include "pwm.h"
#include "drv8503.h"
#include "eeprom.h"
#include "storedconf.h"
#include "canbus.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "storedconf.h"
#include "motion.h"

#include "chprintf.h"

#include "coms.h"
#include "exec.h"
#include "shell/shell.h"

unsigned g_mainLoopTimeoutCount = 0;

enum FanModeT g_fanMode = FM_Auto;
float g_fanTemperatureThreshold = 40.0;

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    switch(g_controlState)
    {
      case CS_Fault: {
        palSetPad(GPIOC, GPIOC_PIN4);       /* Green.  */
        chThdSleepMilliseconds(20);
        palClearPad(GPIOC, GPIOC_PIN4);     /* Green.  */
        chThdSleepMilliseconds(5000);
      } break;
      case CS_LowPower:
      case CS_Standby: {
        palSetPad(GPIOC, GPIOC_PIN4);       /* Green.  */
        chThdSleepMilliseconds(20);
        palClearPad(GPIOC, GPIOC_PIN4);     /* Green.  */
        chThdSleepMilliseconds(2000);
      } break;
      case CS_StartUp: {
        palSetPad(GPIOC, GPIOC_PIN4);       /* Green.  */
        chThdSleepMilliseconds(300);
        palClearPad(GPIOC, GPIOC_PIN4);     /* Green.  */
        chThdSleepMilliseconds(100);
      } break;
      case CS_EmergencyStop: {
        palSetPad(GPIOC, GPIOC_PIN4);       /* Green.  */
        chThdSleepMilliseconds(100);
        palClearPad(GPIOC, GPIOC_PIN4);     /* Green.  */
        chThdSleepMilliseconds(100);
      } break;
      case CS_Ready:
      case CS_Teach: {
        palSetPad(GPIOC, GPIOC_PIN4);       /* Green.  */
        chThdSleepMilliseconds(500);
        palClearPad(GPIOC, GPIOC_PIN4);     /* Green.  */
        chThdSleepMilliseconds(500);
      } break;
      case CS_SelfTest:
      case CS_FactoryCalibrate:
      case CS_Home:
      default: {
        palSetPad(GPIOC, GPIOC_PIN4);       /* Green.  */
        chThdSleepMilliseconds(500);
        palClearPad(GPIOC, GPIOC_PIN4);     /* Green.  */
        chThdSleepMilliseconds(20);
      } break;
    }
  }
}

static THD_WORKING_AREA(waThreadOrangeLed, 128);
static THD_FUNCTION(ThreadOrangeLed, arg) {

  (void)arg;
  chRegSetThreadName("orange blinker");
  while (true) {
    if(g_indicatorState) {
      palSetPad(GPIOC, GPIOC_PIN5);       /* Yellow led. */
      chThdSleepMilliseconds(1000);
      palClearPad(GPIOC, GPIOC_PIN5);       /* Yellow led. */
      if(g_lastFaultCode == FC_Ok) continue;
    }
    if(g_lastFaultCode == FC_Ok) {
      chThdSleepMilliseconds(300);
      continue;
    }
    for(int i = 0;i < (int) g_lastFaultCode;i++) {
      chThdSleepMilliseconds(300);
      palSetPad(GPIOC, GPIOC_PIN5);       /* Yellow led. */
      chThdSleepMilliseconds(300);
      palClearPad(GPIOC, GPIOC_PIN5);       /* Yellow led. */
    }
    chThdSleepMilliseconds(800);
  }
}


/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

extern void InitDrv8503(void);
extern void RunTerminal(void);
extern void InitUSB(void);
extern void InitADC(void);

extern pid_t getpid(void);


bool HasSensorPower()
{
  return palReadPad(GPIOB, GPIOB_PIN12);
}

void EnableSensorPower(bool enable)
{
  if(enable)
    palSetPad(GPIOB, GPIOB_PIN12); // Turn off sensor power
  else
    palClearPad(GPIOB, GPIOB_PIN12); // Turn off sensor power
}

void EnableFanPower(bool enable)
{
  if(enable)
    palSetPad(GPIOA, GPIOA_PIN7); // Turn on aux power
  else
    palClearPad(GPIOA, GPIOA_PIN7); // Turn off aux power
}

uint32_t g_faultState = 0;

void FaultDetected(enum FaultCodeT faultCode)
{
  if(faultCode == FC_Ok)
    return ;
  g_currentLimit = 0;
  int faultBit = 1<<((int) faultCode);
  // Have we already flagged this error?
  if((g_faultState & faultBit) != 0) {
    ChangeControlState(CS_Fault);
    return ;
  }
  g_faultState |= faultBit;
  g_lastFaultCode = faultCode;
  SendParamUpdate(CPI_FaultCode);
  ChangeControlState(CS_Fault);
}

float g_minSupplyVoltage = 6.0;

int ChangeControlState(enum ControlStateT newState)
{
  // No change?
  if(newState == g_controlState)
    return true;


  // Check transition is ok.
  switch(g_controlState)
  {
    case CS_Fault:
    case CS_EmergencyStop:
      if(newState != CS_StartUp) {
        SendParamUpdate(CPI_ControlState);
        return false;
      }
      break;
    default:
      break;
  }

  // Can only jump to boot loader from standby, low power or fault.
  // Check state we're going to.
  if(newState == CS_BootLoader &&
      (g_controlState != CS_Standby &&
          g_controlState != CS_LowPower &&
          g_controlState != CS_Fault
          )) {
    SendParamUpdate(CPI_ControlState);
    return false;
  }

  g_controlState = newState;

  switch(newState)
  {
    case CS_SelfTest:
    case CS_FactoryCalibrate:
      EnableSensorPower(true);
      PWMStop();
      SendParamUpdate(CPI_HomedState);
      break;
    case CS_StartUp:
    case CS_Standby:
    case CS_LowPower:
      PWMStop();
      EnableSensorPower(false);
      SendParamUpdate(CPI_HomedState);
      break;

    case CS_EmergencyStop:
      // Just put the breaks on.
      g_currentLimit = 0;
      g_controlMode = CM_Brake;
      break;
    case CS_Home:
    case CS_Ready:
    case CS_Teach:
      EnableSensorPower(true);
      if(PWMRun())
        break;
      /* no break */
    case CS_Fault: {
      g_currentLimit = 0;
      PWMStop();
      EnableSensorPower(false);
      SendParamUpdate(CPI_HomedState);
    } break;
    case CS_BootLoader: {
      PWMStop();
      EnableSensorPower(false);
      flashJumpApplication(0x08000000);
    } break;
    default:
      return false;
  }

  SendParamUpdate(CPI_ControlState);

  return true;
}

void SendBackgroundStateReport(void)
{
  static int stateCount = 0;

  static int lastUsbError = 0;
  static int lastUsbDrop = 0;
  static int lastCANError = 0;
  static int lastCANDrop = 0;
  static unsigned lastFaultState = 0;
  static unsigned lastMainLoopTimeoutCount = 0;

  static bool lastIndexSensor = false;

  {
    bool isOn = palReadPad(GPIOC, GPIOC_PIN8);
    if(lastIndexSensor != isOn) {
      lastIndexSensor = isOn;
      SendParamUpdate(CPI_IndexSensor);
    }
  }
  // Distribute these over time.
  switch(stateCount)
  {
    default:
      stateCount = 0;
      /* no break */
    case 0:
      SendParamUpdate(CPI_VSUPPLY);
      break;
    case 1:
      SendParamUpdate(CPI_DriveTemp);
      break;
    case 2:
      if(g_controlState != CS_LowPower  && g_controlState != CS_Standby) {
        SendParamUpdate(CPI_MotorTemp);
      }
      break;
    case 3:
      SendParamUpdate(CPI_PhaseVelocity);
      break;
    case 4:
      if(lastUsbError != g_usbErrorCount) {
        lastUsbError = g_usbErrorCount;
        SendParamUpdate(CPI_USBPacketErrors);
      }
      if(lastUsbDrop != g_usbDropCount) {
        lastUsbDrop = g_usbDropCount;
        SendParamUpdate(CPI_USBPacketDrops);
      }
      if(lastCANError != g_canErrorCount) {
        lastCANError = g_canErrorCount;
        SendParamUpdate(CPI_CANPacketErrors);
      }
      if(lastCANDrop != g_canDropCount) {
        lastCANDrop = g_canDropCount;
        SendParamUpdate(CPI_CANPacketDrops);
      }
      if(lastMainLoopTimeoutCount != g_mainLoopTimeoutCount) {
        lastMainLoopTimeoutCount = g_mainLoopTimeoutCount;
        SendParamUpdate(CPI_MainLoopTimeout);
      }
      if(lastFaultState != g_faultState) {
        lastFaultState = g_faultState;
        SendParamUpdate(CPI_FaultState);
      }
      break;
    case 5:
      // Finish here unless we're in diagnostic mode.
      if(g_controlState != CS_Diagnostic) {
        stateCount = -1;
        return ;
      }
      break;
    case 6:
      SendParamUpdate(CPI_DemandPhaseVelocity);
      break;
    case 7:
      SendParamUpdate(CPI_HallSensors);
      break;
  }
  stateCount++;
}

bool g_doFactoryCal = false;

void DoStartup(void)
{
  PWMStop();
  MotionResetCalibration(MHS_Measuring);
  g_lastFaultCode = FC_Ok; // Clear any errors
  g_faultState = 0;
  SendParamUpdate(CPI_FaultCode);
  SendParamUpdate(CPI_FaultState);

  {
    float sum = 0;
    for(int i = 0;i < 10;i++)
      sum += ReadSupplyVoltage();
    g_vbus_voltage = sum / 10.0;
  }

  if(g_vbus_voltage < g_minSupplyVoltage) {
    ChangeControlState(CS_Standby);
    return ;
  }

  {
    float sum = 0;
    for(int i = 0;i < 10;i++) {
      sum += ReadDriveTemperature();
    }
    g_driveTemperature = sum/10.0;
  }
  {
    float sum = 0;
    for(int i = 0;i < 10;i++) {
      sum += ReadMotorTemperature();
    }
    g_motorTemperature = sum/10.0;
  }

  enum FaultCodeT faultCode = PWMSelfTest();
  if(faultCode != FC_Ok) {
    /* Self test failed. */
    FaultDetected(faultCode);
    return ;
  }


  if(g_doFactoryCal) {
    g_doFactoryCal = false;
    ChangeControlState(CS_FactoryCalibrate);
    return ;
  }

  ChangeControlState(CS_Ready);
  return ;
}


/*
 * Application entry point.
 */
int main(void) {

  getpid();

  /*
   * System initialisations.
   * - HAL initialisation, this also initialises the configured device drivers
   *   and performs the board-specific initialisations.
   * - Kernel initialisation, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  EnableSensorPower(false);
  EnableFanPower(false);

  InitADC();

  InitDrv8503();

  InitSerial();
  InitUSB();
  InitCAN();

  g_eeInitDone = true;
  StoredConf_Init();

  LoadSetup();

#if 1
  InitComs();

  enum FaultCodeT faultCode = FC_Ok;

  /* Create the blinker threads. */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  chThdCreateStatic(waThreadOrangeLed, sizeof(waThreadOrangeLed), NORMALPRIO, ThreadOrangeLed, NULL);

  int cycleCount = 0;

  /* Is button pressed ?  */
  g_doFactoryCal = !palReadPad(GPIOB, GPIOB_PIN2);

  while(1) {
    switch(g_controlState)
    {
      case CS_BootLoader:
      case CS_StartUp:
        DoStartup();
        break;
      case CS_FactoryCalibrate:
        SendBackgroundStateReport();
        faultCode = PWMMotorCal();
        if(faultCode != FC_Ok) {
          FaultDetected(faultCode);
        }
        SendBackgroundStateReport();
        ChangeControlState(CS_StartUp);
        break;
      case CS_Standby:
        chThdSleepMilliseconds(500);
        if(g_vbus_voltage > (g_minSupplyVoltage + 0.1)) {
          ChangeControlState(CS_StartUp);
          break;
        }
        SendBackgroundStateReport();
        break;
      case CS_SelfTest:
        break;
      case CS_LowPower:
      case CS_Fault:
        chThdSleepMilliseconds(500);
        SendBackgroundStateReport();
        break;
      case CS_EmergencyStop:
        // Just make sure the breaks are on.
        g_controlMode = CM_Brake;
        g_currentLimit = 0;
        chThdSleepMilliseconds(100);
        SendBackgroundStateReport();
        break;
      case CS_Home:
      case CS_Teach:
      case CS_Diagnostic:
      case CS_Ready: {
        if(chBSemWaitTimeout(&g_reportSampleReady,1000) != MSG_OK) {
          g_mainLoopTimeoutCount++;
          //FaultDetected(FC_InternalTiming);
          break;
        }

        if(g_gateDriverFault) {
          FaultDetected(FC_DriverFault);
          break;
        }

        // This runs at 100Hz
        MotionStep();

        // Check the state of the gate driver.
        uint16_t gateDriveStatus = Drv8503ReadRegister(DRV8503_REG_WARNING);
        {
          // Update gate status
          static uint16_t lastGateStatus = 0;
          if(lastGateStatus != gateDriveStatus) {
            lastGateStatus = gateDriveStatus;
            SendParamUpdate(CPI_DRV8305_01);
          }
        }
        if(gateDriveStatus & DRV8503_WARN_FAULT) {
          FaultDetected(FC_DriverFault);
          break;
        }

        // 20Hz
        if(cycleCount++ > 5) {
          cycleCount = 0;
          SendBackgroundStateReport();
        }
      } break;
    }



    // Stuff we want to check all the time.
    g_driveTemperature +=  (ReadDriveTemperature() - g_driveTemperature) * 0.1;

#if 1
    // Check fan state.
    switch(g_fanMode) {
      case FM_Off:
        palClearPad(GPIOA, GPIOA_PIN7);
        break;
      case FM_Auto: {
        // FIXME:- Add Fan PWM ?
        if(g_driveTemperature > g_fanTemperatureThreshold) {
          EnableFanPower(true);
        } else {
          if(g_driveTemperature < (g_fanTemperatureThreshold-5)) {
            EnableFanPower(false);
          }
        }
      }
      /* no break */
      case FM_On: {
        if(!palReadPad(GPIOB, GPIOB_PIN11)) { // Status feedback
          EnableFanPower(false);
          FaultDetected(FC_FanOverCurrent);
          g_fanMode = FM_Off; // Turn it off.
        }
      } break;
    }

    // Check sensor over current input.
    if(!palReadPad(GPIOB, GPIOB_PIN10)) {
      EnableSensorPower(false);
      FaultDetected(FC_SensorOverCurrent);
    }
#endif

    // We can only read motor temperatures when sensors are enabled.
    if(g_controlState != CS_LowPower  && g_controlState != CS_Standby) {
      g_motorTemperature += (ReadMotorTemperature() - g_motorTemperature) * 0.1;

      if(g_motorTemperature > 60.0) {
        FaultDetected(FC_MotorOverTemperature);
      }
    }

    g_vbus_voltage += (ReadSupplyVoltage() - g_vbus_voltage) * 0.2;
    if(g_vbus_voltage > g_maxSupplyVoltage) {
      FaultDetected(FC_OverVoltage);
    }
    if(g_driveTemperature > 80.0) {
      FaultDetected(FC_DriverOverTemperature);
    }
    if(g_controlState != CS_Fault) {
      if(g_controlState != CS_LowPower &&
         g_controlState != CS_Standby &&
         g_vbus_voltage < g_minSupplyVoltage) {
        ChangeControlState(CS_LowPower);
      }
    }
  }


#else
  /*
   * Shell manager initialisation.
   */
  shellInit();

  while(true) {
    RunTerminal();
    chThdSleepMilliseconds(100);
  }
#endif
}



