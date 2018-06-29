

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
unsigned g_emergencyStopTimer = 0;

enum FanModeT g_fanMode = FM_Auto;
enum SafetyModeT g_safetyMode = SM_GlobalEmergencyStop;
enum JointRoleT g_jointRole = JR_Spare;
float g_fanTemperatureThreshold = 40.0;

/*
 * This is a periodic thread that does nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(waThreadGreenLED, 128);
static THD_FUNCTION(ThreadGreenLED, arg) {

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
      case CS_Standby:
      case CS_SafeStop: {
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
      case CS_Ready: {
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

static THD_WORKING_AREA(waThreadOrangeLED, 128);
static THD_FUNCTION(ThreadOrangeLED, arg) {

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


uint32_t g_faultState = 0;

void FaultDetected(enum FaultCodeT faultCode)
{
  if(faultCode == FC_Ok)
    return ;
  g_currentLimit = 0;
  int faultBit = 1<<((int) faultCode);
  // Have we already flagged this error?
  if((g_faultState & faultBit) != 0) {
    ChangeControlState(CS_Fault,SCS_Fault);
    return ;
  }
  g_faultState |= faultBit;
  g_lastFaultCode = faultCode;
  SendParamUpdate(CPI_FaultCode);
  ChangeControlState(CS_Fault,SCS_Fault);
}

float g_minSupplyVoltage = 6.0;
enum StateChangeSourceT g_stateChangeCause = SCS_Internal;

int ChangeControlState(enum ControlStateT newState,enum StateChangeSourceT changeSource)
{
  // No change?
  if(newState == g_controlState)
    return true;

  // Check transition is ok.
  switch(g_controlState)
  {
    case CS_Fault:
      // no break
    case CS_EmergencyStop:
      if(newState != CS_Fault &&
         newState != CS_Standby
          )
      {
        SendParamUpdate(CPI_ControlState);
        return false;
      }
      break;
    case CS_SafeStop:
      if(newState != CS_Standby &&
          newState != CS_EmergencyStop &&
          newState != CS_Fault )
      {
        SendParamUpdate(CPI_ControlState);
        return false;
      }
      break;
    case CS_Standby:
      if(newState != CS_BootLoader &&
          newState != CS_FactoryCalibrate &&
          newState != CS_EmergencyStop &&
          newState != CS_SelfTest &&
          newState != CS_StartUp  &&
          newState != CS_Fault
          )
      {
        SendParamUpdate(CPI_ControlState);
        return false;
      }
      break;
    case CS_FactoryCalibrate:
      if( newState != CS_Standby &&
          newState != CS_EmergencyStop &&
          newState != CS_SafeStop &&
          newState != CS_Fault )
      {
        SendParamUpdate(CPI_ControlState);
        return false;
      }
      break;
    case CS_Ready:
    case CS_Diagnostic:
      // Don't let it turn to boot-loader from ready
      if(newState == CS_BootLoader) {
        SendParamUpdate(CPI_ControlState);
        return false;
      }
      break;
    case CS_StartUp:
    case CS_SelfTest:
    case CS_Home:
    case CS_BootLoader:
    case CS_MotionCalibrate:
    default:
      break;
  }


  switch(newState)
  {
    case CS_SelfTest:
    case CS_FactoryCalibrate:
      if(g_controlState != CS_Standby)
        break;
      g_stateChangeCause = changeSource;
      g_controlState = newState;
      EnableSensorPower(true);
      PWMStop();
      EnableFanPower(true); // Turn on the fan, as we won't be monitoring temperature while doing the calibration.
      SendParamUpdate(CPI_HomedState);
      break;
    case CS_StartUp:
      if(g_controlState != CS_StartUp) {
        ResetEmergencyStop();
        g_emergencyStopTimer = 0;
      }
      g_stateChangeCause = changeSource;
      g_controlState = newState;
      PWMStop();
      SendParamUpdate(CPI_HomedState);
      break;
    case CS_Standby:
      g_controlState = newState;
      g_stateChangeCause = changeSource;
      PWMStop();
      EnableSensorPower(false);
      SendParamUpdate(CPI_HomedState);
      break;
    case CS_SafeStop:
      g_controlState = newState;
      g_stateChangeCause = changeSource;
      g_currentLimit = 0;
      SetMotorControlMode(CM_Brake);
      break;
    case CS_MotionCalibrate:
    case CS_Diagnostic:
    case CS_Home:
      if(g_controlState != CS_Ready)
        break;
      break;
    case CS_Ready:
      // This can only happen from startup
      if(changeSource == SCS_UserRequest) {
        break;
      }
#if 0
      if(!CheckMotorSetup()) {
        g_controlState = CS_Standby;
        break;
      }
#endif
      g_stateChangeCause = changeSource;
      g_controlState = newState;

      EnableSensorPower(true);
      if(PWMRun())
        break; // If we started the PWM ok, just exit.
      /* no break */
    default: // If we don't recognise the state, just go into fault mode.
    case CS_Fault:
      if(g_safetyMode == SM_LocalStop ||
          g_controlState == CS_Standby ||
          g_controlState == CS_StartUp
          )
      {
        g_stateChangeCause = changeSource;
        g_controlState = CS_Fault;
        g_currentLimit = 0;
        PWMStop();
        EnableSensorPower(false);
        SendParamUpdate(CPI_HomedState);
        break;
      }
      // In case of a fault on a moving robot, attempt breaking anyway.
      // no break
    case CS_EmergencyStop:
      g_stateChangeCause = changeSource;
      g_controlState = CS_EmergencyStop;
      // Just put the breaks on.
      g_currentLimit = 0;
      SetMotorControlMode(CM_Brake);
      break;
    case CS_BootLoader: {

      g_stateChangeCause = changeSource;
      g_controlState = newState;
      PWMStop();
      EnableSensorPower(false);

      usbDisconnectBus(&USBD1);
      usbStop(&USBD1);

      //chSysLockFromIsr();
      chSysDisable();
      NVIC_SystemReset();

      flashJumpApplication(0x08000000);
    } break;
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
      SendParamUpdate(CPI_MotorTemp);
      break;
    case 3:
      if(g_controlState != CS_Standby) {
        SendParamUpdate(CPI_MotorTemp);
      }
      break;
    case 4:
      SendParamUpdate(CPI_PhaseVelocity);
      break;
    case 5:
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
    case 6:
      // Finish here unless we're in diagnostic mode.
      if(g_controlState != CS_Diagnostic) {
        stateCount = -1;
        return ;
      }
      break;
    case 7:
      SendParamUpdate(CPI_DemandPhaseVelocity);
      break;
    case 8:
      SendParamUpdate(CPI_HallSensors);
      break;
  }
  stateCount++;
}


void DoStartup(void)
{
  EnableSensorPower(true);

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
    FaultDetected(FC_UnderVoltage);
    return ;
  }
  if(g_vbus_voltage > g_maxSupplyVoltage) {
    FaultDetected(FC_OverVoltage);
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

  ChangeControlState(CS_Ready,SCS_Internal);
  return ;
}


/*
 * Application entry point.
 */
int main(void) {

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
  chThdCreateStatic(waThreadGreenLED, sizeof(waThreadGreenLED), NORMALPRIO, ThreadGreenLED, NULL);
  chThdCreateStatic(waThreadOrangeLED, sizeof(waThreadOrangeLED), NORMALPRIO, ThreadOrangeLED, NULL);

  int cycleCount = 0;


  SendParamUpdate(CPI_ControlState);

  while(1) {
    switch(g_controlState)
    {
      case CS_StartUp:
        SendBackgroundStateReport();
        if(g_vbus_voltage < g_minSupplyVoltage) {
          ChangeControlState(CS_Standby,SCS_Internal);
          break;
        }
        switch(g_safetyMode)
        {
        case SM_GlobalEmergencyStop:
          if(!EmergencyStopHaveReceivedSafeFlag()) {
            SendBackgroundStateReport();
            chThdSleepMilliseconds(100);
            break;
          }
          // Check received state.
          DoStartup();
          break;
        case SM_MasterEmergencyStop:
          if(!IsEmergencyStopButtonSetToSafe()) {
            SendBackgroundStateReport();
            chThdSleepMilliseconds(100);
            break;
          }
          DoStartup();
          break;
        case SM_LocalStop:
          DoStartup();
          break;
        default:
        case SM_Unknown:
          ChangeControlState(CS_Standby,SCS_Internal);
          break;
        }
        break;
      case CS_FactoryCalibrate:
        if(g_vbus_voltage < g_minSupplyVoltage) {
          FaultDetected(FC_UnderVoltage);
          break;
        }
        SetMotorControlMode(CM_Brake);
        SendBackgroundStateReport();
        faultCode = PWMMotorCal();
        if(faultCode != FC_Ok) {
          FaultDetected(faultCode);
          break;
        }
        SendBackgroundStateReport();
        ChangeControlState(CS_Standby,SCS_Internal);
        break;
      case CS_SelfTest:
        break;
      case CS_BootLoader:
      case CS_Standby:
      case CS_Fault:
        chThdSleepMilliseconds(200);
        SendBackgroundStateReport();
        break;
      case CS_EmergencyStop: {
        // Just make sure the breaks are on.
        SetMotorControlMode(CM_Brake);
        g_currentLimit = 0;
        // Broadcast messages for a while.
        if(g_emergencyStopTimer < 10) {
          CANEmergencyStop(g_deviceId,g_stateChangeCause);
          g_emergencyStopTimer++;
        }
        chThdSleepMilliseconds(100);
        SendBackgroundStateReport();
      } break;
      case CS_SafeStop:

        // FIXME- Add a timer to change to LowPower
        // no break
      case CS_MotionCalibrate:
      case CS_Home:
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
#if 1
      // This catches unexpected states, but also suppresses compiler warnings about unused switch values.
      default:
        FaultDetected(FC_Internal);
        chThdSleepMilliseconds(10);
        break;
#endif
    }



    // Stuff we want to check all the time.
    g_driveTemperature +=  (ReadDriveTemperature() - g_driveTemperature) * 0.1;
    if(g_driveTemperature > 85.0) {
      FaultDetected(FC_DriverOverTemperature);
    }
#if 1
    // Check fan state.
    switch(g_fanMode) {
      case FM_Off:
        EnableFanPower(false);
        break;
      case FM_Auto: {
        // FIXME:- Add Fan PWM ?
        if(g_driveTemperature > g_fanTemperatureThreshold) {
          EnableFanPower(true);
        } else if(g_driveTemperature < (g_fanTemperatureThreshold-5.0f)) {
          EnableFanPower(false);
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
    if(HasSensorPower()) {
      g_motorTemperature += (ReadMotorTemperature() - g_motorTemperature) * 0.1;

      if(g_motorTemperature > 52.0) {
        FaultDetected(FC_MotorOverTemperature);
      }
    }

    g_vbus_voltage += (ReadSupplyVoltage() - g_vbus_voltage) * 0.2;
    if(g_vbus_voltage > g_maxSupplyVoltage) {
      FaultDetected(FC_OverVoltage);
    }
    if(g_controlState != CS_Standby) {
      if(g_vbus_voltage < g_minSupplyVoltage) {
        // What to do when we detect low power supply voltage
        switch(g_controlState)
        {
          case CS_Ready:
          case CS_Diagnostic:
          case CS_Home:
          case CS_MotionCalibrate:
            ChangeControlState(CS_SafeStop,SCS_Internal);
            break;
          case CS_SelfTest:
          case CS_FactoryCalibrate:
          case CS_StartUp:
            ChangeControlState(CS_Standby,SCS_Internal);
            break;
          case CS_SafeStop:
          case CS_BootLoader:
          case CS_Fault:
          case CS_EmergencyStop:
          case CS_Standby:
            break;
        }
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



