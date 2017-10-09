/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

#include "coms_serial.h"

#include "usbcfg.h"
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

#include "usbcfg.h"
#include "coms.h"
#include "shell/shell.h"

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
      case CS_Manual:
      case CS_Teach: {
        palSetPad(GPIOC, GPIOC_PIN4);       /* Green.  */
        chThdSleepMilliseconds(500);
        palClearPad(GPIOC, GPIOC_PIN4);     /* Green.  */
        chThdSleepMilliseconds(500);
      } break;
      case CS_SelfTest:
      case CS_FactoryCalibrate:
      case CS_PositionCalibration:
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

extern void Drv8503Init(void);
extern void RunTerminal(void);
extern void InitUSB(void);
extern void InitADC(void);

extern pid_t getpid(void);

void FaultDetected(enum FaultCodeT faultCode) {
  g_torqueLimit = 0;
  if(g_lastFaultCode == faultCode) {
    ChangeControlState(CS_Fault);
    return ;
  }
  g_lastFaultCode = faultCode;
  SendParamUpdate(CPI_FaultCode);
  ChangeControlState(CS_Fault);
}

float g_minSupplyVoltage = 12.0;

bool ChangeControlState(enum ControlStateT newState)
{
  // No change?
  if(newState == g_controlState)
    return true;

  // Check transition is ok.
  switch(g_controlState)
  {
    case CS_Fault:
      if(newState != CS_StartUp)
        return false;
      break;
    default:
      break;
  }

  g_controlState = newState;

  switch(newState)
  {
    case CS_PositionCalibration:
    case CS_Manual:
    case CS_Teach:
      PWMRun();
      break;
    case CS_StartUp:
    case CS_SelfTest:
    case CS_FactoryCalibrate:
    case CS_Standby:
    case CS_LowPower:
      PWMStop();
      SendParamUpdate(CPI_PositionCal);
      break;

    case CS_EmergencyStop:
    case CS_Fault: {
      g_torqueLimit = 0;
      PWMStop();
      SendParamUpdate(CPI_PositionCal);
    } break;

    default:
      return false;
  }

  SendParamUpdate(CPI_ControlState);

  return true;
}

/*
 * Application entry point.
 */
int main(void) {

  getpid();

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();


  InitADC();

  Drv8503Init();

  InitSerial();

  InitUSB();
  InitCAN();


  g_eeInitDone = true;
  StoredConf_Init();
  StoredConf_Load(&g_storedConfig);

  // Setup angles.
  for(int i = 0;i < 12;i++) {
    g_phaseAngles[i][0] = g_storedConfig.phaseAngles[i][0];
    g_phaseAngles[i][1] = g_storedConfig.phaseAngles[i][1];
    g_phaseAngles[i][2] = g_storedConfig.phaseAngles[i][2];
  }


#if 1
  InitComs();

  enum FaultCodeT faultCode = FC_Ok;

  /* Create the blinker threads. */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  chThdCreateStatic(waThreadOrangeLed, sizeof(waThreadOrangeLed), NORMALPRIO, ThreadOrangeLed, NULL);


  /* Is button pressed at startup ?  */
  bool doFactoryCal = !palReadPad(GPIOB, GPIOA_PIN2);

  while(1) {
    switch(g_controlState)
    {
      case CS_StartUp:
        PWMStop();
        g_lastFaultCode = FC_Ok; // Clear any errors
        SendParamUpdate(CPI_FaultCode);
        g_vbus_voltage = ReadSupplyVoltage();
        if(g_vbus_voltage < g_minSupplyVoltage) {
          ChangeControlState(CS_Standby);
          break;
        }

        faultCode = PWMSelfTest();
        if(faultCode != FC_Ok) {
          /* Self test failed. */
          FaultDetected(faultCode);
          break;
        }

        if(doFactoryCal) {
          doFactoryCal = false;
          ChangeControlState(CS_FactoryCalibrate);
          break;
        }
        ChangeControlState(CS_Manual);
        break;
      case CS_FactoryCalibrate:
        faultCode = PWMFactoryCal();
        if(faultCode != FC_Ok) {
          FaultDetected(faultCode);
        }
        ChangeControlState(CS_StartUp);
        break;
      case CS_Standby:
        chThdSleepMilliseconds(500);
        g_vbus_voltage = ReadSupplyVoltage();
        if(g_vbus_voltage > (g_minSupplyVoltage + 0.1)) {
          ChangeControlState(CS_StartUp);
          break;
        }
        break;
      case CS_SelfTest:
        break;
      case CS_LowPower:
      case CS_Fault:
      case CS_EmergencyStop:
        chThdSleepMilliseconds(1000);
        break;
      case CS_PositionCalibration:
      case CS_Teach:
      case CS_Manual:
        if(chBSemWaitTimeout(&g_reportSampleReady,1000) != MSG_OK) {
          //FaultDetected(FC_InternalTiming);
          break;
        }
#if 0
        if(!palReadPad(GPIOC, GPIOC_PIN15)) { // Fault pin
          FaultDetected(FC_DriverFault);
        }
#endif
        // This runs at 100Hz
        MotionStep();
        break;
    }

    // Stuff we want to check all the time.
    g_driveTemperature = ReadDriveTemperature();
    g_vbus_voltage = ReadSupplyVoltage();
    if(g_controlState != CS_Fault) {
      if(g_vbus_voltage > g_maxSupplyVoltage) {
        FaultDetected(FC_OverVoltage);
      }
      if(g_controlState != CS_LowPower &&
         g_controlState != CS_Standby &&
         g_vbus_voltage < g_minSupplyVoltage) {
        ChangeControlState(CS_LowPower);
      }
      if(g_driveTemperature > 75.0) {
        FaultDetected(FC_OverTemprature);
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



