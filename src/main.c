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
#include "test.h"

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
#include "test.h"
#include "storedconf.h"

#include "shell.h"
#include "chprintf.h"

#include "usbcfg.h"
#include "coms.h"

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palSetPad(GPIOC, GPIOC_PIN4);       /* Orange.  */
    chThdSleepMilliseconds(500);
    palClearPad(GPIOC, GPIOC_PIN4);     /* Orange.  */
    chThdSleepMilliseconds(500);
    //SendSync(&SDU1);
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

  //InitComs();


  g_eeInitDone = true;
  StoredConf_Init();
  StoredConf_Load(&g_storedConfig);

  /*
   * Shell manager initialisation.
   */
  shellInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  InitCAN();

  int i = 0;
  BaseSequentialStream *chp =(BaseSequentialStream *)&SDU1;
  while(true) {
    //RunTerminal();

    if(SDU1.config->usbp->state == USB_ACTIVE) {
      //SendSync(chp);
      //(,(const uint8_t *) "Hello\n\r", 7);
      chprintf(chp, "count %d \r\n",i++);

      //chnWrite((BaseSequentialStream *)&SDU1,(const uint8_t *) "Hello\n\r", 7);
    }

    chThdSleepMilliseconds(100);
  }

}

