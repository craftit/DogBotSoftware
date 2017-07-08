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
  }
}



#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "test.h"

#include "shell.h"
#include "chprintf.h"

#include "usbcfg.h"

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
#define TEST_WA_SIZE    THD_WORKING_AREA_SIZE(256)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state time\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%08lx %08lx %4lu %4lu %9s\r\n",
            (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
            (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
            states[tp->p_state]);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
  thread_t *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: test\r\n");
    return;
  }
  tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriorityX(),
                           TestThread, chp);
  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }
  chThdWait(tp);
}

/* Can be measured using dd if=/dev/xxxx of=/dev/null bs=512 count=10000.*/
static void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) {
  static uint8_t buf[] =
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: write\r\n");
    return;
  }

  while (chnGetTimeout((BaseChannel *)chp, TIME_IMMEDIATE) == Q_TIMEOUT) {
#if 1
    /* Writing in channel mode.*/
    chnWrite(&SDU1, buf, sizeof buf - 1);
#else
    /* Writing in buffer mode.*/
    (void) obqGetEmptyBufferTimeout(&SDU1.obqueue, TIME_INFINITE);
    memcpy(SDU1.obqueue.ptr, buf, SERIAL_USB_BUFFERS_SIZE);
    obqPostFullBuffer(&SDU1.obqueue, SERIAL_USB_BUFFERS_SIZE);
#endif
  }
  chprintf(chp, "\r\n\nstopped\r\n");
}

extern int DoADC(void);
extern void InitADC(void);

static void cmd_doDiag(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;

  chprintf(chp, "Starting conversion \r\n");
  int value = DoADC();
  chprintf(chp, "Result:%d  \r\n",value);

  chprintf(chp, "Power good: ");
  if (palReadPad(GPIOD, GPIOD_PIN2)) {
    chprintf(chp, "Ready \r\n");
  } else {
    chprintf(chp, "No \r\n");
  }

  chprintf(chp, "Status: ");
  if (palReadPad(GPIOC, GPIOC_PIN15)) {
    chprintf(chp, "Ok \r\n");
  } else {
    chprintf(chp, "Fault \r\n");
  }

  for(int i = 0;i < 15;i++){
    chprintf(chp, "Status %d: %x \r\n",i,Drv8503ReadRegister(i));
  }

}

static void cmd_doDrvTest(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;
  chprintf(chp, "Testing... \r\n");
  Drv8503Test();
}

static void cmd_doScan(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;
  chprintf(chp, "Scan SVM... \r\n");
  InitPWM();
  PWMSVMScan(chp);
}

static void cmd_doPWM(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;
  chprintf(chp, "Initialise PWM... \r\n");
  InitPWM();
  PWMCal(chp);
}

static void cmd_doPWMStop(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;
  chprintf(chp, "Stopping PWM... \r\n");
  PWMStop();
}

static void cmd_doPWMRun(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;
  chprintf(chp, "Run PWM... \r\n");
  PWMRun();
}

static void cmd_doPWMInit(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;
  chprintf(chp, "Init PWM... \r\n");
  InitPWM();
}

static void cmd_doADC(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;
  chprintf(chp, "Testing ADC... \r\n");
  //for(int i = 0;i < 2;i++)
  while(true)
  {
    uint16_t *samples = ReadADCs();
    chprintf(chp, "%d",samples[0]);
    for(int i =1;i < 14;i++) {
      chprintf(chp, ",\t%d",samples[i]);
    }
    chprintf(chp, "\r\n");
    chThdSleepMilliseconds(10);
    if (!palReadPad(GPIOB, GPIOA_PIN2)) {
      break;
    }
  }
}

extern uint16_t g_current[3];
extern uint16_t g_hall[3];
extern binary_semaphore_t g_adcInjectedDataReady;
extern int g_adcInjCount;

static void cmd_doPhase(BaseSequentialStream *chp, int argc, char *argv[]) {
  msg_t ret = MSG_OK;
  (void)argv;
  (void)argc;
  chprintf(chp, "Testing phase measurements... \r\n");
  //for(int i = 0;i < 2;i++)
  InitPWM();
  while(true)
  {
    ret = chBSemWaitTimeout(&g_adcInjectedDataReady,100);
    if(ret == MSG_TIMEOUT) {
      chprintf(chp, "Timeout");
      break;
    }
    if(ret == MSG_RESET) {
      chprintf(chp, "Reset");
      break;
    }
    chprintf(chp, "%d : %d %d %d / %d %d %d \r\n",
        g_adcInjCount,
        g_current[0],
        g_current[1],
        g_current[2],
        g_hall[0],
        g_hall[1],
        g_hall[2]
        );
    //chThdSleepMilliseconds(100);
    if (!palReadPad(GPIOB, GPIOA_PIN2)) {
      break;
    }
  }
}


static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"test", cmd_test},
  {"write", cmd_write},
  {"diag", cmd_doDiag},
  {"drvtest", cmd_doDrvTest},
  {"pwm", cmd_doPWM},
  {"scan", cmd_doScan },
  {"init", cmd_doPWMInit},
  {"stop", cmd_doPWMStop},
  {"run", cmd_doPWMRun},
  {"adc", cmd_doADC},
  {"phase",cmd_doPhase},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

extern void Drv8503Init(void);

/*
 * Application entry point.
 */
int main(void) {
  thread_t *shelltp = NULL;

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

  InitSerial();

  InitUSB();

  Drv8503Init();

  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
#if 0
  bool wasTrue = false;
  while (true) {
    if (palReadPad(GPIOB, GPIOA_PIN2)) {
      palSetPad(GPIOC, GPIOC_PIN5);       /* Orange.  */
      if(!wasTrue) {
        wasTrue = true;
        SendSerialTest();
      }
    } else {
      wasTrue = false;
      palClearPad(GPIOC,GPIOC_PIN5);       /* Orange.  */
    }
    chThdSleepMilliseconds(100);
  }
#endif

#if 1
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
      shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
    else if (chThdTerminatedX(shelltp)) {
      chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
      shelltp = NULL;           /* Triggers spawning of a new shell.        */
    }
    chThdSleepMilliseconds(1000);
  }
#endif
}

