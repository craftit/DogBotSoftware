
#include "ch.h"
#include "hal.h"

#include "coms_serial.h"

#include "usbcfg.h"
#include "pwm.h"
#include "drv8503.h"
#include "eeprom.h"
#include "storedconf.h"
#include "protocol.h"
#include "canbus.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "storedconf.h"

#include "shell/shell.h"
#include "chprintf.h"
#include "coms.h"

#include "usbcfg.h"

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
#define TEST_WA_SIZE    THD_WORKING_AREA_SIZE(256)


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

  chprintf(chp, "PWMRun:%d ThreadRunning:%d Timeout:%d \r\n",g_pwmRun,g_pwmThreadRunning,g_pwmTimeoutCount);

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
  chprintf(chp, "Initialise PWM and calibrate... \r\n");
  InitPWM();
  PWMCalSVM(chp);
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

  chprintf(chp, "Shunt zero %d %d %d \r\n",
      (int) (g_currentZeroOffset[0] * 100.0),
      (int) (g_currentZeroOffset[1] * 100.0),
      (int) (g_currentZeroOffset[2] * 100.0)
  );

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
    chprintf(chp, "%2d : %4d %4d %4d / %4d %4d %4d \r",
        g_adcInjCount,
        g_currentADCValue[0],
        g_currentADCValue[1],
        g_currentADCValue[2],
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


static void cmd_eeInit(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argv;
  (void)argc;
  if(!g_eeInitDone) {
    chprintf(chp, "Init starting \r\n");
    StoredConf_Init();
  }
  chprintf(chp, "Init done \r\n");
}

static void cmd_eeLoad(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argv;
  (void)argc;
  uint16_t ret = 0;
  if(!g_eeInitDone) {
    chprintf(chp, "Init starting \r\n");
    StoredConf_Init();
    g_eeInitDone = true;
  }
  chprintf(chp, "Init complete \r\n");

  if(!StoredConf_Load(&g_storedConfig)) {
    chprintf(chp, "No stored configuration found \r\n",(int) ret);
  }
  for(int i = 0;i < 12;i++) {
    g_phaseAngles[i][0] = g_storedConfig.phaseAngles[i][0];
    g_phaseAngles[i][1] = g_storedConfig.phaseAngles[i][1];
    g_phaseAngles[i][2] = g_storedConfig.phaseAngles[i][2];
    chprintf(chp, "Phase: %d %d %d \r\n",g_phaseAngles[i][0],g_phaseAngles[i][1],g_phaseAngles[i][2]);
  }

  chprintf(chp, "Initalised system ok %d \r\n",(int) g_storedConfig.controllerId);
}

static void cmd_eeSave(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argv;
  (void)argc;
  uint16_t ret = 0;
  if(!g_eeInitDone) {
    StoredConf_Init();
    g_eeInitDone = true;
  }
  g_storedConfig.configState = 1;
  g_storedConfig.controllerId = 2;
  for(int i = 0;i < 12;i++) {
    g_storedConfig.phaseAngles[i][0] = g_phaseAngles[i][0];
    g_storedConfig.phaseAngles[i][1] = g_phaseAngles[i][1];
    g_storedConfig.phaseAngles[i][2] = g_phaseAngles[i][2];
  }
  if(!StoredConf_Save(&g_storedConfig)) {
    chprintf(chp, "No stored configuration found \r\n",(int) ret);
    return ;
  }

}

static void cmd_doDump(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;
  bool isAngle = false;
  if(argc == 1) {
    if(*argv[0] == 'a')
      isAngle = true;

    if(*argv[0] == 'b') {
      for(int i = 0;i < 12;i++) {
        chprintf(chp, "Phase: %d %d %d \r\n",g_phaseAngles[i][0],g_phaseAngles[i][1],g_phaseAngles[i][2]);
      }
      return ;
    }
  }


  chprintf(chp, "Dump \r\n");
  //for(int i = 0;i < 2;i++)
  while(true)
  {
    if(isAngle) {
      chprintf(chp, " Iq:%1.4f Id:%1.4f Errq:%4d Errd:%4d   %4d %4d %4d - Angle: %5d  Current: %d      \r",
        g_Iq,
        g_Id,
        (int) (g_Ierr_q * 1000.0),
        (int) (g_Ierr_d * 1000.0),
        (int) (g_current[0] * 1000.0),
        (int) (g_current[1] * 1000.0),
        (int) (g_current[2] * 1000.0),
        (int) (g_phaseAngle * 1000.0),
        (int) (g_current_Ibus * 1000.0)
        );
    } else {
      chprintf(chp, "Pos count %d  Position %d  Velocity %d      \r",
            g_phaseRotationCount,
            (int)(g_currentPhasePosition * 1000.0),
            (int)(g_currentPhaseVelocity));
    }
    chThdSleepMilliseconds(20);
    if (!palReadPad(GPIOB, GPIOA_PIN2)) {
      break;
    }
  }
}

static void cmd_doSet(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;
  if(argc != 2) {
    chprintf(chp, "set {torque,position,velocity,mode,pos_gain,vel_gain,vel_filt} value \r\n");
    return ;
  }
  chprintf(chp, "Args %s %s \r\n",argv[0],argv[1]);

  if(strcmp("torque",argv[0]) == 0 || strcmp("t",argv[0]) == 0) {
    int val = atoi(argv[1]);
    g_demandTorque = (float) val / 1000.0f;
    chprintf(chp, "Setting torque to %d \r\n",val);
  }
  if(strcmp("velocity",argv[0]) == 0 || strcmp("v",argv[0]) == 0) {
    int val = atoi(argv[1]);
    g_demandPhaseVelocity = (float) val / 1000.0f;
    chprintf(chp, "Setting velocity to %d \r\n",val);
  }
  if(strcmp("position",argv[0]) == 0 || strcmp("p",argv[0]) == 0) {
    int val = atoi(argv[1]);
    g_demandPhasePosition = (float) val / 10.0f;
    chprintf(chp, "Setting position to %d \r\n",val);
  }

  if(strcmp("pos_gain",argv[0]) == 0 || strcmp("pg",argv[0]) == 0) {
    int val = atoi(argv[1]);
    g_positionGain = (float) val / 1000.0f;
    chprintf(chp, "Setting position gain to %d \r\n",val);
  }

  if(strcmp("vel_gain",argv[0]) == 0 || strcmp("vg",argv[0]) == 0) {
    int val = atoi(argv[1]);
    g_velocityGain = (float) val / 1000.0f;
    chprintf(chp, "Setting velocity gain to %d \r\n",val);
  }

  if(strcmp("vel_filt",argv[0]) == 0 || strcmp("vf",argv[0]) == 0) {
    int val = atoi(argv[1]);
    g_velocityFilter = (float) val;
    chprintf(chp, "Setting velocity filter to %d \r\n",val);
  }


  if(strcmp("torque_limit",argv[0]) == 0 || strcmp("tl",argv[0]) == 0) {
    int val = atoi(argv[1]);
    g_torqueLimit = (float) val / 1000.0;
    chprintf(chp, "Setting torque limit to %d \r\n",val);
  }

  if(strcmp("pos_igain",argv[0]) == 0 || strcmp("pi",argv[0]) == 0) {
    int val = atoi(argv[1]);
    g_positionIGain = (float) val / 1000.0;
    chprintf(chp, "Setting position gain to %d \r\n",val);
  }

  if(strcmp("pos_iclamp",argv[0]) == 0 || strcmp("pic",argv[0]) == 0) {
    int val = atoi(argv[1]);
    g_positionIClamp = (float) val / 1000.0;
    chprintf(chp, "Setting position integral clamp to %d \r\n",val);
  }

  if(strcmp("motor_p_gain",argv[0]) == 0 || strcmp("mp",argv[0]) == 0) {
    int val = atoi(argv[1]);
    g_motor_p_gain = (float) val / 1000.0;
    chprintf(chp, "Setting motor p gain to %d \r\n",val);
  }

  if(strcmp("motor_i_gain",argv[0]) == 0 || strcmp("mi",argv[0]) == 0) {
    int val = atoi(argv[1]);
    g_motor_i_gain = (float) val / 1000.0;
    chprintf(chp, "Setting motor i gain to %d \r\n",val);
  }

  if(strcmp("mode",argv[0]) == 0 || strcmp("m",argv[0]) == 0) {
    if(strcmp("p",argv[1]) == 0) {
      g_controlMode = CM_Position;
    }
    if(strcmp("v",argv[1]) == 0) {
      g_controlMode = CM_Velocity;
    }
    if(strcmp("t",argv[1]) == 0) {
      g_controlMode = CM_Torque;
    }
    if(strcmp("i",argv[1]) == 0) {
      g_controlMode = CM_Idle;
    }

    chprintf(chp, "Setting mode to %d \r\n",(int) g_controlMode);
  }

}

static void cmd_doCan(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;

  InitCAN();
  chprintf(chp, "Can started. \r\n");

  CANTxFrame txmsg;

  //chRegSetThreadName("can transmitter");
  txmsg.IDE = CAN_IDE_EXT;
  txmsg.EID = 0x01234567;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 8;
  txmsg.data32[0] = 0x55AA55AA;
  txmsg.data32[1] = 0x00FF00FF;

  while (true) {
    chprintf(chp, "Can send. \r\n");
    msg_t ret = canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
    if(ret == MSG_OK) {
      chprintf(chp, "Transmit ok. \r\n");
    } else {
      chprintf(chp, "Transmit error. \r\n");
    }
    chThdSleepMilliseconds(200);

    if (!palReadPad(GPIOB, GPIOA_PIN2)) {
      break;
    }
  }
}

static void cmd_doPing(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;

  chprintf(chp, "Sending ping. \r\n");
#if 0
  if(SendPing(&SDU1)) {
    chprintf(chp, "Sent ok. \r\n");
  } else {
    chprintf(chp, "Send error. \r\n");
  }
#endif

}
static void cmd_doSync(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  (void)argc;

  chprintf(chp, "Sending sync. \r\n");
#if 0
  if(SendSync(&SDU1)) {
    chprintf(chp, "Sent ok. \r\n");
  } else {
    chprintf(chp, "Send error. \r\n");
  }
#endif
}


static const ShellCommand commands[] = {
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
  {"eeinit",cmd_eeInit},
  {"load",cmd_eeLoad},
  {"save",cmd_eeSave},
  {"dump",cmd_doDump},
  {"set",cmd_doSet },
  {"can",cmd_doCan },
  {"ping",cmd_doPing },
  {"sync",cmd_doSync },
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};
thread_t *shelltp1 = NULL;


event_listener_t g_shell_el;


void RunTerminal(void)
{
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */

  if (SDU1.config->usbp->state == USB_ACTIVE) {
    /* Starting shells.*/
    if (shelltp1 == NULL) {
      shelltp1 = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                     "shell1", NORMALPRIO + 1,
                                     shellThread, (void *)&shell_cfg1);
    }

    /* Waiting for an exit event then freeing terminated shells.*/
    chEvtWaitAny(EVENT_MASK(0));
    if (chThdTerminatedX(shelltp1)) {
      chThdRelease(shelltp1);
      shelltp1 = NULL;
    }
  }
  else {
    chThdSleepMilliseconds(1000);
  }

#if 0
  if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
    shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
  else if (chThdTerminatedX(shelltp)) {
    chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
    shelltp = NULL;           /* Triggers spawning of a new shell.        */
  }

    if (!shelltp2 && (SDU2.config->usbp->state == USB_ACTIVE))
      shelltp2 = shellCreate(&shell_cfg2, SHELL_WA_SIZE, NORMALPRIO);
    else if (chThdTerminatedX(shelltp2)) {
      chThdRelease(shelltp2);    /* Recovers memory of the previous shell.   */
      shelltp2 = NULL;           /* Triggers spawning of a new shell.        */
    }
#endif
}

void InitUSB(void)
{
  /*
   * Initializes two serial-over-USB CDC drivers.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  chEvtRegister(&shell_terminated, &g_shell_el, 0);

}

