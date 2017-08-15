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
#include <stdint.h>
#include "coms.h"

#define STM32_UID ((uint32_t *)0x1FFF7A10)

uint32_t m_nodeUId[2]; // Not absolutely guaranteed to be unique but collisions are unlikely;

/*
 * Internal loopback mode, 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static const CANConfig cancfg = {
  CAN_MCR_ABOM |
  CAN_MCR_AWUM |
  CAN_MCR_TXFP,
//  CAN_BTR_LBKM |
  CAN_BTR_SJW(0) |
  CAN_BTR_TS2(1) |
  CAN_BTR_TS1(8) |
  CAN_BTR_BRP(6)
};

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx_wa, 256);
static THD_FUNCTION(can_rx, p) {
  event_listener_t el;
  CANRxFrame rxmsg;

  (void)p;
  chRegSetThreadName("can receiver");
  chEvtRegister(&CAND1.rxfull_event, &el, 0);
  while(!chThdShouldTerminateX()) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
      continue;
    while (canReceive(&CAND1, CAN_ANY_MAILBOX,
                      &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
      /* Process message.*/
      palTogglePad(GPIOC, GPIOC_PIN5);       /* Yellow led. */
    }
  }
  chEvtUnregister(&CAND1.rxfull_event, &el);
}

/*
 * Transmitter thread.
 */
static THD_WORKING_AREA(can_tx_wa, 256);
static THD_FUNCTION(can_tx, p) {
  CANTxFrame txmsg;

  (void)p;
  chRegSetThreadName("can transmitter");
  txmsg.IDE = CAN_IDE_EXT;
  txmsg.EID = 0x01234567;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 8;
  txmsg.data32[0] = 0x55AA55AA;
  txmsg.data32[1] = 0x00FF00FF;

  while (!chThdShouldTerminateX()) {
    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
    chThdSleepMilliseconds(500);
  }
}

/*
 * Application entry point.
 */
int InitCAN(void)
{
  // Setup an unique node id.
  m_nodeUId[0] = STM32_UID[0];
  m_nodeUId[1] = STM32_UID[1] + STM32_UID[2];

  palClearPad(GPIOB, GPIOB_PIN5);       /* Make sure transmitter is in normal mode.  */

  static bool canInitDone = false;
  if(!canInitDone) {
    canInitDone = true;
    /*
     * Activates the CAN driver 1.
     */
    canStart(&CAND1, &cancfg);

    /*
     * Starting the transmitter and receiver threads.
     */
    chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 7,
                      can_rx, NULL);
  #if 0
    chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7,
                      can_tx, NULL);
  #endif
  }
  return 0;
}
