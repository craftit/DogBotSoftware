
#include "ch.h"
#include "hal.h"
#include "dogbot/protocol.h"
#include "bootloader.h"
#include "board.h"
#include "exec.h"
#include "canbus.h"

#include "coms.h"
#include "bmc.h"

enum FaultCodeT g_lastFaultCode = FC_Ok;
uint32_t g_faultState = 0;
uint8_t g_indicatorState = 0;
enum ControlStateT g_controlState = CS_BootLoader;

void FaultDetected(enum FaultCodeT faultCode)
{
  if(faultCode == FC_Ok)
    return ;
  int faultBit = 1<<((int) faultCode);
  // Have we already flagged this error?
  if((g_faultState & faultBit) != 0) {
    return ;
  }
  g_faultState |= faultBit;
  g_lastFaultCode = faultCode;
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

  bool forceBootload = !palReadPad(BUTTON1_GPIO_Port, BUTTON1_Pin);
  if(forceBootload)
    g_controlState = CS_BootLoader;
  else {
    if (RCC->CSR & RCC_CSR_SFTRSTF) { // Software reset ?
      g_controlState = CS_BootLoader;
    } else {
      g_controlState = CS_Standby;
    }
  }
  /*!< Remove reset flags */
  RCC->CSR |= RCC_CSR_RMVF;

  bool sentBootloaderMode = false;
  int deviceZeroTimeout = 0;

  if(g_controlState == CS_BootLoader) {
    chSysInit();

    InitCAN();

  #if 1
    InitUSB();
    InitComs();
  #endif


    palSetPad(LED_GREEN_GPIO_Port, LED_GREEN_Pin);       /* Green.  */
    palClearPad(LED_RED_GPIO_Port, LED_RED_Pin);       /* Red led. */

    while(g_controlState == CS_BootLoader)
    {
      palTogglePad(LED_GREEN_GPIO_Port, LED_GREEN_Pin);       /* Red led. */
      palTogglePad(LED_RED_GPIO_Port, LED_RED_Pin);       /* Green led. */

      chThdSleepMilliseconds(100);

      if(g_deviceId != 0 && !sentBootloaderMode) {
        /* Make sure the world knows we're in boot loader mode. */
        SendParamUpdate(CPI_ControlState);
        sentBootloaderMode = true;
      }

      // Keep generating announcements every second until we've been given an id.
      if(g_deviceId == 0) {
        if(deviceZeroTimeout++ > 10) {
          deviceZeroTimeout = 0;
          SendAnnounceId();
        }
      }

      // Just send this regularly so the controller knows we're alive
      SendParamUpdate(CPI_ControlState);
    }

    usbDisconnectBus(&USBD2);
    usbStop(&USBD2);

    //chSysLockFromIsr();
    chSysDisable();
  }

  flashJumpApplication(0x08010000);
}



