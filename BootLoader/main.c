
#include "ch.h"
#include "hal.h"
#include "dogbot/protocol.h"
#include "bootloader.h"
#include "exec.h"
#include "canbus.h"

#include "coms.h"

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

  bool forceBootload = !palReadPad(GPIOB, GPIOB_PIN2);
  if(forceBootload)
    g_controlState = CS_BootLoader;
  else {
    if (RCC->CSR & RCC_CSR_SFTRSTF) { // Software reset ?
      g_controlState = CS_BootLoader;
    } else {
      g_controlState = CS_StartUp;
    }
  }
  /*!< Remove reset flags */
  RCC->CSR |= RCC_CSR_RMVF;

  bool sentBootloaderMode = false;

  if(g_controlState == CS_BootLoader) {
    chSysInit();

    InitCAN();

  #if 1
    InitUSB();
    InitComs();
  #endif

    palSetPad(GPIOC, GPIOC_PIN4);       /* Green.  */
    palClearPad(GPIOC, GPIOC_PIN5);       /* Yellow led. */

    while(g_controlState == CS_BootLoader)
    {

      palTogglePad(GPIOC, GPIOC_PIN5);       /* Yellow led. */
      palTogglePad(GPIOC, GPIOC_PIN4);       /* Green led. */
      chThdSleepMilliseconds(100);
#if 0
      if(g_deviceId != 0 && !sentBootloaderMode) {
        /* Let the world know we're in bootloader mode. */
        SendParamUpdate(CPI_ControlState);
        sentBootloaderMode = true;
      }
#endif
      // Just send this regularly so the controller knows we're alive
      SendParamUpdate(CPI_ControlState);
    }

    usbDisconnectBus(&USBD1);
    usbStop(&USBD1);

    //chSysLockFromIsr();
    chSysDisable();
  }

  flashJumpApplication(0x08010000);
}



