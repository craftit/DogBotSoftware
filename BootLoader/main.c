
#include "ch.h"
#include "hal.h"
#include "dogbot/protocol.h"
#include "bootloader.h"
#include "exec.h"

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
  chSysInit();

  InitCAN();


  palSetPad(GPIOC, GPIOC_PIN4);       /* Green.  */
  palClearPad(GPIOC, GPIOC_PIN5);       /* Yellow led. */

  while(g_controlState == CS_BootLoader)
  {
    palTogglePad(GPIOC, GPIOC_PIN5);       /* Yellow led. */
    palTogglePad(GPIOC, GPIOC_PIN4);       /* Green led. */
    chThdSleepMilliseconds(100);

    SendParamUpdate(CPI_ControlState);
  }

  flashJumpApplication(0x08010000);
}



