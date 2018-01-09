
#include "ch.h"
#include "hal.h"
#include "dogbot/protocol.h"
#include "bootloader.h"

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


void flashJumpApplication(uint32_t address) {
  typedef void (*pFunction)(void);

  pFunction Jump_To_Application;

  /* variable that will be loaded with the start address of the application */
  vu32* JumpAddress;
  const vu32* ApplicationAddress = (vu32*) address;

  /* get jump address from application vector table */
  JumpAddress = (vu32*) ApplicationAddress[1];

  /* load this address into function pointer */
  Jump_To_Application = (pFunction) JumpAddress;

  /* reset all interrupts to default */
  chSysDisable();

  /* Clear pending interrupts just to be on the save side*/
  //SCB->ICSR = SCB_ICSR_PENDSVCLR;
//      ICSR_PENDSVCLR;;

  /* Disable all interrupts */
  int i;
  for(i=0; i<8; i++)
    NVIC->ICER[i] = NVIC->IABR[i];

  /* set stack pointer as in application's vector table */
  __set_MSP((u32) (ApplicationAddress[0]));
  Jump_To_Application();
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



