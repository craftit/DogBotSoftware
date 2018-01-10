
#include "exec.h"
#include "hal.h"

void flashJumpApplication(uint32_t address)
{
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
