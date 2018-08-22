
#include "ch.h"
#include "drv8305.h"
#include "hal.h"

/*
 * Low speed SPI configuration (328.125kHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig ls_spicfg = {
  NULL,
  GPIOC,
  9,
  SPI_CR1_BR_2 | SPI_CR1_BR_1  | SPI_CR1_CPHA,
  0
};


//| SPI_CR1_DFF

uint16_t Drv8305SetRegister(uint16_t addr,uint16_t value) {
  uint8_t data[2];
  uint8_t txbuff[2];
  txbuff[0] = (addr & 0x0f) << 3 | ((value >> 8) & 0xff);
  txbuff[1] = value & 0xff;

  spiAcquireBus(&SPID3);              /* Acquire ownership of the bus.    */
  spiStart(&SPID3, &ls_spicfg);       /* Setup transfer parameters.       */
  spiSelect(&SPID3);                  /* Slave Select assertion.          */
  spiExchange(&SPID3,2,
              &txbuff, &data);          /* Atomic transfer operations.      */
  spiUnselect(&SPID3);                /* Slave Select de-assertion.       */
  spiReleaseBus(&SPID3);              /* Ownership release.               */
  return data[0];
}

uint16_t Drv8305ReadRegister(uint16_t addr) {
  uint8_t cmd[2] ;
  cmd[0] = (1 << 7) | (addr << 3);
  cmd[1] = 0;
  uint8_t data[2];
  data[0] = 0xff;
  data[1] = 0xff;
  spiAcquireBus(&SPID3);              /* Acquire ownership of the bus.    */
  spiStart(&SPID3, &ls_spicfg);       /* Setup transfer parameters.       */
  spiSelect(&SPID3);                  /* Slave Select assertion.          */
  spiExchange(&SPID3, 2,
              &cmd, &data);          /* Atomic transfer operations.      */
  spiUnselect(&SPID3);                /* Slave Select de-assertion.       */
  spiReleaseBus(&SPID3);              /* Ownership release.               */
  return (uint16_t) data[0] << 8 | (uint16_t) data[1];
}

uint16_t Drv8305ReadStatus(void)
{
  uint16_t ret= Drv8305ReadRegister(0x5);
  return ret;
}


void InitDrv8305(void)
{
  // Make sure the output of the sense amplifiers is clamped to 3.3V clear any faults
  // before being enabled
  Drv8305SetRegister(DRV8305_REG_IC_CONTROL,
      DRV8305_IC_FLIP_OTSD |      // Enable over temperature shutdown
      DRV8305_IC_EN_SNS_CLAMP |   // Enable sense amplifier clamp to 3.3V
      DRV8305_IC_CLR_FLTS         // Clear faults
      );

  Drv8305SetRegister(DRV8305_REG_DRIVER_HS,
      DRV8305_PEAK_SOURCE_70mA |
      DRV8305_PEAK_SINK_50mA |
      DRV8305_SOURCE_TIME_1780ns
      );
  Drv8305SetRegister(DRV8305_REG_DRIVER_LS,
      DRV8305_PEAK_SOURCE_70mA |
      DRV8305_PEAK_SINK_70mA |
      DRV8305_SOURCE_TIME_1780ns
      );

  Drv8305SetRegister(DRV8305_REG_VDS_SENSE_CONTROL,
      DRV8305_VDS_SHUT_DOWN |
      DRV8305_VDS_THRESHOLD_0V222  // 0V155 is just under 100 amps at 25C,
      );


}



uint16_t Drv8305Test(void)
{
  // Turn everything off
  palClearPad(GPIOA, GPIOA_PIN8);  // HC
  palClearPad(GPIOA, GPIOA_PIN9);  // HB
  palClearPad(GPIOA, GPIOA_PIN10); // HA
  palClearPad(GPIOB, GPIOB_PIN13); // LC
  palClearPad(GPIOB, GPIOB_PIN14); // LB
  palClearPad(GPIOB, GPIOB_PIN15); // LA

  palSetPad(GPIOC, GPIOC_PIN13); // Wake
  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  palSetPad(GPIOA, GPIOA_PIN10); // HA
  palSetPad(GPIOB, GPIOB_PIN14); // LB


  return 0;
}

