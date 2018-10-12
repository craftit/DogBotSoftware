

#include "ch.h"
#include "hal.h"

#include "drv8320.h"

#define DRVSPI SPID1
/*
 * Low speed SPI configuration (328.125kHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig ls_spicfg = {
  NULL,
  DRIVE_SPI_NSELECT_GPIO_Port,
  DRIVE_SPI_NSELECT_Pin,
  SPI_CR1_BR_2 | SPI_CR1_BR_1  | SPI_CR1_CPHA,
  0
};


//| SPI_CR1_DFF

uint16_t Drv8320SetRegister(uint16_t addr,uint16_t value) {
  uint8_t data[2];
  uint8_t txbuff[2];
  txbuff[0] = (addr & 0x0f) << 3 | ((value >> 8) & 0x07);
  txbuff[1] = value & 0xff;

  spiAcquireBus(&DRVSPI);              /* Acquire ownership of the bus.    */
  spiStart(&DRVSPI, &ls_spicfg);       /* Setup transfer parameters.       */
  spiSelect(&DRVSPI);                  /* Slave Select assertion.          */
  spiExchange(&DRVSPI,2,
              &txbuff, &data);          /* Atomic transfer operations.      */
  spiUnselect(&DRVSPI);                /* Slave Select de-assertion.       */
  spiReleaseBus(&DRVSPI);              /* Ownership release.               */
  return ((uint16_t) data[0] << 8) | ((uint16_t) data[1]);
}

uint16_t Drv8320ReadRegister(uint16_t addr) {
  uint8_t cmd[2] ;
  cmd[0] = (1 << 7) | (addr << 3);
  cmd[1] = 0;
  uint8_t data[2];
  data[0] = 0xff;
  data[1] = 0xff;
  spiAcquireBus(&DRVSPI);              /* Acquire ownership of the bus.    */
  spiStart(&DRVSPI, &ls_spicfg);       /* Setup transfer parameters.       */
  spiSelect(&DRVSPI);                  /* Slave Select assertion.          */
  spiExchange(&DRVSPI, 2,
              &cmd, &data);          /* Atomic transfer operations.      */
  spiUnselect(&DRVSPI);                /* Slave Select de-assertion.       */
  spiReleaseBus(&DRVSPI);              /* Ownership release.               */
  return ((uint16_t) data[0] << 8) | ((uint16_t) data[1]);
}

uint16_t Drv8320ReadStatus(void)
{
  uint16_t ret= Drv8320ReadRegister(0x5);
  return ret;
}


void InitDrv8320(void)
{
  // Make sure the output of the sense amplifiers is clamped to 3.3V clear any faults
  // before being enabled

  Drv8320SetRegister(DRV8320_REG_DRIVE_CONTROL,
                     DRV8320_DC_PWM6x |      // Enable over temperature shutdown
                     DRV8320_DC_1PWM_COM |   // Enable sense amplifier clamp to 3.3V
                     DRV8320_DC_CLR_FLTS         // Clear faults
                    );

  Drv8320SetRegister(DRV8320_GATE_DRIVE_HS,
                     DRV8320_PEAK_SOURCE_1000mA |
                     DRV8320_PEAK_SINK_2000mA
                     );
  Drv8320SetRegister(DRV8320_GATE_DRIVE_LS,
                     DRV8320_PEAK_SOURCE_1000mA |
                     DRV8320_PEAK_SINK_2000mA |
                     DRV8320_DRIVE_TIME_4000ns
      );

  Drv8320SetRegister(DRV8320_OCP_CONTROL,
                     DRV8320_TRETRY_4ms |
                     DRV8320_DEAD_TIME_100ns |
                     DRV8320_OCP_MODE_RETRY |
                     DRV8320_OCP_DEG_4us |
                     DRV8320_VDS_LVL_0V26  // 0V26 is around 185 amps at 100C,
                     );


}



uint16_t Drv8320Test(void)
{


  return 0;
}

