
#include "TLE5012B.h"
#include "ch.h"
#include "hal.h"
#include "mathfunc.h"


#define ENCSPI SPID2
/*
 * Low speed SPI configuration 5.125MHz
 * This is clocked from APB1 peripheral clock running at 41MHz
 *
 */
static const SPIConfig magencoder_spicfg = {
  NULL,
  ENC_SPI_NSELECT_GPIO_Port,
  ENC_SPI_NSELECT_Pin,
  SPI_CR1_CPHA | SPI_CR1_BIDIMODE | SPI_CR1_BR_1,
  0
};




static const int16_t g_TLE5012CommandReadBit   = 1 << 15;
static const int16_t g_TLE5012CommandLocked    = 0xA << 11;
static const int16_t g_TLE5012CommandUpdateReg =   1 << 10;

static int16_t TLE5012CommandAddress(int16_t addr)
{ return (addr & 0x3f) << 4; }

static void SPIEnableWrite(SPI_TypeDef *spi)
{
  spi->CR1 = spi->CR1 | SPI_CR1_BIDIOE ;
}

static void SPIEnableRead(SPI_TypeDef *spi)
{
  spi->CR1 = spi->CR1 & (~SPI_CR1_BIDIOE) ;
}

#if 0
void TLE5012WriteRegister(uint16_t address, uint16_t data)
{
  spiSelect(&ENCSPI);                  /* Slave Select assertion.          */

  // Transmit command, skip the status word
  SPIEnableWrite(&ENCSPI);

  uint16_t command = TLE5012CommandAddress(address) ;
  // Turn on transmit

  spiPolledExchange(&ENCSPI,command);
  spiPolledExchange(&ENCSPI,data);

  spiUnselect(&ENCSPI);                /* Slave Select de-assertion.       */
}

uint16_t TLE5012WriteRegisterStatus(uint16_t address, uint16_t data)
{
  spiSelect(&ENCSPI);                  /* Slave Select assertion.          */

  // Transmit command, skip the status word
  SPIEnableWrite(&ENCSPI);

  uint16_t command = TLE5012CommandAddress(address) ;

  spiPolledExchange(&ENCSPI,command);
  spiPolledExchange(&ENCSPI,data);

  SPIEnableRead(&ENCSPI);

  uint16_t ret = spiPolledExchange(&ENCSPI,data);

  spiUnselect(&ENCSPI);                /* Slave Select de-assertion.       */
  spiReleaseBus(&ENCSPI);              /* Ownership release.               */

  return ret;
}
#endif

uint16_t TLE5012ReadRegister(uint16_t address)
{
  uint16_t data = 0;
#if 0
  spiAcquireBus(&ENCSPI);              /* Acquire ownership of the bus.    */
  spiStart(&ENCSPI, &magencoder_spicfg);       /* Setup transfer parameters.       */
  spiSelect(&ENCSPI);                  /* Slave Select assertion.          */

  // Turn on transmit
  SPIEnableWrite(ENCSPI->spi);


  uint16_t rxbuff;
  spiExchange(&ENCSPI,2,&command, &rxbuff);
  //spiPolledExchange(&ENCSPI,command);

  chThdSleepMicroseconds(1);

  // Switch to receive
  SPIEnableRead(ENCSPI->spi);

  //spiPolledExchange(&ENCSPI,0);
  spiExchange(&ENCSPI,2,&command, &data);


  spiUnselect(&ENCSPI);                /* Slave Select de-assertion.       */
  spiReleaseBus(&ENCSPI);              /* Ownership release.               */
#else
  // This routine takes 4.2 us to run.

  palClearPad(ENC_SPI_NSELECT_GPIO_Port,ENC_SPI_NSELECT_Pin);

  SPI_TypeDef *SPIx = SPI2;

  SPIEnableWrite(SPIx);
  SPIx->CR1 |= SPI_CR1_SPE;

  data = SPIx->DR; // This clears the read bit

  uint16_t command = TLE5012CommandAddress(address) | g_TLE5012CommandReadBit;
  SPIx->DR = command;
  while((SPIx->SR & SPI_SR_BSY) != 0) ;

  //data = SPIx->DR; // This clears the read bit

  for(int i = 0 ;i < 20;i++)
    data += SPIx->SR;

  SPIEnableRead(SPIx);

  while((SPIx->SR & SPI_SR_RXNE) == 0) ;

  data = SPIx->DR; // This clears the read bit

  while((SPIx->SR & SPI_SR_RXNE) == 0) ;

  data = SPIx->DR; // This clears the read bit

  SPIEnableWrite(SPIx);

  SPIx->CR1 &= ~SPI_CR1_SPE;

  palSetPad(ENC_SPI_NSELECT_GPIO_Port,ENC_SPI_NSELECT_Pin);

#endif
  return data;
}

int16_t TLE5012ReadAngleInt()
{
  int16_t rawData = TLE5012ReadRegister(2);

  rawData = (rawData & (0x7fff));

  // check if the value received is positive or negative
  if (rawData & (1 << 14))
  {
    rawData = rawData - 32768;
  }
  return rawData;
}

float TLE5012ReadAngleFloat()
{
  int16_t rawData = TLE5012ReadAngleInt();
  return ((rawData * M_PI * 2) / 32768.0);
}



//| SPI_CR1_BIDIOE

enum FaultCodeT InitTLE5012B(void)
{
#if 0
  spiAcquireBus(&ENCSPI);              /* Acquire ownership of the bus.    */
  spiStart(&ENCSPI, &magencoder_spicfg);       /* Setup transfer parameters.       */

  uint16_t val = TLE5012ReadRegister(0);

  spiReleaseBus(&ENCSPI);              /* Ownership release.               */
#else
  palSetPad(ENC_SPI_NSELECT_GPIO_Port,ENC_SPI_NSELECT_Pin);
  rccEnableSPI2(TRUE);

  SPI_TypeDef *SPIx = SPI2;

  // Set things up
  SPIx->CR1 = SPI_CR1_CPHA | SPI_CR1_BIDIMODE | SPI_CR1_BR_1 | SPI_CR1_DFF | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
  SPIx->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
  SPIx->CR2 = 0;

  // Enable
  SPIx->CR1 |= SPI_CR1_SPE;
#endif

  return FC_Ok;
}


