
#include "TLE5012B.h"
#include "ch.h"
#include "hal.h"
#include "mathfunc.h"


#define USE_DMA 1

#define ENCSPI SPID2

#define SPI2_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI2_RX_DMA_STREAM,                        \
                       STM32_SPI2_RX_DMA_CHN)

#define SPI2_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI2_TX_DMA_STREAM,                        \
                       STM32_SPI2_TX_DMA_CHN)


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



// Compute the CRC

uint8_t crc8(uint8_t *data ,uint8_t length)
{
  const uint32_t CRC_POLYNOMIAL=0x1D;
  const uint32_t CRC_SEED =0xFF;

  uint32_t crc = CRC_SEED;
  int16_t i,bit;

  for ( i=0 ; i<length ; i++ )
  {
    crc ^= data[i];
    for ( bit=0 ; bit<8 ; bit++)
    {
      if ( (crc & 0x80)!=0 )
      {
          crc <<= 1;
          crc ^= CRC_POLYNOMIAL;
      }
      else
      {
          crc <<= 1;
      }
    }
  }

  return (~crc) & CRC_SEED;
}


static uint16_t g_spi2Buffer[8];
thread_reference_t g_spi2thread = 0;

#if USE_DMA

static const stm32_dma_stream_t  *g_spi2dmarx = STM32_DMA_STREAM(STM32_SPI_SPI2_RX_DMA_STREAM);
static const uint32_t g_spi2rxdmamode = STM32_DMA_CR_CHSEL(SPI2_RX_DMA_CHANNEL) |
    STM32_DMA_CR_PL(STM32_SPI_SPI2_DMA_PRIORITY) |
    STM32_DMA_CR_DIR_P2M |
    STM32_DMA_CR_TCIE |
    STM32_DMA_CR_DMEIE |
    STM32_DMA_CR_TEIE |
    STM32_DMA_CR_PSIZE_HWORD |
    STM32_DMA_CR_MSIZE_HWORD;

static void StartDMARecieve(uint16_t *rxbuf,int n)
{
  dmaStreamSetMemory0(g_spi2dmarx, rxbuf);
  dmaStreamSetTransactionSize(g_spi2dmarx, n);
  dmaStreamSetMode(g_spi2dmarx, g_spi2rxdmamode | STM32_DMA_CR_MINC);
  dmaStreamEnable(g_spi2dmarx);
}

static void spi_rx_interrupt(void *chan, uint32_t flags) {

  /* DMA errors handling.*/
  (void)flags;
  (void) chan;

  SPI_TypeDef *SPIx = SPI2;
  SPIEnableWrite(SPIx);

  /* Stop everything.*/
  dmaStreamDisable(g_spi2dmarx);

  osalSysLockFromISR();
  osalThreadResumeI(&g_spi2thread, MSG_OK);
  osalSysUnlockFromISR();
}


#endif
extern uint32_t g_debugUInt32;

bool TLE5012ReadRegister(uint16_t address,uint16_t *value)
{
  uint16_t data;
  SPI_TypeDef *SPIx = SPI2;

  // This routine typically takes 23 us to run.
  SPIEnableWrite(SPIx);

  SPIx->CR1 |= SPI_CR1_SPE; // SPI Enable

  palClearPad(ENC_SPI_NSELECT_GPIO_Port,ENC_SPI_NSELECT_Pin);

  data = SPIx->DR; // This clears the read bit

  uint16_t command = TLE5012CommandAddress(address) | g_TLE5012CommandReadBit | 1 ;
  SPIx->DR = command;
  while((SPIx->SR & SPI_SR_BSY) != 0) ;

  // Introduce a short delay
  for(int i = 0 ;i < 55;i++)
    data += SPIx->SR;

#if USE_DMA
  osalSysLock();
  StartDMARecieve(g_spi2Buffer,2);
  SPIEnableRead(SPIx);
  (void) osalThreadSuspendS(&g_spi2thread);
  osalSysUnlock();

  palSetPad(ENC_SPI_NSELECT_GPIO_Port,ENC_SPI_NSELECT_Pin);

  data = g_spi2Buffer[0];

  *value = data;
  uint16_t safetyWord = g_spi2Buffer[1];

  if((safetyWord & 0x7000) != 0x7000)
    return false;

  // Check the CRC
  {
      uint8_t temp[8];

      temp[0] = command >> 8;
      temp[1] = command;

      temp[2] = data >> 8;
      temp[3] = data;

      uint8_t crcReceived = safetyWord;

      uint8_t crc = crc8(temp, 4);

      if (crc != crcReceived)
        return false;

      g_debugUInt32 = (uint32_t) crcReceived | ((uint32_t) crc << 8);

  }


#else
  SPIEnableRead(SPIx);

#if 0
  while((SPIx->SR & SPI_SR_RXNE) == 0) ;
  data = SPIx->DR; // This clears the read bit
#endif

  while((SPIx->SR & SPI_SR_RXNE) == 0) ;

  data = SPIx->DR; // This clears the read bit

  while((SPIx->SR & SPI_SR_RXNE) == 0) ;

  g_debugUInt32 = SPIx->DR; // This clears the read bit

  SPIEnableWrite(SPIx);

  SPIx->CR1 &= ~SPI_CR1_SPE; // Disable SPI
  palSetPad(ENC_SPI_NSELECT_GPIO_Port,ENC_SPI_NSELECT_Pin);
  *value = data;
#endif
  return true;
}

bool TLE5012ReadAngleInt(int16_t *data)
{
  uint16_t rawData;
  if(!TLE5012ReadRegister(2,&rawData))
    return false;

  rawData = (rawData & (0x7fff));

  // check if the value received is positive or negative
  if (rawData & (1 << 14))
    *data = rawData - 32768;
  else
    *data = rawData;
  return true;
}

bool TLE5012ReadAngleFloat(float *angle)
{
  int16_t rawData;
  if(!TLE5012ReadAngleInt(&rawData))
    return false;
  *angle = ((rawData * M_PI * 2) / 32768.0);
  return true;
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
  for(int i = 0;i < 4;i++)
    g_spi2Buffer[i] = 0;

  palSetPad(ENC_SPI_NSELECT_GPIO_Port,ENC_SPI_NSELECT_Pin);
  rccEnableSPI2(TRUE);

  SPI_TypeDef *SPIx = SPI2;

  //
  // Set things up
  SPIx->CR1 = SPI_CR1_CPHA | SPI_CR1_BIDIMODE | SPI_CR1_BR_1| SPI_CR1_BR_0| SPI_CR1_DFF | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
  SPIx->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
  SPIx->CR2 = 0;

  //nvicEnableVector(36, 1);
  palClearPad(GPIOA,GPIOA_PIN15);

#if USE_DMA

  dmaStreamAllocate(g_spi2dmarx,
                        STM32_SPI_SPI2_IRQ_PRIORITY,
                        (stm32_dmaisr_t)spi_rx_interrupt,
                        (void *)SPIx);
  dmaStreamSetPeripheral(g_spi2dmarx, &SPIx->DR);
  SPIx->CR2 = SPI_CR2_RXDMAEN;

#endif
#endif

  return FC_Ok;
}


