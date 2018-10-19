
#include "ch.h"
#include "hal.h"

#include "lan9252.h"
#include <string.h>

#define LANSPI SPID3
/*
 * Low speed SPI configuration (328.125kHz, CPHA=0, CPOL=0, MSb first).
 *
 *
 */
static const SPIConfig ls_spicfg = {
  NULL,
  ETH_SPI_NSELECT_GPIO_Port,
  ETH_SPI_NSELECT_Pin,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPHA | SPI_CR1_CPOL,
  0
};


static void Lan9252SPIExchange(const uint8_t *txbuff,uint8_t *rxbuff,int len)
{
  spiAcquireBus(&LANSPI);              /* Acquire ownership of the bus.    */
  spiStart(&LANSPI, &ls_spicfg);       /* Setup transfer parameters.       */
  spiSelect(&LANSPI);                  /* Slave Select assertion.          */
  spiExchange(&LANSPI,len,
              txbuff, rxbuff);          /* Atomic transfer operations.      */
  spiUnselect(&LANSPI);                /* Slave Select de-assertion.       */
  spiReleaseBus(&LANSPI);              /* Ownership release.               */
}


void Lan9252ResetSQI()
{
  uint8_t rxBuff[1];
  uint8_t txBuff[1];

  txBuff[0] = 0xff; // Low speed write, no dummy data

  Lan9252SPIExchange(txBuff,rxBuff,1);

}

const int g_bufferSize = 16;




uint32_t Lan9252SetRegister32(uint16_t addr,uint32_t value)
{
  uint8_t rxbuff[7];
  uint8_t txbuff[7];

  txbuff[0] = 0x02; // Low speed write, no dummy data
  txbuff[1] = 0x00 | ((addr >> 8) & 0x3f);
  txbuff[2] = addr & 0xff;

  for(int i = 0;i < 4;i++)
    txbuff[i+3] =(value >> (8 * i)) & 0xff;

  Lan9252SPIExchange(txbuff,rxbuff,7);

  uint32_t result = 0;
  for(int i = 0;i < 4;i++)
    result = result | (((uint32_t) rxbuff[i+3]) << (8 * i));
  return result;
}


uint32_t Lan9252ReadRegister32(uint16_t addr)
{
  uint8_t txbuff[7] ;
  uint8_t rxbuff[7];

  txbuff[0] = 0x03; // Low speed read, no dummy data
  txbuff[1] = 0x00 | ((addr >> 8) & 0x3f);
  txbuff[2] = addr & 0xff;
  txbuff[3] = 0;
  txbuff[4] = 0;
  txbuff[5] = 0;
  txbuff[6] = 0;

  Lan9252SPIExchange(txbuff,rxbuff,7);

  uint32_t result = 0;
  for(int i = 0;i < 4;i++)
    result = result | (((uint32_t) rxbuff[i+3]) << (8 * i));

  return result;
}

enum FaultCodeT Lan9252Read(uint16_t addr,uint8_t *data,uint8_t len)
{
  uint8_t txbuff[3+g_bufferSize] ;
  uint8_t rxbuff[3+g_bufferSize];

  if(len > g_bufferSize)
    return FC_Internal;

  txbuff[0] = 0x03; // Low speed read, no dummy data
  txbuff[1] = 0x40 | ((addr >> 8) & 0x3f); // Address with auto increment
  txbuff[2] = addr & 0xff;

  Lan9252SPIExchange(txbuff,rxbuff,len+3);

  memcpy(data,&rxbuff[3],len);

  return FC_Ok;
}


enum FaultCodeT Lan9252Write(uint16_t addr,const uint8_t *data,uint8_t len)
{
  uint8_t txbuff[3+g_bufferSize] ;
  uint8_t rxbuff[3+g_bufferSize];

  if(len > g_bufferSize)
    return FC_Internal;

  txbuff[0] = 0x02; // Low speed write, no dummy data
  txbuff[1] = 0x40 | ((addr >> 8) & 0x3f); // Address with auto increment
  txbuff[2] = addr & 0xff;

  memcpy(&txbuff[3],data,len);

  Lan9252SPIExchange(txbuff,rxbuff,len+3);

  return FC_Ok;
}


uint16_t Lan9252ReadStatus(void)
{
  //uint16_t ret= Lan9252ReadRegister(0x5);
  return 0;
}


enum FaultCodeT InitLan9252(void)
{
  Lan9252ResetSQI();
  enum FaultCodeT ret = FC_Internal;
  const uint64_t expectedValue = 0x87654321;
  for(int i = 0;i < 10;i++) {
    uint64_t val = Lan9252ReadRegister32(0x64);
    if(val == expectedValue) {
      ret = FC_Ok;
      break;
    }
    chThdSleepMilliseconds(10);
  }

  return ret;
}



uint16_t Lan9252Test(void)
{


  return 0;
}
