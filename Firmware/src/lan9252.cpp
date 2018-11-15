
#include "ch.h"
#include "hal.h"

#include "lan9252.h"
#include <string.h>

#define LANSPI SPID3
/*
 * Low speed SPI configuration (21MBits/s, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig ls_spicfg = {
  NULL,
  ETH_SPI_NSELECT_GPIO_Port,
  ETH_SPI_NSELECT_Pin,
  SPI_CR1_CPHA | SPI_CR1_CPOL,
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

const int g_bufferSize = 32;


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

#define ESC_CSR_CMD_REG         0x304
#define ESC_CSR_DATA_REG        0x300
#define ESC_WRITE_BYTE          0x80
#define ESC_READ_BYTE           0xC0
#define ESC_CSR_BUSY            0x80

#define ECAT_REG_BASE_ADDR              0x0300

#define CSR_DATA_REG_OFFSET             0x00
#define CSR_CMD_REG_OFFSET              0x04
#define PRAM_READ_ADDR_LEN_OFFSET       0x08
#define PRAM_READ_CMD_OFFSET            0x0c
#define PRAM_WRITE_ADDR_LEN_OFFSET      0x10
#define PRAM_WRITE_CMD_OFFSET           0x14

#define PRAM_SPACE_AVBL_COUNT_MASK      0x1f
#define IS_PRAM_SPACE_AVBL_MASK         0x01

#define CSR_DATA_REG                    ECAT_REG_BASE_ADDR+CSR_DATA_REG_OFFSET
#define CSR_CMD_REG                     ECAT_REG_BASE_ADDR+CSR_CMD_REG_OFFSET
#define PRAM_READ_ADDR_LEN_REG          ECAT_REG_BASE_ADDR+PRAM_READ_ADDR_LEN_OFFSET
#define PRAM_READ_CMD_REG               ECAT_REG_BASE_ADDR+PRAM_READ_CMD_OFFSET
#define PRAM_WRITE_ADDR_LEN_REG         ECAT_REG_BASE_ADDR+PRAM_WRITE_ADDR_LEN_OFFSET
#define PRAM_WRITE_CMD_REG              ECAT_REG_BASE_ADDR+PRAM_WRITE_CMD_OFFSET

#define PRAM_READ_FIFO_REG              0x04
#define PRAM_WRITE_FIFO_REG             0x20

#define PRAM_RW_ABORT_MASK      (1 << 30)
#define PRAM_RW_BUSY_32B        (1 << 31)
#define PRAM_RW_BUSY_8B         (1 << 7)
#define PRAM_SET_READ           (1 << 6)
#define PRAM_SET_WRITE          0

union param32_t {
  uint32_t value;
  uint32_t uint32[1];
  uint16_t uint16[2];
  uint8_t uint8[4];
} ;

union param64_t {
  uint64_t value;
  uint64_t uint64[1];
  uint32_t uint32[2];
  uint16_t uint16[4];
  uint8_t uint8[8];
} ;


enum FaultCodeT Lan9252ReadCSR(uint16_t addr,uint8_t *data,uint8_t len)
{
  if(len != 1 && len != 2 && len != 4)
    return FC_Internal;

  union param32_t buff;

  // Send write command
  buff.uint8[0] = addr & 0xff;
  buff.uint8[1] = (addr >> 8) & 0xff;
  buff.uint8[2] = len;
  buff.uint8[3] = ESC_READ_BYTE;

  Lan9252SetRegister32(ESC_CSR_CMD_REG,buff.value);

  do {
    buff.value = Lan9252ReadRegister32(ESC_CSR_CMD_REG);
  } while(buff.value & ESC_CSR_BUSY);

  buff.value = Lan9252ReadRegister32(ESC_CSR_DATA_REG);

  memcpy(data,buff.uint8,len);

  return FC_Ok;
}

enum FaultCodeT Lan9252WriteCSR(uint16_t addr,const uint8_t *data,uint8_t len)
{
  if(len != 1 && len != 2 && len != 4)
    return FC_Internal;

  union param32_t buff;
  memcpy(buff.uint8,data,len);

  // Set data register
  Lan9252SetRegister32(ESC_CSR_DATA_REG,buff.value);


  // Send write command
  buff.uint8[0] = addr & 0xff;
  buff.uint8[1] = (addr >> 8) & 0xff;
  buff.uint8[2] = len;
  buff.uint8[3] = ESC_WRITE_BYTE;

  Lan9252SetRegister32(ESC_CSR_CMD_REG,buff.value);

  // Wait until write is complete
  do {
    buff.value = Lan9252ReadRegister32(ESC_CSR_CMD_REG);
  } while(buff.value & ESC_CSR_BUSY);

  return FC_Ok;
}



// Read from the process ram

extern enum FaultCodeT Lan9252ReadPDRam(uint16_t address, uint8_t *readBuffer, uint16_t count)
{
  union param64_t param64;

  // Reset/Abort any previous commands.
  param64.value = PRAM_RW_ABORT_MASK;
  Lan9252SetRegister32(PRAM_READ_CMD_REG, param64.value);

  // The host should not modify this field unless the PRAM Read Busy
  // (PRAM_READ_BUSY) bit is a 0.
  do {
    param64.value = Lan9252ReadRegister32(PRAM_READ_CMD_REG);
  } while((param64.uint8[3] & PRAM_RW_BUSY_8B));

  // Write Address and Length Register (PRAM_READ_ADDR_LEN) with the
  // starting uint8_t address and length) and Set PRAM Read Busy (PRAM_READ_BUSY) bit
  // (-EtherCAT Process RAM Read Command Register) to start read operation
  param64.uint16[0] = address;
  param64.uint16[1] = count;
  param64.uint16[2] = 0x0;
  param64.uint16[3] = 0x8000;

  Lan9252Write(PRAM_READ_ADDR_LEN_REG,param64.uint8,8);

  uint16_t readAddr = PRAM_READ_FIFO_REG;
  int alignmentOffset = (address & 0x03);
  int size = count;

  while(size > 0) {
    // Read PRAM write Data Available (PRAM_WRITE_AVAIL) bit is set
    do {
      param64.value = Lan9252ReadRegister32 (PRAM_READ_CMD_REG);
    } while(!(param64.uint8[0] & IS_PRAM_SPACE_AVBL_MASK));

    // Check write data available count
    int fifoSpace = param64.uint8[1] & PRAM_SPACE_AVBL_COUNT_MASK;
    int toRead = size + alignmentOffset;

    // Limit read to available space in fifo
    if(toRead > fifoSpace) {
      toRead = fifoSpace;
    } else {
      // Are we reading a partial 32 bit value at the end?
      int partial = toRead & 0x3;
      if(partial != 0) {
        // Round it up to a power of 4.
        toRead += 4 - partial;
      }
    }

    {
      uint8_t txbuff[3+g_bufferSize+8] ;
      uint8_t rxbuff[3+g_bufferSize+8];

      txbuff[0] = 0x03; // Low speed read, no dummy data
      txbuff[1] = 0x40 | ((readAddr >> 8) & 0x3f); // Address with auto increment
      txbuff[2] = readAddr & 0xff;

      Lan9252SPIExchange(txbuff,rxbuff,toRead+3);

      int requestedBytes = toRead-alignmentOffset;
      // Make sure we only copy in the bytes the caller actually requested.
      if(requestedBytes > size)
        requestedBytes = size;

      memcpy(readBuffer,&txbuff[3+alignmentOffset],requestedBytes);
    }

    toRead -= alignmentOffset; // Remove alignment offset from total
    size -= toRead;
    readBuffer += toRead;
    alignmentOffset = 0;
  }

  return FC_Ok;
}

// This function writes the PDRAM using LAN9252 FIFO.

enum FaultCodeT Lan9252WritePDRam(uint16_t address, const uint8_t *writeBuffer, uint16_t count)
{
  union param64_t param64;

  // Reset or Abort any previous commands.
  param64.value = PRAM_RW_ABORT_MASK;

  Lan9252SetRegister32(PRAM_WRITE_CMD_REG, param64.value);

  // Make sure there is no previous write is pending (PRAM Write Busy) bit is a 0
  do {
    param64.value = Lan9252ReadRegister32 (PRAM_WRITE_CMD_REG);
  } while((param64.uint8[3] & PRAM_RW_BUSY_8B));

  // Write Address and Length Register (ECAT_PRAM_WR_ADDR_LEN) with the starting
  // uint8_t address and length) and write to the EtherCAT Process RAM Write Command Register (ECAT_PRAM_WR_CMD)
  // with the  PRAM Write Busy (PRAM_WRITE_BUSY) bit set

  param64.uint16[0] = address;
  param64.uint16[1] = count;
  param64.uint16[2] = 0x0;
  param64.uint16[3] = 0x8000;

  Lan9252Write(PRAM_WRITE_ADDR_LEN_REG, param64.uint8,8);

  uint16_t writeAddr = PRAM_WRITE_FIFO_REG;
  int alignmentOffset = (address & 0x03);

  int size = count;

  while(size > 0) {
    // Read PRAM write Data Available (PRAM_WRITE_AVAIL) bit is set
    do {
      param64.value = Lan9252ReadRegister32 (PRAM_WRITE_CMD_REG);
    } while(!(param64.uint8[0] & IS_PRAM_SPACE_AVBL_MASK));

    // Check write data available count
    int fifoSpace = param64.uint8[1] & PRAM_SPACE_AVBL_COUNT_MASK;
    int toWrite = size + alignmentOffset;

    // Limit write to available space
    if(toWrite > fifoSpace) {
      toWrite = fifoSpace;
    } else {
      // Are we writing a partial 32 bit value at the end?
      int partial = toWrite & 0x3;
      if(partial != 0) {
        // Round it up to a power of 4.
        toWrite += 4 - partial;
      }
    }

    {
      uint8_t txbuff[3+g_bufferSize+8] ;
      uint8_t rxbuff[3+g_bufferSize+8];

      txbuff[0] = 0x02; // Low speed write, no dummy data
      txbuff[1] = 0x40 | ((writeAddr >> 8) & 0x3f); // Address with auto increment
      txbuff[2] = writeAddr & 0xff;
      memcpy(&txbuff[3+alignmentOffset],writeBuffer,toWrite);

      Lan9252SPIExchange(txbuff,rxbuff,toWrite+3);
    }

    toWrite -= alignmentOffset; // Remove alignment offset from total
    size -= toWrite;
    writeBuffer += toWrite;
    alignmentOffset = 0;
  }

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
