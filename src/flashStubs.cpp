#include "flashops.hh"

// These functions are only available in the boot-loader, but we want to keep the coms code common.

bool BootLoaderReset(bool enable)
{
  (void) enable;
  return false;
}

bool BootLoaderBeginWrite(uint8_t seqNum,uint32_t address,uint16_t len)
{
  (void) seqNum;
  (void) address;
  (void) len;
  return false;
}

bool BootLoaderErase(uint8_t seqNum,uint32_t blockAddress)
{
  (void) seqNum;
  (void) blockAddress;
  return false;
}

bool BootLoaderCheckSum(uint8_t seqNum,uint32_t address,uint16_t len)
{
  (void) seqNum;
  (void) address;
  (void) len;
  return false;
}

bool BootLoaderBeginRead(uint8_t seqNum,uint32_t address,uint16_t len)
{
  (void) seqNum;
  (void) address;
  (void) len;
  return false;
}

bool BootLoaderData(uint8_t seqNum,uint8_t *data,uint8_t len)
{
  (void) seqNum;
  (void) data;
  (void) len;
  return false;
}
