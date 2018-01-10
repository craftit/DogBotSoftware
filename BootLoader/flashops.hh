#ifndef BMC_BOOTLOADER_HEADER
#define BMC_BOOTLOADER_HEADER 1

#include "stm32f4xx_flash.h"

bool BootLoaderReset(bool enable);
bool BootLoaderBeginWrite(uint8_t seqNum,uint32_t address,uint16_t len);
bool BootLoaderErase(uint8_t seqNum,uint32_t address);
bool BootLoaderCheckSum(uint8_t seqNum,uint32_t address,uint16_t len);
bool BootLoaderBeginRead(uint8_t seqNum,uint32_t address,uint16_t len);
bool BootLoaderData(uint8_t seqNum,uint8_t *data,uint8_t len);


#endif
