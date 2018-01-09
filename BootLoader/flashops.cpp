
#include "flashops.hh"
#include "canbus.h"

#define FLASH_SECTORS                   12

// Base address of the Flash sectors
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) // Base @ of Sector 0, 16 Kbytes
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) // Base @ of Sector 1, 16 Kbytes
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) // Base @ of Sector 2, 16 Kbytes
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) // Base @ of Sector 3, 16 Kbytes
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) // Base @ of Sector 4, 64 Kbytes
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) // Base @ of Sector 5, 128 Kbytes
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) // Base @ of Sector 6, 128 Kbytes
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) // Base @ of Sector 7, 128 Kbytes
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) // Base @ of Sector 8, 128 Kbytes
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) // Base @ of Sector 9, 128 Kbytes
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) // Base @ of Sector 10, 128 Kbytes
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) // Base @ of Sector 11, 128 Kbytes

// Private constants
static const uint32_t g_flash_addr[FLASH_SECTORS] = {
                ADDR_FLASH_SECTOR_0,
                ADDR_FLASH_SECTOR_1,
                ADDR_FLASH_SECTOR_2,
                ADDR_FLASH_SECTOR_3,
                ADDR_FLASH_SECTOR_4,
                ADDR_FLASH_SECTOR_5,
                ADDR_FLASH_SECTOR_6,
                ADDR_FLASH_SECTOR_7,
                ADDR_FLASH_SECTOR_8,
                ADDR_FLASH_SECTOR_9,
                ADDR_FLASH_SECTOR_10,
                ADDR_FLASH_SECTOR_11
};
static const uint16_t g_flash_sector[12] = {
                FLASH_Sector_0,
                FLASH_Sector_1,
                FLASH_Sector_2,
                FLASH_Sector_3,
                FLASH_Sector_4,
                FLASH_Sector_5,
                FLASH_Sector_6,
                FLASH_Sector_7,
                FLASH_Sector_8,
                FLASH_Sector_9,
                FLASH_Sector_10,
                FLASH_Sector_11
};

uint8_t* flash_helper_get_sector_address(uint32_t fsector) {
  uint8_t *res = 0;

  for(int i = 0; i < FLASH_SECTORS; i++) {
    if(g_flash_sector[i] == fsector) {
      res = (uint8_t *) g_flash_addr[i];
      break;
    }
  }

  return res;
}

enum BootLoaderStateT g_bootLoaderState = BLS_Idle;


static uint8_t g_bootLoader_lastSeqNum = 0;
static uint32_t g_bootLoader_address = 0;
static uint16_t g_bootLoader_len = 0;
static uint32_t g_bootLoader_at = 0;
static uint8_t g_bootLoaderTxSequenceNumber = 0;
static uint8_t g_bootLoaderBuffer[8];


bool BootLoaderCheckSequence(uint8_t seqNum,enum ComsPacketTypeT packetType)
{
  if(g_bootLoader_lastSeqNum != seqNum) {
    CANSendError(CET_BootLoaderLostSequence,packetType,g_bootLoader_lastSeqNum);
    return false;
  }
  g_bootLoader_lastSeqNum++;
  return true;
}

bool BootLoaderReset()
{
  g_bootLoaderTxSequenceNumber = 0;
  g_bootLoaderState = BLS_Idle;
  g_bootLoader_lastSeqNum = 0;

  CANSendBootLoaderResult(0,BLS_Idle,FOS_Ok);
  return true;
}


bool BootLoaderBeginWrite(uint8_t seqNum,uint32_t address,uint16_t len)
{
  if(!BootLoaderCheckSequence(seqNum,CPT_FlashWrite))
    return false;

  if(g_bootLoaderState != BLS_Idle) {
    CANSendError(CET_BootLoaderUnexpectState,CPT_FlashWrite,g_bootLoaderState);
    return false;
  }
  g_bootLoader_address = address;
  g_bootLoader_at = address;
  g_bootLoader_len = len;
  g_bootLoaderState = BLS_Write;
  CANSendBootLoaderResult(seqNum,BLS_Write,FOS_Ok);
  return true;
}

bool BootLoaderErase(uint8_t seqNum)
{
  if(!BootLoaderCheckSequence(seqNum,CPT_FlashEraseSector))
    return false;
  if(g_bootLoaderState != BLS_Write) {
    CANSendError(CET_BootLoaderUnexpectState,CPT_FlashEraseSector,g_bootLoaderState);
    return false;
  }

  uint32_t address = g_bootLoader_address;

  // Be really inflexible for the moment
  int sectorNumber = 4;
  if(address != 0x0801000){
    CANSendError(CET_BootLoaderProtected,CPT_FlashEraseSector,sectorNumber);
    return false;
  }

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR);

  int ret = FLASH_EraseSector(g_flash_sector[sectorNumber],VoltageRange_3);

  FLASH_Lock();

  if(ret != 0) {
    CANSendError(CET_BootLoaderErase,CPT_FlashEraseSector,ret);
  }
  CANSendBootLoaderResult(seqNum,BLS_Write,FOS_Ok);
  return true;
}

bool BootLoaderCheckSum(uint8_t seqNum,uint32_t address,uint16_t len)
{
  if(!BootLoaderCheckSequence(seqNum,CPT_FlashChecksum))
    return false;
  if(g_bootLoaderState == BLS_Error) {
    CANSendError(CET_BootLoaderUnexpectState,CPT_FlashChecksum,g_bootLoaderState);
    return false;
  }
  uint32_t sum = 0;
  uint8_t *at = (uint8_t *) address;
  uint8_t *end = at + len;
  for(;at < end;at++)
    sum += *at;

  return CANSendBootLoaderCheckSumResult(seqNum,sum);
}

/*
 * Send data.
 */
static uint8_t g_bootLoaderSequenceRead = 0;

static THD_WORKING_AREA(waThreadBootLoadRead, 128);
static THD_FUNCTION(ThreadBootLoadRead, arg) {

  uint8_t *at = (uint8_t *) g_bootLoader_address;
  uint8_t *end = at + g_bootLoader_len;
  for(;at < end &&
      g_bootLoaderState == BLS_Read &&
      !chThdShouldTerminateX();
  )
  {
    uint8_t *next = at + 7;
    if(next <= end) {
      CANSendBootLoaderData(g_bootLoaderTxSequenceNumber++,at,7);
      at = next;
    } else {
      int len = end - at;
      if(len > 0)
        CANSendBootLoaderData(g_bootLoaderTxSequenceNumber++,at,len);
      at += len;
      break;
    }
    chThdSleepMicroseconds(300); // Throttle things a little.
  }

  g_bootLoaderState = BLS_Idle;
  CANSendBootLoaderResult(g_bootLoaderSequenceRead,BLS_Idle,FOS_Ok);

}

static thread_t *g_bootLoadThreadThread = 0;

bool BootLoaderBeginRead(uint8_t seqNum,uint32_t address,uint16_t len)
{
  if(!BootLoaderCheckSequence(seqNum,CPT_FlashRead))
    return false;

  // We can only being read from idle.
  if(g_bootLoaderState != BLS_Idle) {
    CANSendError(CET_BootLoaderUnexpectState,CPT_FlashRead,g_bootLoaderState);
    return false;
  }

  // Check thread isn't still running.

  if(g_bootLoadThreadThread != 0 && !chThdTerminatedX(g_bootLoadThreadThread)) {
    CANSendError(CET_BootLoaderBusy,CPT_FlashRead,g_bootLoaderState);
    return false;
  }

  g_bootLoader_address = address;
  g_bootLoader_at = address;
  g_bootLoader_len = len;
  g_bootLoaderState = BLS_Read;
  g_bootLoaderSequenceRead = seqNum;

  CANSendBootLoaderResult(g_bootLoaderSequenceRead,BLS_Read,FOS_Ok);

  g_bootLoadThreadThread = chThdCreateStatic(waThreadBootLoadRead, sizeof(waThreadBootLoadRead), NORMALPRIO, ThreadBootLoadRead, NULL);

  return true;
}


bool BootLoaderData(uint8_t seqNum,uint8_t *data,uint8_t len)
{
  if(!BootLoaderCheckSequence(seqNum,CPT_FlashData))
    return false;
  if(g_bootLoaderState != BLS_Write) {
    CANSendError(CET_BootLoaderUnexpectState,CPT_FlashData,g_bootLoaderState);
    return false;
  }

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR);

  // FIXME:- Do stuff!

  //g_bootLoader_at;
  g_bootLoaderBuffer[0] = 0;
  uint32_t xdata = 0;

  if (FLASH_ProgramWord(g_bootLoader_at,xdata) != FLASH_COMPLETE) {
    CANSendError(CET_BootLoaderWriteFailed,CPT_FlashData,seqNum);

    g_bootLoaderState = BLS_Error;

    FLASH_Lock();
    return 1;
  }

  FLASH_Lock();

  return true;
}

