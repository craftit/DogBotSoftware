
#include "flashops.hh"
#include "canbus.h"
#include "hal.h"
#include "coms.h"

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

void SendBootLoaderResult(uint8_t lastSeqNum,enum BootLoaderStateT state,enum FlashOperationStatusT result)
{
  if(g_canBridgeMode) {
    USBSendBootLoaderResult(g_deviceId,lastSeqNum,state,result);
  }
  CANSendBootLoaderResult(g_deviceId,lastSeqNum,state,result);
}

bool SendBootLoaderCheckSumResult(uint8_t seqNum,uint32_t sum)
{
  if(g_canBridgeMode) {
    USBSendBootLoaderCheckSumResult(g_deviceId,seqNum,sum);
  }
  CANSendBootLoaderCheckSumResult(g_deviceId,seqNum,sum);
  return true;
}

void SendBootLoaderData(uint8_t seqNum,uint8_t *data,uint8_t len)
{
  if(g_canBridgeMode) {
    USBSendBootLoaderData(g_deviceId,seqNum,data,len);
  }
  CANSendBootLoaderData(g_deviceId,seqNum,data,len);
}


enum BootLoaderStateT g_bootLoaderState = BLS_Disabled;


static uint8_t g_bootLoader_lastSeqNum = 0;
static uint32_t g_bootLoader_address = 0;
static uint16_t g_bootLoader_len = 0;
static uint32_t g_bootLoader_at = 0;
static uint32_t g_bootLoader_lastAck = 0;
static uint8_t g_bootLoaderTxSequenceNumber = 0;


bool BootLoaderCheckSequence(uint8_t seqNum,enum ComsPacketTypeT packetType)
{
  if(g_bootLoader_lastSeqNum != seqNum) {
    SendError(CET_BootLoaderLostSequence,packetType,g_bootLoader_lastSeqNum);
    return false;
  }
  g_bootLoader_lastSeqNum++;
  return true;
}

bool BootLoaderReset(bool enable)
{
  g_bootLoaderTxSequenceNumber = 0;
  if(enable)
    g_bootLoaderState = BLS_Ready;
  else
    g_bootLoaderState = BLS_Disabled;
  g_bootLoader_lastSeqNum = 0;
  g_bootLoader_lastAck = 0;
  g_bootLoader_len = 0;
  g_bootLoader_at = 0;
  g_bootLoader_address = 0;

  SendBootLoaderResult(0,g_bootLoaderState,FOS_Ok);
  return true;
}

bool BootLoaderErase(uint8_t seqNum,uint32_t blockAddress)
{
  if(!BootLoaderCheckSequence(seqNum,CPT_FlashEraseSector))
    return false;
  if(g_bootLoaderState == BLS_Disabled)
    return true;
  if(g_bootLoaderState != BLS_Ready && g_bootLoaderState != BLS_Write) {
    SendError(CET_BootLoaderUnexpectedState,CPT_FlashEraseSector,g_bootLoaderState);
    return false;
  }

  int sectorNumber = -1;
  for(int i = 0;i < FLASH_SECTORS;i++) {
    if(g_flash_addr[i] == blockAddress) {
      sectorNumber = i;
      break;
    }
  }

  if(sectorNumber < 0) {
    SendError(CET_BootLoaderUnalignedAddress,CPT_FlashEraseSector,blockAddress >> 16);
    return false;
  }

  // Be really inflexible for the moment
  if(sectorNumber < 3){
    SendError(CET_BootLoaderProtected,CPT_FlashEraseSector,sectorNumber);
    return false;
  }

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR);

  int ret = FLASH_EraseSector(g_flash_sector[sectorNumber],VoltageRange_3);

  FLASH_Lock();

  if(ret != 0 && ret != FLASH_COMPLETE) {
    SendError(CET_BootLoaderErase,CPT_FlashEraseSector,ret);
  }
  SendBootLoaderResult(seqNum,g_bootLoaderState,FOS_Ok);
  return true;
}


bool BootLoaderBeginWrite(uint8_t seqNum,uint32_t address,uint16_t len)
{
  if(!BootLoaderCheckSequence(seqNum,CPT_FlashWrite))
    return false;
  if(g_bootLoaderState == BLS_Disabled)
    return true;
  if(g_bootLoaderState != BLS_Ready) {
    SendError(CET_BootLoaderUnexpectedState,CPT_FlashWrite,g_bootLoaderState);
    return false;
  }
  // Check alignment of address
  if((address & 0x3) != 0) {
    g_bootLoaderState = BLS_Error;
    SendError(CET_BootLoaderUnalignedAddress,CPT_FlashWrite,g_bootLoaderState);
    return false;
  }
  if(address < 0x08010000) {
    g_bootLoaderState = BLS_Error;
    SendError(CET_BootLoaderProtected,CPT_FlashWrite,g_bootLoaderState);
    return false;
  }

  g_bootLoader_address = address;
  g_bootLoader_at = address;
  g_bootLoader_lastAck = 0;
  g_bootLoader_len = len;
  g_bootLoaderState = BLS_Write;
  SendBootLoaderResult(seqNum,BLS_Write,FOS_Ok);
  return true;
}


bool BootLoaderData(uint8_t seqNum,uint8_t *data,uint8_t len)
{
  if(!BootLoaderCheckSequence(seqNum,CPT_FlashData)) {
   // return false;
  }
  if(g_bootLoaderState == BLS_Disabled)
    return true;
  if(g_bootLoaderState != BLS_Write) {
    SendError(CET_BootLoaderUnexpectedState,CPT_FlashData,g_bootLoaderState);
    return false;
  }

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR);

  // FIXME:- Do stuff!

  union WriteBufferT {
    uint8_t uint8[4];
    uint32_t uint32[1];
  } ;

  static union WriteBufferT g_bootLoaderBuffer;

  for(int i = 0;i < len;i++,g_bootLoader_at++) {
    int ringAt = g_bootLoader_at & 0x03;
    g_bootLoaderBuffer.uint8[ringAt] = data[i];
    if(ringAt == 3) {
      uint32_t addr = g_bootLoader_at & 0xfffffffc;
      if (FLASH_ProgramWord(addr,g_bootLoaderBuffer.uint32[0]) != FLASH_COMPLETE) {
        SendError(CET_BootLoaderWriteFailed,CPT_FlashData,seqNum);
        g_bootLoaderState = BLS_Error;
        FLASH_Lock();
        return 1;
      }
      g_bootLoaderBuffer.uint32[0] = 0;
    }
  }
  // Write any remaining bytes in buffer.
  if(g_bootLoader_at >= (g_bootLoader_address + g_bootLoader_len)) {
    uint32_t addr = g_bootLoader_at & 0xfffffffc;
    for(int i = 0;i < (int) (g_bootLoader_at & 0x3);i++) {
      if (FLASH_ProgramByte(addr+i,g_bootLoaderBuffer.uint8[i]) != FLASH_COMPLETE) {
        SendError(CET_BootLoaderWriteFailed,CPT_FlashData,seqNum);
        g_bootLoaderState = BLS_Error;
        FLASH_Lock();
        return 1;
      }
    }
    g_bootLoaderState = BLS_Ready;
    SendBootLoaderResult(seqNum,BLS_Write,FOS_WriteComplete);
  } else {
    // Send an acknowledge every 16 messages so we can throttle sending data appropriately
    g_bootLoader_lastAck++;
    if(g_bootLoader_lastAck >= 8) {
      g_bootLoader_lastAck = 0;
      SendBootLoaderResult(seqNum,BLS_Write,FOS_DataAck);
    }
  }

  FLASH_Lock();
  return true;
}



bool BootLoaderCheckSum(uint8_t seqNum,uint32_t address,uint16_t len)
{
  if(!BootLoaderCheckSequence(seqNum,CPT_FlashChecksum))
    return false;
  if(g_bootLoaderState == BLS_Disabled)
    return true;
  if(g_bootLoaderState == BLS_Error) {
    SendError(CET_BootLoaderUnexpectedState,CPT_FlashChecksum,g_bootLoaderState);
    return false;
  }
  uint32_t sum = 0;
  uint8_t *at = (uint8_t *) address;
  uint8_t *end = at + len;
  for(;at < end;at++)
    sum += *at;

  return SendBootLoaderCheckSumResult(seqNum,sum);
}

/*
 * Send data.
 */
static uint8_t g_bootLoaderSequenceRead = 0;

static THD_WORKING_AREA(waThreadBootLoadRead, 128);
static THD_FUNCTION(ThreadBootLoadRead, arg) {
  (void) arg;

  uint8_t *at = (uint8_t *) g_bootLoader_address;
  uint8_t *end = at + g_bootLoader_len;
  for(;at < end &&
      g_bootLoaderState == BLS_Read &&
      !chThdShouldTerminateX();
  )
  {
    uint8_t *next = at + 7;
    if(next <= end) {
      SendBootLoaderData(g_bootLoaderTxSequenceNumber++,at,7);
      at = next;
    } else {
      int len = end - at;
      if(len > 0)
        SendBootLoaderData(g_bootLoaderTxSequenceNumber++,at,len);
      at += len;
      break;
    }
    chThdSleepMicroseconds(300); // Throttle things a little.
  }

  g_bootLoaderState = BLS_Ready;
  SendBootLoaderResult(g_bootLoaderSequenceRead,BLS_Ready,FOS_Ok);

}

static thread_t *g_bootLoadThreadThread = 0;

bool BootLoaderBeginRead(uint8_t seqNum,uint32_t address,uint16_t len)
{
  if(!BootLoaderCheckSequence(seqNum,CPT_FlashRead))
    return false;
  if(g_bootLoaderState == BLS_Disabled)
    return true;
  // We can only being read from idle.
  if(g_bootLoaderState != BLS_Ready) {
    SendError(CET_BootLoaderUnexpectedState,CPT_FlashRead,g_bootLoaderState);
    return false;
  }

  // Check thread isn't still running.

  if(g_bootLoadThreadThread != 0 && !chThdTerminatedX(g_bootLoadThreadThread)) {
    SendError(CET_BootLoaderBusy,CPT_FlashRead,g_bootLoaderState);
    return false;
  }

  g_bootLoader_address = address;
  g_bootLoader_at = address;
  g_bootLoader_len = len;
  g_bootLoaderState = BLS_Read;
  g_bootLoaderSequenceRead = seqNum;

  SendBootLoaderResult(g_bootLoaderSequenceRead,BLS_Read,FOS_Ok);

  g_bootLoadThreadThread = chThdCreateStatic(waThreadBootLoadRead, sizeof(waThreadBootLoadRead), NORMALPRIO, ThreadBootLoadRead, NULL);

  return true;
}

