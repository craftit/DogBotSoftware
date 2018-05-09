
#include "dogbot/FirmwareUpdate.hh"
#include "dogbot/protocol.h"
#include <cassert>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <condition_variable>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


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
#define ADDR_FLASH_SECTOR_END   ((uint32_t)0x08100000) // End of sector 11

// Private constants
static const uint32_t g_flash_addr[FLASH_SECTORS+1] = {
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
                ADDR_FLASH_SECTOR_11,
                ADDR_FLASH_SECTOR_END
};

namespace DogBotN {

  //! State machine for handling firmware update for a set of devices.

  FirmwareUpdateC::FirmwareUpdateC(const std::shared_ptr<ComsC> &coms)
   : m_coms(coms)
  {}

  //! Wait for result.
  bool FirmwareUpdateC::WaitForResult(uint8_t seqNo,const std::string &op)
  {
    //m_log->info("Waiting for seq {} for op '{}' ",(int) seqNo,op);
    using Ms = std::chrono::milliseconds;
    std::unique_lock<std::mutex> lk(m_mutexResult);
    if(!m_condVar.wait_for(lk,Ms(2000),[&]{
      //m_log->info("Got Notify Result:{} ",m_gotResult);
      return m_gotResult;
    })) {
      m_log->error("Timeout waiting for op '{}'.",op);
      return false;
    }
    m_gotResult = false;
    if(m_pktResult.m_rxSequence != seqNo) {
      m_log->warn("Got seq {} (expected {}) , and result {} for op '{}' ",(int) m_pktResult.m_rxSequence,(int) seqNo,(int) m_pktResult.m_result,op);
    }
    return m_pktResult.m_rxSequence == seqNo;
  }

  //! Write data to flash
  bool FirmwareUpdateC::WriteFlash(uint8_t targetDevice,uint32_t targetAddress,uint8_t *data,uint32_t size)
  {
    uint16_t blockSize = size;
    uint8_t *blockBuffer = data;
    SendBootLoaderBeginWrite(targetDevice,++m_seqNo,targetAddress,blockSize);

    WaitForResult(m_seqNo,"write");

    // Send data.
    int at = 0;
    while(at < blockSize) {
      for(int i = 0;i < 8 && at < blockSize;i++) {
        int len = 7;
        int nextAt = at+len;
        if(nextAt > blockSize) {
          len = blockSize - at;
          nextAt = blockSize;
        }
        SendBootLoaderData(targetDevice,++m_seqNo,&blockBuffer[at],len);
        //m_log->info("Seq {}, Sent {} data bytes.",(int) m_seqNo,len);
        at = nextAt;
      }
      // Wait for ack.
      //m_log->info("Waiting for data ack. seq {} ",m_seqNo);
      if(!WaitForResult(m_seqNo,"data")) {
        m_log->error("Failed to get data ack..");
        return false;
      }
    }

    return true;
  }


  bool FirmwareUpdateC::Hex2Number(const char *start,const char *end,uint32_t &value) {
    const char *at = start;
    value = 0;
    for(;at != end;at++) {
      value = value << 4;
      char v = toupper(*at);
      if(v >= '0' && v <='9') {
        value |= v-'0';
      } else if(v >= 'A' && v <= 'F') {
        value |= (v-'A') + 10;
      } else {
        m_log->error("Unexpected character in hex number '{}' ({})",v,(int)v);
        // Unexpected digit.
        return false;
      }
    }
    return true;
  }

  //! Load an intel hex file
  bool FirmwareUpdateC::LoadHexFile(const std::string &filename)
  {
    char hexBuff[1024];

    m_log->info("Reading file '{}' ",filename);
    std::ifstream strm(filename);
    if(!strm.is_open()) {
      m_log->error("Failed to open file '{}' ",filename);
      return false;
    }

    uint32_t startAddress =0;
    uint32_t baseAddress =0;
    uint32_t blockAddress = 0;
    std::vector<uint8_t> currentVector;
    currentVector.reserve((uint32_t)1<<18);
    bool inBlock = false;

    while(strm.good()) {
      strm.getline(hexBuff,1024);
      int len = strm.gcount();
      if(len < 1)
        continue; // Skip line.
      //std::cout << "Line:" << hexBuff << std::endl;
      if(hexBuff[0] != ':') {
        m_log->error("Unexpected line start : {} ",hexBuff);
        return false;
      }
      if(len < 10) {
        m_log->error("Short input line : {} ",hexBuff);
        return false;
      }
      uint32_t digits;
      if(!Hex2Number(&hexBuff[1],&hexBuff[3],digits)) {
        return false;
      }
      //m_log->error("Line len {} expected: {}",len,(9+digits*2+2));
      if(len < (9 + 2 + digits*2)) {
        m_log->error("Line too short : {} ",hexBuff);
        return false;
      }
      uint8_t checkSum = digits;
      uint32_t address;
      if(!Hex2Number(&hexBuff[3],&hexBuff[7],address)) {
        return false;
      }
      checkSum += address & 0xff;
      checkSum += (address >> 8) & 0xff;
      uint32_t recordType;
      if(!Hex2Number(&hexBuff[7],&hexBuff[9],recordType)) {
        return false;
      }
      checkSum += recordType;
      uint8_t data[512];
      for(int i = 0;i < digits;i++) {
        uint32_t value = 0;
        if(!Hex2Number(&hexBuff[9+i*2],&hexBuff[9+i*2+2],value))
          return false;
        data[i] = value;
        checkSum += value;
      }

      uint32_t checkSumValue;
      if(!Hex2Number(&hexBuff[9+digits*2],&hexBuff[9+digits*2+2],checkSumValue)) {
        return false;
      }
      //if(recordType != 0)  m_log->info("Digits: {:02X}  Address: {:04X}  Type: {:02X}  Checksum: {:02X} ",digits,address,recordType,checkSumValue);
      uint8_t computedCheckSum = (1+~checkSum);
      if(checkSumValue != computedCheckSum) {
        m_log->error("Check sum failed. Computed:{:02X}  Given:{:02X} ",(unsigned) computedCheckSum,(unsigned) checkSumValue);
        return false;
      }

      switch(recordType) {
        case 0 : // Data
          if(!inBlock) {
            blockAddress = baseAddress + address;
            inBlock = true;
            currentVector.clear();
          }
          // Append ?
          if((blockAddress + currentVector.size()) != (baseAddress+address)) {
            m_dataMap[blockAddress] = currentVector;
            m_log->info("Block {:08X} Len:{:08X} ",blockAddress,currentVector.size());
            currentVector.clear();
            blockAddress = baseAddress+address;
            //m_log->info("New block {:08X} ",blockAddress);
          }
          for(int i = 0;i < digits;i++)
            currentVector.push_back(data[i]);
          break;
        case 1: // End of file
          goto doneWithFile;
        case 2: // Extended segment address.
        case 3: // Start segment address
          m_log->error("Don't know how to deal with record type {:x} ",recordType);
          return false;
        case 4: // Extended linear address.
          if(!Hex2Number(&hexBuff[9],&hexBuff[9+digits*2],baseAddress)) {
            return false;
          }
          baseAddress = baseAddress << 16;
          //m_log->info("Base address: {:08X} ",baseAddress);
          break;
        case 5: // Start linear address.
          if(!Hex2Number(&hexBuff[9],&hexBuff[9+digits*2],startAddress))
            return false;
          m_log->info("Start address: {:08X} ",startAddress);
          break;
      }
    }
    doneWithFile:
    if(currentVector.size() > 0 && inBlock) {
      m_log->info("Block {:08X} Len:{:08X} ",blockAddress,currentVector.size());
      m_dataMap[blockAddress] = currentVector;
    }

    return true;

  }


  //! Start update

  bool FirmwareUpdateC::DoUpdate(int targetDevice,const std::string &filename)
  {
    using Ms = std::chrono::milliseconds;

    if(!LoadHexFile(filename))
      return false;

    CallbackSetC callbacks;

    if(!m_dryRun) {
      callbacks += m_coms->SetHandler(CPT_FlashCmdResult,[&](const uint8_t *data,int len)
       {
         auto *pkt = (const struct PacketFlashResultC *) data;
         //m_log->info("Got flash result from device {}, waiting for {}  rxSequence:{} State:{} Result:{} ",(int) pkt->m_deviceId,(int) targetDevice,(int) pkt->m_rxSequence,(int) pkt->m_state,(int) pkt->m_result);
         // Is packet of interest ?
         if(pkt->m_deviceId != targetDevice && targetDevice != 0)
           return ;
         std::unique_lock<std::mutex> lk(m_mutexResult);
         if(sizeof(PacketFlashResultC) != len) {
           m_log->error("Unexpected FlashCmdResult packet length. {} ",len);
           m_flagError = true;
           return ;
         }
         memcpy(&m_pktResult,data,sizeof(PacketFlashResultC));
         m_gotResult = true;
         lk.unlock();
         m_condVar.notify_all();
       }
      );

      // FIXME:- Check we're in a state where we can change into the bootloader.

     // if(m_coms->SetParam(targetDevice,CPI_ControlState,(uint8_t) CS_BootLoader))

      // Change device into boot-loader mode.
      // This involves a reboot, so try a few times.
      bool inBootloader = false;
      for(int i = 0;i < 3;i++) {
        if(m_coms->SetParam(targetDevice,CPI_ControlState,(uint8_t) CS_BootLoader)) {
          inBootloader = true;
          break;
        }
        sleep(1);
      }
      if(!inBootloader) {
        m_log->error("Failed to change into boot-loader.");
        return false;
      }

      // Wait for change to happen.

      m_log->info("Setting up connection for device {} ",targetDevice);
      SendBootLoaderReset(targetDevice,true);

      // Wait for ack.
      if(!WaitForResult(0,"reset")) {
        return false;
      }
    }

    m_seqNo = -1;

    m_log->info("Starting erase. ");

    // Decide what blocks to erase.
    std::vector<uint32_t> eraseBlocks;
    for(auto block = m_dataMap.begin(); block != m_dataMap.end();block++) {
      uint32_t targetAddress = block->first;
      uint32_t endAddress = targetAddress + block->second.size();
      if(targetAddress < g_flash_addr[4] || endAddress > g_flash_addr[FLASH_SECTORS]) {
        m_log->error("Block at address {:08X} to {:08X} out of boot loader range",targetAddress,endAddress);
        return false;
      }
      for(int i = 4;i < FLASH_SECTORS;i++) {
        if(targetAddress >= g_flash_addr[i+1])
          continue;
        if(find(eraseBlocks.begin(),eraseBlocks.end(),g_flash_addr[i]) == eraseBlocks.end()) {
          eraseBlocks.push_back(g_flash_addr[i]);
        }
        if(endAddress < g_flash_addr[i+1])
          break;
      }
    }
    for(auto a : eraseBlocks) {
      m_log->info("Erasing block at {:08X} ",a);
      if(!m_dryRun) {
        SendBootLoaderErase(targetDevice,++m_seqNo,a);
        if(!WaitForResult(m_seqNo,"erase"))
          return false;
      }
    }

    m_log->info("Erase completed ok");


    m_log->info("Writing data...");

    for(auto block = m_dataMap.begin(); block != m_dataMap.end();block++) {
      uint32_t targetAddress = block->first;
      std::vector<uint8_t> &data = block->second;
      uint32_t at = 0;
      while(at < data.size()) {
        uint32_t blockSize = data.size();
        if(blockSize >= (1<<16))
          blockSize = (1<<16)-1;
        m_log->info("Writing block at {:08X} of length {:04X} ",targetAddress+at,blockSize);
        if(!m_dryRun) {
          if(!WriteFlash(targetDevice,targetAddress+at,&data[at],blockSize)) {
            m_log->info("Write failed.");
            return false;
          }
        }
        at += blockSize;
      }
    }

    m_log->info("Done.");

    if(m_exitBootloaderOnComplete) {
      // Restart into normal mode
      if(!m_coms->SetParam(targetDevice,CPI_ControlState,(uint8_t) CS_Standby)) {
        m_log->error("Failed to restart the controller.");
      }
    }

    return true;
  }

#if 0

  struct PacketFlashChecksumResultC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_sequenceNumber;
    uint32_t m_sum;
  };

#endif

  //! Send a boot-loader reset
  void FirmwareUpdateC::SendBootLoaderReset(uint8_t deviceId,bool enable)
  {
    struct PacketFlashResetC pkt;
    pkt.m_packetType = CPT_FlashCmdReset;
    pkt.m_deviceId = deviceId;
    pkt.m_enable = enable;
    m_coms->SendPacket((const uint8_t *)&pkt,sizeof(pkt));
  }

  //! Send a boot-loader begin write
  void FirmwareUpdateC::SendBootLoaderBeginWrite(uint8_t deviceId,uint8_t seqNum,uint32_t address,uint16_t len)
  {
    struct PacketFlashWriteC pkt;
    pkt.m_packetType = CPT_FlashWrite;
    pkt.m_deviceId = deviceId;
    pkt.m_sequenceNumber = seqNum;
    pkt.m_addr = address;
    pkt.m_len = len;
    m_coms->SendPacket((const uint8_t *)&pkt,sizeof(pkt));
  }

  //! Send a boot-loader erase sector
  void FirmwareUpdateC::SendBootLoaderErase(uint8_t deviceId,uint8_t seqNum,uint32_t address)
  {
    struct PacketFlashEraseC pkt;
    pkt.m_packetType = CPT_FlashEraseSector;
    pkt.m_deviceId = deviceId;
    pkt.m_sequenceNumber = seqNum;
    pkt.m_addr = address;
    m_coms->SendPacket((const uint8_t *)&pkt,sizeof(pkt));
  }

  //! Send a boot-loader checksum request
  void FirmwareUpdateC::SendBootLoaderCheckSum(uint8_t deviceId,uint8_t seqNum,uint32_t address,uint16_t len)
  {
    struct PacketFlashChecksumC pkt;
    pkt.m_packetType = CPT_FlashChecksum;
    pkt.m_deviceId = deviceId;
    pkt.m_sequenceNumber = seqNum;
    pkt.m_addr = address;
    pkt.m_len = len;
    m_coms->SendPacket((const uint8_t *)&pkt,sizeof(pkt));
  }

  //! Send a boot-loader begin read
  void FirmwareUpdateC::SendBootLoaderBeginRead(uint8_t deviceId,uint8_t seqNum,uint32_t address,uint16_t len)
  {
    struct PacketFlashReadC pkt;
    pkt.m_packetType = CPT_FlashRead;
    pkt.m_deviceId = deviceId;
    pkt.m_sequenceNumber = seqNum;
    pkt.m_addr = address;
    pkt.m_len = len;
    m_coms->SendPacket((const uint8_t *)&pkt,sizeof(pkt));
  }

  //! Send a boot-loader begin read
  void FirmwareUpdateC::SendBootLoaderData(uint8_t deviceId,uint8_t seqNum,uint8_t *data,uint8_t len)
  {
    struct PacketFlashDataBufferC pkt;
    pkt.m_header.m_packetType = CPT_FlashData;
    pkt.m_header.m_deviceId = deviceId;
    pkt.m_header.m_sequenceNumber = seqNum;
    assert(len <= 7);
    memcpy(pkt.m_data,data,len);
    m_coms->SendPacket((const uint8_t *)&pkt,sizeof(pkt.m_header) + len);
  }



}
