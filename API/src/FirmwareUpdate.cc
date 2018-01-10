
#include "dogbot/FirmwareUpdate.hh"
#include "dogbot/protocol.h"
#include <cassert>
#include <string.h>
#include <condition_variable>

namespace DogBotN {

  //! State machine for handling firmware update for a set of devices.

  FirmwareUpdateC::FirmwareUpdateC(std::shared_ptr<ComsC> &coms)
   : m_coms(coms)
  {}

  //! Start update

  bool FirmwareUpdateC::DoUpdate(const std::string &filename)
  {
    using Ms = std::chrono::milliseconds;

    uint8_t targetDevice = 0;
    uint32_t targetAddress = 0x08010000;
    uint16_t blockSize = 0;

    uint8_t blockBuffer[1<<16];

    ComsRegisteredCallbackSetC callbacks(m_coms);

    bool m_flagError = false;
    bool m_gotResult = false;
    std::mutex m_mutexResult;
    std::condition_variable m_condVar;
    struct PacketFlashResultC m_pktResult;

    callbacks.SetHandler(CPT_FlashCmdResult,[&](uint8_t *data,int len)
     {
       auto *pkt = (struct PacketFlashResultC *) data;
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


    // Change device into boot-loader mode.
    if(!m_coms->SetParam(targetDevice,CPI_ControlState,CS_BootLoader)) {
      m_log->error("Failed to change into boot-loader.");
      return false;
    }

    // Wait for change to happen.

    SendBootLoaderReset(targetDevice,true);

    // Wait for ack.
    {
      std::unique_lock<std::mutex> lk(m_mutexResult);
      if(!m_condVar.wait_for(lk,Ms(1000),[&]{
        if(!m_gotResult)
          return false;
        return (m_pktResult.m_packetType == CPT_FlashCmdReset);
      })) {
        m_log->error("Timeout waiting for reset.");
        return false;
      }
      m_gotResult = false;
    }

    uint8_t seqNo = 0;

    SendBootLoaderErase(targetDevice,seqNo++,targetAddress);

    // Wait for ack.
    {
      std::unique_lock<std::mutex> lk(m_mutexResult);
      if(!m_condVar.wait_for(lk,Ms(2000),[&]{
        if(!m_gotResult)
          return false;
        return (m_pktResult.m_packetType == CPT_FlashEraseSector);
      }))
      {
        m_log->error("Timeout waiting for erase.");
        return false;
      }
      m_gotResult = false;
    }

    SendBootLoaderBeginWrite(targetDevice,seqNo++,targetAddress,blockSize);

    // Send data.
    int at = 0;
    while(at < blockSize) {
      for(int i = 0;i < 16 && at < blockSize;i++) {
        int len = 7;
        int nextAt = at+len;
        if(nextAt > blockSize) {
          len = blockSize - nextAt;
          nextAt = blockSize;
        }
        SendBootLoaderData(targetDevice,seqNo++,&blockBuffer[at],len);
        at = nextAt;
      }
      // Wait for ack.
      {
        std::unique_lock<std::mutex> lk(m_mutexResult);
        if(!m_condVar.wait_for(lk,Ms(1000),[&]{
          if(!m_gotResult)
            return false;
          return (m_pktResult.m_packetType == CPT_FlashData);
        })) {
          m_log->error("Timeout waiting for write.");
          return false;
        }
        m_gotResult = false;
      }
    }

    // Restart into normal mode
    if(!m_coms->SetParam(targetDevice,CPI_ControlState,CS_StartUp)) {
      m_log->error("Failed to restart the controller.");
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

  CPT_FlashCmdReset   = 20, // Status from a flash command
  CPT_FlashCmdResult  = 21, // Status from a flash command
  CPT_FlashChecksumResult = 22, // Generate a checksum
  CPT_FlashEraseSector = 23, // Erase a flash sector
  CPT_FlashChecksum    = 24, // Generate a checksum
  CPT_FlashData        = 25, // Data packet
  CPT_FlashWrite       = 26, // Write buffer
  CPT_FlashRead        = 27  // Read buffer and send it back
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
