
#include "dogbot/FirmwareUpdate.hh"
#include "dogbot/protocol.h"
#include <cassert>
#include <string.h>
#include <fstream>
#include <condition_variable>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

namespace DogBotN {

  static long GetFileSize(std::string filename)
  {
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
  }

  //! State machine for handling firmware update for a set of devices.

  FirmwareUpdateC::FirmwareUpdateC(const std::shared_ptr<ComsC> &coms)
   : m_coms(coms)
  {}

  //! Wait for result.
  bool FirmwareUpdateC::WaitForResult(uint8_t seqNo,const std::string &op)
  {
    m_log->info("Waiting for seq {} for op '{}' ",(int) seqNo,op);
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
    m_log->info("Got seq {} (expected {}) , and result {} for op '{}' ",(int) m_pktResult.m_rxSequence,(int) seqNo,(int) m_pktResult.m_result,op);
    return m_pktResult.m_rxSequence == seqNo;
  }

  //! Start update

  bool FirmwareUpdateC::DoUpdate(int targetDevice,const std::string &filename)
  {
    using Ms = std::chrono::milliseconds;

    uint32_t targetAddress = 0x08020000;
    uint16_t blockSize = 0;

    uint8_t blockBuffer[1<<16];

    for(int i = 0;i < 1<<16;i++)
      blockBuffer[i] = i;
    blockSize = 128;

#if 0
    {
      long fileSize = GetFileSize(filename);
      if(fileSize < 0) {
        m_log->error("Failed to find file '{}' ",filename);
        return false;
      }
      std::ifstream inf(filename,inf.binary);
      if(!inf.is_open()) {
        m_log->error("Failed to open file '{}' ",filename);
        return false;
      }
#if 0
      if(fileSize >= 1<<16) {
        m_log->error("File {} to large. {} ",filename,fileSize);
        return false;
      }
      inf.read((char *)blockBuffer,fileSize);
      blockSize = inf.gcount();
#endif
      m_log->info("Read {} bytes of {}. ",blockSize,fileSize);
    }
#endif

    ComsRegisteredCallbackSetC callbacks(m_coms);


    callbacks.SetHandler(CPT_FlashCmdResult,[&](uint8_t *data,int len)
     {
       auto *pkt = (struct PacketFlashResultC *) data;
       m_log->info("Got flash result from device {}, waiting for {}  rxSequence:{} State:{} Result:{} ",
                   (int) pkt->m_deviceId,(int) targetDevice,(int) pkt->m_rxSequence,(int) pkt->m_state,(int) pkt->m_result);
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
    if(!m_coms->SetParam(targetDevice,CPI_ControlState,(uint8_t) CS_BootLoader)) {
      m_log->error("Failed to change into boot-loader.");
      return false;
    }

    // Wait for change to happen.

    m_log->info("Setting up connection for device {} ",targetDevice);
    SendBootLoaderReset(targetDevice,true);

    // Wait for ack.
    WaitForResult(0,"reset");

    m_log->info("Starting erase. ");

    uint8_t seqNo = -1;

    SendBootLoaderErase(targetDevice,++seqNo,targetAddress);

    WaitForResult(seqNo,"erase");

    m_log->info("Erase completed ok");


    SendBootLoaderBeginWrite(targetDevice,++seqNo,targetAddress,blockSize);

    WaitForResult(seqNo,"write");

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
        SendBootLoaderData(targetDevice,++seqNo,&blockBuffer[at],len);
        m_log->info("Seq {}, Sent {} data bytes.",(int) seqNo,len);
        at = nextAt;
      }
      // Wait for ack.
      m_log->info("Waiting for data ack. seq {} ",seqNo);
      if(!WaitForResult(seqNo,"data")) {
        m_log->error("Failed to get data ack..");
        break;
      }
    }

#if 0
    // Restart into normal mode
    if(!m_coms->SetParam(targetDevice,CPI_ControlState,(uint8_t) CS_StartUp)) {
      m_log->error("Failed to restart the controller.");
    }
#endif

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
