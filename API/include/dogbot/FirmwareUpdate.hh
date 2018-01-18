#ifndef DOGBOT_FIRMWARE_UPDATE_HEADER
#define DOGBOT_FIRMWARE_UPDATE_HEADER 1

#include "dogbot/Coms.hh"

namespace DogBotN {

  //! State machine for handling firmware update for a set of devices.

  class FirmwareUpdateC
  {
  public:
    //! Construct from coms object
    FirmwareUpdateC(const std::shared_ptr<ComsC> &coms);

    //! Start update
    bool DoUpdate(int deviceId,const std::string &filename);

  protected:
    //! Connect
    void Init();

    //! Send a boot-loader reset
    void SendBootLoaderReset(uint8_t deviceId,bool enable);

    //! Send a boot-loader begin write
    void SendBootLoaderBeginWrite(uint8_t deviceId,uint8_t seqNum,uint32_t address,uint16_t len);

    //! Send a boot-loader erase sector
    void SendBootLoaderErase(uint8_t deviceId,uint8_t seqNum,uint32_t address);

    //! Send a boot-loader checksum request
    void SendBootLoaderCheckSum(uint8_t deviceId,uint8_t seqNum,uint32_t address,uint16_t len);

    //! Send a boot-loader begin read
    void SendBootLoaderBeginRead(uint8_t deviceId,uint8_t seqNum,uint32_t address,uint16_t len);

    //! Send a boot-loader begin read
    void SendBootLoaderData(uint8_t deviceId,uint8_t seqNum,uint8_t *data,uint8_t len);

    //! Wait for result.
    bool WaitForResult(uint8_t seqNo,const std::string &op);

    //! Set the handler for a particular type of packet.
    //! Returns the id of the handler or -1 if failed.
    ComsCallbackHandleC SetHandler(ComsPacketTypeT packetType,const std::function<void (uint8_t *data,int len)> &handler);

    std::shared_ptr<ComsC> m_coms;
    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    bool m_flagError = false;
    bool m_gotResult = false;
    std::mutex m_mutexResult;
    std::condition_variable m_condVar;
    struct PacketFlashResultC m_pktResult;
  };

}



#endif
