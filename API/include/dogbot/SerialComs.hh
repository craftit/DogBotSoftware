#ifndef DOGBOG_COMS_HEADER
#define DOGBOG_COMS_HEADER 1

#include <sstream>

#include <thread>
#include <vector>
#include <functional>
#include <assert.h>
#include <mutex>

#include "dogbot/protocol.h"
#include <spdlog/spdlog.h>
#include <cstdint>

namespace DogBotN {

  //! Low level serial communication over usb with the driver board

  class SerialComsC
  {
  public:
    SerialComsC(const char *portAddr);

    //! default
    SerialComsC();

    //! Destructor
    // Disconnects and closes file descriptors
    ~SerialComsC();

    //! Set the logger to use
    void SetLogger(std::shared_ptr<spdlog::logger> &log);

    //! Open a port.
    bool Open(const char *portAddr);

    //! Close connection
    void Close();

    //! Is connection ready ?
    bool IsReady() const;

    //! Accept a byte
    void AcceptByte(uint8_t sendByte);

    //! Process received packet.
    void ProcessPacket();

    //! Send packet
    void SendPacket(const uint8_t *data,int len);

    //! Send a move command with an effort level.
    void SendMoveWithEffort(int deviceId,float pos,float effort,enum PositionReferenceT posRef);

    //! Set a parameter
    void SendSetParam(int deviceId,ComsParameterIndexT param,uint8_t value);

    //! Set a parameter
    void SendSetParam(int deviceId,ComsParameterIndexT param,BufferTypeT &buff,int len);

    //! Query a parameter
    void SendQueryParam(int deviceId,ComsParameterIndexT param);

    //! Send query devices message
    void SendQueryDevices();

    //! Set a device id
    void SendSetDeviceId(uint8_t deviceId,uint32_t uid0,uint32_t uid1);

    //! Send a ping
    void SendPing(int deviceId);

    //! Set the handler for a particular type of packet.
    //! Returns the id of the handler or -1 if failed.
    int SetHandler(ComsPacketTypeT packetType,const std::function<void (uint8_t *data,int )> &handler);

    //! Delete given handler
    void DeleteHandler(ComsPacketTypeT packetType,int id);

    volatile bool m_terminate = false;
    int m_state = 0;
    int m_checkSum = 0;
    int m_packetLen = 0;
    uint8_t m_data[64];
    int m_at = 0;
    const uint8_t m_charSTX = 0x02;
    const uint8_t m_charETX = 0x03;

    int m_fd = -1;

    // Packet structure.
    // x    STX
    // x    Len - Of data excluding STX,ETX and Checksum.
    // 0    Address
    // 1    Type
    // 2..n data.
    // n    2-CRC
    // n    ETX.

    bool RunRecieve();

    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    std::thread m_threadRecieve;

    std::mutex m_accessPacketHandler;
    std::mutex m_accessTx;
    std::timed_mutex m_mutexExitOk;

    std::vector<std::vector<std::function<void (uint8_t *data,int )> > > m_packetHandler;
  };
}
#endif
