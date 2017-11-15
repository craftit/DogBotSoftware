#ifndef DOGBOG_COMS_HEADER
#define DOGBOG_COMS_HEADER 1

#include <sstream>

#include <thread>
#include <vector>
#include <functional>
#include <assert.h>
#include <mutex>
#include <string.h>

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

    //! Send a move command with an effort limit.
    void SendMoveWithEffort(int deviceId,float pos,float effort,enum PositionReferenceT posRef);

    //! Send velocity command with an effort limit.
    void SendVelocityWithEffort(int deviceId,float velocity,float effort);

    //! Send torque to apply
    void SendTorque(int deviceId,float torque);

    //! Set a parameter synchronous, method will not return until parameter
    //! has been confirmed set or an error occurred.
    template<typename ParamT>
    bool SetParam(int deviceId,ComsParameterIndexT param,ParamT value)
    {
      using Ms = std::chrono::milliseconds;
      bool ret = false;

      PacketParam8ByteC msg;
      msg.m_header.m_packetType = CPT_SetParam;
      msg.m_header.m_deviceId = deviceId;
      msg.m_header.m_index = (uint16_t) param;
      memcpy(msg.m_data.uint8, &value,sizeof(value));

      std::timed_mutex done;
      done.lock();

      int handler = SetHandler(CPT_ReportParam,[this,deviceId,param,&msg,&ret,&done](uint8_t *data,int len) mutable {
        if(len < sizeof(PacketParamHeaderC)) {
          m_log->error("Short ReportParam packet received. {} Bytes ",len);
          return ;
        }
        PacketParam8ByteC *pkt = reinterpret_cast<PacketParam8ByteC *>(data);
        // Is it from the device we're interested in
        if(pkt->m_header.m_deviceId != deviceId)
          return ;
        if(pkt->m_header.m_index != param)
          return ;
        if(len != sizeof(msg.m_header)+sizeof(value)) {
          m_log->error("Unexpected reply size {}, when {} bytes were sent for parameter {} ",len,sizeof(msg.m_header)+sizeof(value),(int) param);
          done.unlock();
          return ;
        }
        if(memcmp(msg.m_data.uint8,pkt->m_data.uint8,sizeof(value)) == 0) {
          ret = true;
          done.unlock();
        }
      });
      assert(sizeof(value) <= 7);
      if(sizeof(value) > 7) {
        std::cerr << "Parameter " << (int) param << " too large. " << std::endl;
        return false;
      }
      for(int i = 0;i < 4 && ret;i++) {
        // Send data.
        SendPacket((uint8_t*) &msg,sizeof(msg.m_header)+sizeof(value));
        if(done.try_lock_for(Ms(100))) {
          break;
        }
      }

      //! Delete given handler
      DeleteHandler(CPT_ReportParam,handler);

      return ret;
    }



    //! Set a parameter
    void SendSetParam(int deviceId,ComsParameterIndexT param,uint8_t value);

    //! Set a parameter
    void SendSetParam(int deviceId,ComsParameterIndexT param,int value);

    //! Set a parameter
    void SendSetParam(int deviceId,ComsParameterIndexT param,float value);

    //! Set a parameter
    void SendSetParam(int deviceId,ComsParameterIndexT param,BufferTypeT &buff,int len);

    //! Query a parameter
    void SendQueryParam(int deviceId,ComsParameterIndexT param);

    //! Send query devices message
    void SendQueryDevices();

    //! Set a device id
    void SendSetDeviceId(uint8_t deviceId,uint32_t uid0,uint32_t uid1);

    //! Send a sync message
    void SendSync();

    //! Send a ping
    void SendPing(int deviceId);

    //! Send a calibration zero
    void SendCalZero(int deviceId);

    //! Set the handler for a particular type of packet.
    //! Returns the id of the handler or -1 if failed.
    int SetHandler(ComsPacketTypeT packetType,const std::function<void (uint8_t *data,int )> &handler);

    //! Delete given handler
    void DeleteHandler(ComsPacketTypeT packetType,int id);

    //! Convert a report value to an angle in radians
    static float PositionReport2Angle(int16_t val)
    { return val * M_PI * 4.0/ 65535.0; }

    //! Convert a report value to a torque
    static float TorqueReport2Value(int16_t val)
    { return val * 10.0/ 65535.0; }


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
