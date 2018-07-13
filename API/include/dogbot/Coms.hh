#ifndef DOGBOG_COMS_HEADER
#define DOGBOG_COMS_HEADER 1

#include <sstream>

#include <thread>
#include <vector>
#include <functional>
#include <future>
#include <assert.h>
#include <mutex>
#include <string.h>

#include <cstdint>
#include "dogbot/protocol.h"
#include "dogbot/CallbackArray.hh"
#include <spdlog/spdlog.h>

namespace DogBotN {


  //! Low level communication interface

  class ComsC
  {
  public:
    typedef std::function<void (const uint8_t *data,int len)> PacketFuncT;


    ComsC(std::shared_ptr<spdlog::logger> &log);

    //! default
    ComsC();

    //! Destructor
    // Disconnects and closes file descriptors
    virtual ~ComsC();

    //! Access name of channel, this is the same string as used to open the device.
    const std::string &Name() const
    { return m_name; }

    //! Set the logger to use
    virtual void SetLogger(const std::shared_ptr<spdlog::logger> &log);

    //! Open a port.
    virtual bool Open(const std::string &portAddr);

    //! Close connection
    virtual void Close();

    //! Is connection ready ?
    virtual bool IsReady() const;

    //! Process received packet.
    void ProcessPacket(const uint8_t *data,int len);

    //! Send packet
    void SendPacket(const uint8_t *data,int len);


    //! Send an emergency stop
    void SendEmergencyStop();

    //! Send a move command with an effort limit.
    void SendMoveWithEffortLimit(int deviceId,float pos,float effortLimit,enum PositionReferenceT posRef,uint8_t timestamp = 0);

    //! Send a move command with a current limit.
    void SendMoveWithEffort(int deviceId,float pos,float effort,enum PositionReferenceT posRef,uint8_t timestamp = 0);

    //! Send velocity command with a current limit.
    void SendVelocityWithEffort(int deviceId,float velocity,float currentLimit);

    //! Send torque to apply as a motor current.
    void SendTorque(int deviceId,float current);

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

      std::promise<bool> promiseDone;
      std::future<bool> done = promiseDone.get_future();

      auto handler = SetHandler(CPT_ReportParam,[this,deviceId,param,&msg,&ret,&promiseDone](const uint8_t *data,int len) mutable {
        if(len < sizeof(PacketParamHeaderC)) {
          m_log->error("Short ReportParam packet received. {} Bytes ",len);
          return ;
        }
        const PacketParam8ByteC *pkt = reinterpret_cast<const PacketParam8ByteC *>(data);
        // Is it from the device we're interested in
        if(pkt->m_header.m_deviceId != deviceId)
          return ;
        if(pkt->m_header.m_index != param)
          return ;
        if(len != sizeof(msg.m_header)+sizeof(value)) {
          m_log->error("Unexpected reply size {}, when {} bytes were sent for parameter {} ",len,sizeof(msg.m_header)+sizeof(value),(int) param);
          promiseDone.set_value(false);
          return ;
        }
        if(memcmp(msg.m_data.uint8,pkt->m_data.uint8,sizeof(value)) == 0) {
          if(!ret) {
            ret = true;
            promiseDone.set_value(true);
          }
        } else {
          m_log->warn("Unexpected value returned. ");
        }
      });
      assert(sizeof(value) <= 7);
      if(sizeof(value) > 7) {
        //! Delete given handler
        handler.Remove();
        m_log->error("Parameter {} too large.",(int) param);
        return false;
      }
      for(int i = 0;i < 4 && !ret;i++) {
        // Send data.
        SendPacket((uint8_t*) &msg,sizeof(msg.m_header)+sizeof(value));
        if(done.wait_for(Ms(250)) == std::future_status::ready) {
          break;
        }
      }

      //! Delete given handler
      handler.Remove();

      return ret;
    }

    //! Set a parameter
    void SendSetParam(int deviceId,ComsParameterIndexT param,uint8_t value);

    //! Set a parameter
    void SendSetParam(int deviceId,ComsParameterIndexT param,int value);

    //! Set a parameter
    void SendSetParam(int deviceId,ComsParameterIndexT param,float value);

    //! Set a parameter
    void SendSetParam(int deviceId,ComsParameterIndexT param,double value);

    //! Set a parameter
    void SendSetParam(int deviceId,ComsParameterIndexT param,BufferTypeT &buff,int len);

    //! Set set plaform activity
    void SendSetPlaformActivity(int deviceId,uint32_t key,enum PlatformActivityT pa);

    //! Query a parameter
    void SendQueryParam(int deviceId,ComsParameterIndexT param);

    //! Send query devices message
    void SendQueryDevices();

    //! Set a device id
    void SendSetDeviceId(uint8_t deviceId,uint32_t uid0,uint32_t uid1);

    //! Send a sync message
    void SendSync();

    //! Send a ping
    void SendPing(int deviceId,int payload = 0);

    //! Send an enable bridge mode
    void SendEnableBridge(bool enable);

    //! Send a calibration zero
    void SendCalZero(int deviceId);

    //! Send command to store configuration to eeprom.
    void SendStoreConfig(int deviceId);

    //! Send command to load configuration from eeprom.
    void SendLoadConfig(int deviceId);

    //! Set handler for all received packets, this is called as well as any specific handlers that have been installed.
    CallbackHandleC SetGenericHandler(const PacketFuncT &handler)
    { return m_genericHandler.Add(handler); }

    //! Set handler for all received packets, this is called as well as any specific handlers that have been installed.
    CallbackHandleC SetCommandHandler(const PacketFuncT &handler)
    { return m_commandCallback.Add(handler); }

    //! Set the handler for a particular type of packet.
    //! Returns the id of the handler or -1 if failed.
    CallbackHandleC SetHandler(ComsPacketTypeT packetType,const PacketFuncT &handler)
    {
      assert((size_t) packetType < (size_t)CPT_Final);
      return m_packetHandler[packetType].Add(handler);
    }

    //! Add a update callback for motor position
    CallbackHandleC AddCommandCallback(const PacketFuncT &callback)
    { return m_commandCallback.Add(callback); }

    //! Convert a report value to an angle in radians
    static float PositionReport2Angle(int16_t val)
    { return (float) val * DOGBOT_SERVOREPORT_POSITIONRANGE / 32767.0; }

    //! Convert a report value to an angle in radians
    static float VelocityReport2Angle(int16_t val)
    { return (float) val * DOGBOT_SERVOREPORT_VELOCITYRANGE / 32767.0; }

    //! Convert a report value to a torque
    static float TorqueReport2Fraction(int16_t val)
    { return ((float) val)/ 32767.0; }

  protected:
    std::string m_name;

    //! Send packet
    virtual void SendPacketWire(const uint8_t *data,int len);

    volatile bool m_terminate = false;

    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    std::vector<CallbackArrayC<PacketFuncT> > m_packetHandler = std::vector<CallbackArrayC<PacketFuncT> >((size_t) CPT_Final);
    CallbackArrayC<PacketFuncT> m_genericHandler;

    CallbackArrayC<PacketFuncT> m_commandCallback;

  };


}
#endif
