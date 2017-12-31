
#include <sstream>

#include <thread>
#include <vector>
#include <functional>
#include <assert.h>
#include <mutex>

#include <stdio.h>
#include <string.h>

#include <fstream>
#include <iostream>
//#include <fcntl.h>
//#include <unistd.h>
//#include <sys/termios.h>
#include "dogbot/Coms.hh"
#include "dogbot/DogBotAPI.hh"

namespace DogBotN
{

  ComsC::ComsC(std::shared_ptr<spdlog::logger> &log)
   : m_log(log)
  {
  }

  //! default
  ComsC::ComsC()
  {}

  // Disconnects and closes file descriptors
  ComsC::~ComsC()
  {
    Close();
  }

  //! Close connection
  void ComsC::Close()
  {
    m_terminate = true;
  }

  //! Is connection ready ?
  bool ComsC::IsReady() const
  {
    return true; // Born ready!
  }

  //! Set the logger to use
  void ComsC::SetLogger(const std::shared_ptr<spdlog::logger> &log)
  {
    m_log = log;
  }

  bool ComsC::Open(const std::string &portAddr)
  {
    return false;
  }

  //! Set handler for all packets, this is called as well as any specific handlers that have been installed.
  //! Only one can be set at any time.
  int ComsC::SetGenericHandler(const std::function<void (uint8_t *data,int len)> &handler)
  {
    std::lock_guard<std::mutex> lock(m_accessPacketHandler);
    for(int i = 0;i < m_genericHandler.size();i++) {
      if(!m_genericHandler[i]) {
        m_genericHandler[i] = handler;
        return i;
      }
    }
    int ret = m_genericHandler.size();
    m_genericHandler.push_back(handler);
    return ret;
  }


  //! Remove generic handler
  void ComsC::RemoveGenericHandler(int id)
  {
    if(id < 0)
      return ;
    std::lock_guard<std::mutex> lock(m_accessPacketHandler);
    assert(id < m_genericHandler.size());
    if(id >= m_genericHandler.size())
      return ;
    m_genericHandler[id] = std::function<void (uint8_t *data,int len)>();
  }



  ComsCallbackHandleC ComsC::SetHandler(ComsPacketTypeT packetId,const std::function<void (uint8_t *data,int )> &handler)
  {
    std::lock_guard<std::mutex> lock(m_accessPacketHandler);
    assert((int) packetId < 256);

    while(m_packetHandler.size() <= (int) packetId) {
      m_packetHandler.push_back(std::vector<std::function<void (uint8_t *data,int )> >());
    }
    std::vector<std::function<void (uint8_t *data,int )> > &list = m_packetHandler[(int) packetId];
    for(int i = 0;i < list.size();i++) {
      if(!list[i]) { // Found free slot.
        list[i] = handler;
        return ComsCallbackHandleC(packetId,i);
      }
    }
    int id = list.size();
    list.push_back(handler);
    return ComsCallbackHandleC(packetId,id);
  }

  //! Delete given handler
  void ComsC::DeleteHandler(const ComsCallbackHandleC &handle)
  {
    ComsPacketTypeT packetType = handle.PacketType();
    int id = handle.Id();
    assert(id >= 0);
    assert((unsigned) packetType < m_packetHandler.size());
    assert(id < m_packetHandler[(int) packetType].size());
    m_packetHandler[(int) packetType][id] = std::function<void (uint8_t *data,int )>();
  }



  //! Process received packet.
  void ComsC::ProcessPacket(uint8_t *packetData,int packetLen)
  {
    std::string dataStr;
#if 0
    for(int i = 0;i < packetLen;i++) {
      dataStr += std::to_string(packetData[i]) + " ";
    }
    m_log->debug("Got packet [%d] %s ",packetLen,dataStr.c_str());
#endif

    // Do some handling
    {
      std::lock_guard<std::mutex> lock(m_accessPacketHandler);
      for(auto &func : m_genericHandler) {
        if(func) func(packetData,packetLen);
      }
    }

    // data[0] //
    unsigned packetId = packetData[0];
    switch((ComsPacketTypeT) packetId)
    {
      case CPT_Ping: {
        // Who should be responsible to replying to this ?
#if 0
        uint8_t data[64];
        int at = 0;
        data[at++] = CPT_Pong;
        SendPacket(data,at);
#endif
      } break;
      case CPT_Sync:
        m_log->debug("Got sync. ");
        break;
      default: {
        {
          std::lock_guard<std::mutex> lock(m_accessPacketHandler);
          if(packetId < m_packetHandler.size()) {
            bool hasHandler = false;
            for(auto a : m_packetHandler[packetId]) {
              if(a) {
                hasHandler =  true;
                a(packetData,packetLen);
              }
            }
            if(hasHandler)
              return ;
          }
        }
        // Fall back to the default handlers.
        switch(packetId) {
          case CPT_Pong: {
            if(packetLen != sizeof(struct PacketPingPongC)) {
              m_log->error("Unexpected Pong packet length {}, expected {} ",packetLen,sizeof(struct PacketPingPongC));
              return;
            }

            const PacketPingPongC *pkt = (const PacketPingPongC *) packetData;

            m_log->debug("Got pong from %d. ",(int) pkt->m_deviceId);
          } break;
          case CPT_Error: {
            if(packetLen != sizeof(struct PacketErrorC)) {
              m_log->error("Unexpected Error packet length {}, expected {} ",packetLen,sizeof(struct PacketErrorC));
              return;
            }
            const PacketErrorC *pkt = (const PacketErrorC *) packetData;

            m_log->error("Received packet error code from device {} : {} ({}) Packet:{} ({}) Arg:{} ",
                         (int) pkt->m_deviceId,
                         DogBotN::ComsErrorTypeToString((enum ComsErrorTypeT)pkt->m_errorCode),
                         (int) pkt->m_errorCode,
                         DogBotN::ComsPacketTypeToString((enum ComsPacketTypeT) pkt->m_causeType),(int) pkt->m_causeType,
                         (int) pkt->m_errorData);
          } break;
          default:
            m_log->debug("Don't know how to handle packet {} (Of {}) ",packetId,(int) m_packetHandler.size());
        }
      } break;
    }
  }

  //! Send packet
  void ComsC::SendPacket(const uint8_t *buff,int len)
  {
    assert(0); // Abstract method
  }


  //! Set a parameter
  void ComsC::SendSetParam(int deviceId,ComsParameterIndexT param,uint8_t value)
  {
    PacketParam8ByteC msg;
    msg.m_header.m_packetType = CPT_SetParam;
    msg.m_header.m_deviceId = deviceId;
    msg.m_header.m_index = (uint16_t) param;
    msg.m_data.uint8[0] = value;
    SendPacket((uint8_t*) &msg,sizeof(msg.m_header)+1);
  }

  //! Set a parameter
  void ComsC::SendSetParam(int deviceId,ComsParameterIndexT param,int value)
  {
    PacketParam8ByteC msg;
    msg.m_header.m_packetType = CPT_SetParam;
    msg.m_header.m_deviceId = deviceId;
    msg.m_header.m_index = (uint16_t) param;
    msg.m_data.uint8[0] = value;
    SendPacket((uint8_t*) &msg,sizeof(msg.m_header)+1);
  }



  //! Set a parameter
  void ComsC::SendSetParam(int deviceId,ComsParameterIndexT param,float value)
  {
    PacketParam8ByteC msg;
    msg.m_header.m_packetType = CPT_SetParam;
    msg.m_header.m_deviceId = deviceId;
    msg.m_header.m_index = (uint16_t) param;
    msg.m_data.float32[0] = value;
    SendPacket((uint8_t*) &msg,sizeof(msg.m_header)+4);
  }


  //! Set a parameter
  void ComsC::SendSetParam(int deviceId,ComsParameterIndexT param,BufferTypeT &buff,int len)
  {
    PacketParam8ByteC msg;
    msg.m_header.m_packetType = CPT_SetParam;
    msg.m_header.m_deviceId = deviceId;
    msg.m_header.m_index = (uint16_t) param;
    assert(len >= 0 && len < sizeof(BufferTypeT));
    memcpy(&msg.m_data,&buff,len);
    SendPacket((uint8_t*) &msg,sizeof(msg.m_header)+len);
  }

  //! Query a parameter
  void ComsC::SendQueryParam(int deviceId,ComsParameterIndexT param)
  {
    PacketReadParamC msg;
    msg.m_packetType = CPT_ReadParam;
    msg.m_deviceId = deviceId;
    msg.m_index = (uint16_t) param;
    SendPacket((uint8_t*) &msg,sizeof(msg));
  }


  //! Send query devices message
  void ComsC::SendQueryDevices()
  {
    uint8_t data[2];
    data[0] = CPT_QueryDevices;
    SendPacket(data,1);
  }

  //! Send a sync message
  void ComsC::SendSync()
  {
    uint8_t data[2];
    data[0] = CPT_Sync;
    SendPacket(data,1);
  }

  //! Set a device id
  void ComsC::SendSetDeviceId(uint8_t deviceId,uint32_t uid0,uint32_t uid1)
  {
    PacketDeviceIdC pkg;
    pkg.m_packetType = CPT_SetDeviceId;
    pkg.m_deviceId = deviceId;
    pkg.m_uid[0] = uid0;
    pkg.m_uid[1] = uid1;
    SendPacket((uint8_t*) &pkg,sizeof(pkg));
  }

  //! Send a move command
  void ComsC::SendMoveWithEffort(int deviceId,float pos,float effort,enum PositionReferenceT posRef)
  {
    struct PacketServoC servoPkt;
    servoPkt.m_packetType = CPT_Servo;
    servoPkt.m_deviceId = deviceId;
    servoPkt.m_position = pos * 65535.0 / (4.0 * M_PI);
    if(effort < 0) effort = 0;
    if(effort > 10.0) effort = 10.0;
    servoPkt.m_torqueLimit = effort * 65535.0 / (10.0);
    servoPkt.m_mode = ((int) posRef) | (((int) CM_Position) << 2);

    SendPacket((uint8_t *)&servoPkt,sizeof servoPkt);
  }

  //! Send velocity command with an effort limit.
  void ComsC::SendVelocityWithEffort(int deviceId,float velocity,float effort)
  {

    struct PacketServoC servoPkt;
    servoPkt.m_packetType = CPT_Servo;
    servoPkt.m_deviceId = deviceId;
    servoPkt.m_position = velocity * 32767.0 / (4.0 * M_PI);
    if(effort < 0) effort = 0;
    if(effort > 10.0) effort = 10.0;
    servoPkt.m_torqueLimit = effort * 65535.0 / (10.0);
    servoPkt.m_mode = (((int) CM_Velocity) << 2);

    SendPacket((uint8_t *)&servoPkt,sizeof servoPkt);
  }

  //! Send torque to apply
  void ComsC::SendTorque(int deviceId,float torque)
  {
    struct PacketServoC servoPkt;
    servoPkt.m_packetType = CPT_Servo;
    servoPkt.m_deviceId = deviceId;
    if(torque > 10) torque = 10;
    if(torque < -10) torque = -10;
    servoPkt.m_position = torque * 32767.0 / (10.0);
    servoPkt.m_mode = (((int) CM_Torque) << 2);

    SendPacket((uint8_t *)&servoPkt,sizeof servoPkt);

  }


  //! Send a calibration zero
  void ComsC::SendCalZero(int deviceId)
  {
    struct PacketCalZeroC pkt;
    pkt.m_packetType = CPT_CalZero;
    pkt.m_deviceId = deviceId;
    SendPacket((uint8_t *)&pkt,sizeof pkt);
  }

  //! Send a move command
  void ComsC::SendPing(int deviceId)
  {
    PacketPingPongC pkt;
    pkt.m_packetType = CPT_Ping;
    pkt.m_deviceId = deviceId;
    SendPacket((uint8_t *)&pkt,sizeof(pkt));
  }


  //! Send an enable bridge mode
  void ComsC::SendEnableBridge(bool enable)
  {
    PacketBridgeModeC pkt;
    pkt.m_packetType = CPT_BridgeMode;
    pkt.m_enable = enable;
    SendPacket((uint8_t *)&pkt,sizeof(pkt));
  }

  //! Send an emergency stop
  void ComsC::SendEmergencyStop()
  {
    uint8_t packetType = CPT_EmergencyStop;
    SendPacket((uint8_t *)&packetType,sizeof(packetType));
  }

  // ------------------------------------------------------------------------------

  //! Construct from 'coms' object we're registering callbacks from.
  ComsRegisteredCallbackSetC::ComsRegisteredCallbackSetC(std::shared_ptr<ComsC> &coms)
   : m_coms(coms)
  {
    assert(coms);
  }

  //! Destructor
  ComsRegisteredCallbackSetC::~ComsRegisteredCallbackSetC()
  {
    PopAll();
  }

  //! /param cb A callback to be added to the list.
  void ComsRegisteredCallbackSetC::Push(const ComsCallbackHandleC &cb)
  {
    m_callbacks.push_back(cb);
  }

  //! De-register all currently registered callbacks.
  void ComsRegisteredCallbackSetC::PopAll()
  {
    while(!m_callbacks.empty()) {
      m_coms->DeleteHandler(m_callbacks.back());
      m_callbacks.pop_back();
    }
  }

  void ComsRegisteredCallbackSetC::SetHandler(ComsPacketTypeT packetType,const std::function<void (uint8_t *data,int len)> &handler)
  {
    Push(m_coms->SetHandler(packetType,handler));
  }

}
