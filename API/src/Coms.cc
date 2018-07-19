
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
#include "dogbot/Strings.hh"

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif

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


  //! Process received packet.
  void ComsC::ProcessPacket(const uint8_t *packetData,int packetLen)
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
      for(auto &func : m_genericHandler.Calls()) {
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
          bool hasHandler = false;
          for(auto a : m_packetHandler[packetId].Calls()) {
            if(a) {
              hasHandler =  true;
              a(packetData,packetLen);
            }
          }
          if(hasHandler)
            return ;
        }
        // Fall back to the default handlers.
        switch(packetId) {
          case CPT_Pong: {
            if(packetLen != sizeof(struct PacketPingPongC)) {
              m_log->error("Unexpected Pong packet length {}, expected {} ",packetLen,sizeof(struct PacketPingPongC));
              return;
            }

            const PacketPingPongC *pkt = (const PacketPingPongC *) packetData;

            m_log->debug("Got pong from {}. ",(int) pkt->m_deviceId);
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
    // I'm not totally happy with the delay this will add to sending the packet,
    // but it isn't clear what to do about that without adding lots of complexity.
    for(auto &a : m_commandCallback.Calls()) {
      if(a) a(buff,len);
    }
    SendPacketWire(buff,len);
  }

  //! Send packet
  void ComsC::SendPacketWire(const uint8_t *buff,int len)
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
  void ComsC::SendSetParam(int deviceId,ComsParameterIndexT param,double value)
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

  //! Send set plaform activity
  void ComsC::SendSetPlaformActivity(int deviceId,uint32_t key,enum PlatformActivityT pa)
  {
    PacketParam8ByteC msg;
    msg.m_header.m_packetType = CPT_SetParam;
    msg.m_header.m_deviceId = deviceId;
    msg.m_header.m_index = (uint16_t) CPI_PlatformActivity;
    msg.m_data.uint32[0] = key;
    msg.m_data.uint8[4] = (uint8_t) pa;
    SendPacket((uint8_t*) &msg,sizeof(msg.m_header)+5);
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
    ONDEBUG(m_log->info("Sending device query. "));
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
  void ComsC::SendMoveWithEffortLimit(
      int deviceId,
      float pos,
      float effort,
      enum PositionReferenceT posRef,
      uint8_t timestamp)
  {
    struct PacketServoC servoPkt;
    servoPkt.m_packetType = CPT_Servo;
    servoPkt.m_deviceId = deviceId;
    servoPkt.m_timestamp = timestamp;
    servoPkt.m_demand = pos * DOGBOT_PACKETSERVO_FLOATSCALE / DOGBOT_SERVOREPORT_POSITIONRANGE;
    if(effort < 0.0) effort = 0.0;
    if(effort > 1.0) effort = 1.0;
    servoPkt.m_torque = effort * DOGBOT_PACKETSERVO_FLOATSCALE;
    servoPkt.m_mode = ((int) posRef) | (((int) CM_Position) << 2);

    SendPacket((uint8_t *)&servoPkt,sizeof servoPkt);
  }

  //! Send a move command with a current limit.
  void ComsC::SendMoveWithEffort(
      int deviceId,
      float pos,
      float effort,
      enum PositionReferenceT posRef,
      uint8_t timestamp
      )
  {
    struct PacketServoC servoPkt;
    servoPkt.m_packetType = CPT_Servo;
    servoPkt.m_deviceId = deviceId;
    servoPkt.m_timestamp = timestamp;
    servoPkt.m_demand = pos * DOGBOT_PACKETSERVO_FLOATSCALE / DOGBOT_SERVOREPORT_POSITIONRANGE;
    if(effort < -1.0) effort = -1.0;
    if(effort > 1.0) effort = 1.0;
    servoPkt.m_torque = effort * DOGBOT_PACKETSERVO_FLOATSCALE;
    servoPkt.m_mode = ((int) posRef) | (((int) CM_Position) << 2) | DOGBOT_PACKETSERVOMODE_DEMANDTORQUE;

    SendPacket((uint8_t *)&servoPkt,sizeof servoPkt);
  }

  //! Send velocity command with an effort limit.
  void ComsC::SendVelocityWithEffort(int deviceId,float velocity,float effort)
  {

    struct PacketServoC servoPkt;
    servoPkt.m_packetType = CPT_Servo;
    servoPkt.m_deviceId = deviceId;
    servoPkt.m_timestamp = 0;
    servoPkt.m_demand = velocity * DOGBOT_PACKETSERVO_FLOATSCALE / DOGBOT_SERVOREPORT_POSITIONRANGE;
    if(effort < 0) effort = 0;
    if(effort > 1.0) effort = 1.0;
    servoPkt.m_torque = effort * DOGBOT_PACKETSERVO_FLOATSCALE;
    servoPkt.m_mode = (((int) CM_Velocity) << 2);

    SendPacket((uint8_t *)&servoPkt,sizeof servoPkt);
  }

  //! Send torque to apply
  void ComsC::SendTorque(int deviceId,float torque)
  {
    struct PacketServoC servoPkt;
    servoPkt.m_packetType = CPT_Servo;
    servoPkt.m_timestamp = 0;
    servoPkt.m_deviceId = deviceId;
    if(torque > 1) torque = 1;
    if(torque < -1) torque = -1;
    servoPkt.m_demand = torque * DOGBOT_PACKETSERVO_FLOATSCALE;
    servoPkt.m_mode = (((int) CM_Torque) << 2);
    servoPkt.m_torque = DOGBOT_PACKETSERVO_FLOATSCALE;

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

  //! Send command to store configuration to eeprom.
  void ComsC::SendStoreConfig(int deviceId)
  {
    struct PacketStoredConfigC pkt;
    pkt.m_packetType = CPT_SaveSetup;
    pkt.m_deviceId = deviceId;
    SendPacket((uint8_t *)&pkt,sizeof pkt);
  }

  //! Send command to load configuration from eeprom.
  void ComsC::SendLoadConfig(int deviceId)
  {
    struct PacketStoredConfigC pkt;
    pkt.m_packetType = CPT_LoadSetup;
    pkt.m_deviceId = deviceId;
    SendPacket((uint8_t *)&pkt,sizeof pkt);
  }


  //! Send a move command
  void ComsC::SendPing(int deviceId,int payload)
  {
    PacketPingPongC pkt;
    pkt.m_packetType = CPT_Ping;
    pkt.m_deviceId = deviceId;
    pkt.m_payload = payload;
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

}
