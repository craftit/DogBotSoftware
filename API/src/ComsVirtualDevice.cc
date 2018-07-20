
#include "dogbot/ComsVirtualDevice.hh"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/Strings.hh"

#include <iostream>

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif

namespace DogBotN
{
  //! default
  ComsVirtualDeviceC::ComsVirtualDeviceC()
  {}

  // Disconnects and closes file descriptors
  ComsVirtualDeviceC::~ComsVirtualDeviceC()
  {
  }

  //! Close connection
  void ComsVirtualDeviceC::Close()
  {
  }

  //! Is connection ready ?
  bool ComsVirtualDeviceC::IsReady() const
  {
    return true;
  }

  bool ComsVirtualDeviceC::Open(const std::string &portAddr)
  {
    return true;
  }

  //! Send packet
  void ComsVirtualDeviceC::SendPacketWire(const uint8_t *buff,int len)
  {
    VirtProcessPacket(buff,len);
  }

  //! Send packet out from device
  bool ComsVirtualDeviceC::VirtSendPacket(const uint8_t *data,int len)
  {
    ProcessPacket(data,len);
    return true;
  }

  bool ComsVirtualDeviceC::ChangeControlState(enum ControlStateT newState,enum StateChangeSourceT changeSource)
  {
    m_controlState = newState;
    return true;
  }

  //! Set the device type to use
  void ComsVirtualDeviceC::SetDeviceType(enum DeviceTypeT deviceType)
  {
    m_deviceType = deviceType;
    m_nodeUId1 = (m_nodeUId1 & 0x0fffffffu) | ((uint32_t) m_deviceType) << 28;
    //m_nodeUId2;
    ONDEBUG(m_log->info("Setting up device type '{}' with id 0x{:8x} 0x{:8x} ",ComsDeviceTypeToString(m_deviceType),m_nodeUId1,m_nodeUId2));
  }


  void ComsVirtualDeviceC::VirtSendError(
      uint8_t deviceId,
      ComsErrorTypeT code,
      uint8_t originalPacketType,
      uint8_t data
      )
  {
    PacketErrorC pkt;
    pkt.m_packetType = CPT_Error;
    pkt.m_deviceId = deviceId;
    pkt.m_errorCode = code;
    pkt.m_causeType = originalPacketType;
    pkt.m_errorData = data;
    VirtSendPacket((uint8_t *)&pkt,sizeof(pkt));
  }



  bool ComsVirtualDeviceC::SetParam(enum ComsParameterIndexT index,union BufferTypeT *data,int len)
  {
    switch(index )
    {
      case CPI_FirmwareVersion:
        return false; // Can't set this
      case CPI_CANBridgeMode:
        if(len != 1)
          return false;
        //g_canBridgeMode = data->uint8[0] > 0;
        break;
      case CPI_BoardUID:
      case CPI_VSUPPLY:
      case CPI_5VRail:
      case CPI_DriveTemp:
      case CPI_DeviceType:
      case CPI_TIM1_SR:
        break;
      case CPI_ControlState: {
        if(len != 1)
          return false;
        enum ControlStateT newState = (enum ControlStateT) data->uint8[0];
        if(!ChangeControlState(newState,SCS_UserRequest))
          return false;
      } break;
      case CPI_FaultCode:
        // Just clear it.
        m_lastFaultCode = FC_Ok;
        break;
      case CPI_FaultState:
        // Just clear it.
        m_faultState = 0;
        break;
      case CPI_Indicator:
        if(len != 1)
          return false;
        m_indicatorState = data->uint8[0] > 0;
        break;
      case CPI_FINAL:
        return false;
      default:
        return false;
    }

    VirtSendParamUpdate(index);

    return true;
  }


  // Up to 8 bytes of data may be returned by this function.

  bool ComsVirtualDeviceC::ReadParam(enum ComsParameterIndexT index,int *len,union BufferTypeT *data)
  {
    switch(index)
    {
      case CPI_DeviceType:
        *len = 1;
        data->uint8[0] = (int) DeviceType();
        break;
      case CPI_FirmwareVersion:
        *len = 1;
        data->uint8[0] = 1;
        break;
      case CPI_CANBridgeMode:
        *len = 1;
        data->uint8[0] = 0;
        break;
      case CPI_BoardUID:
        *len = 8;
        data->uint32[0] = m_nodeUId1;
        data->uint32[1] = m_nodeUId2;
        break;
      case CPI_TIM1_SR: {
        *len = 2;
        data->uint16[0] = 0;
      } break;
      case CPI_VSUPPLY: {
        unsigned val = 0;//g_vbus_voltage * 1000.0f;
        *len = 2;
        data->uint16[0] = val;
      } break;
      case CPI_5VRail: {
        *len = 4;
        data->float32[0] = 0;
      } break;
      case CPI_ControlState:
        *len = 1;
        data->uint8[0] = (int) m_controlState;
        break;
      case CPI_FaultCode:
        *len = 1;
        data->uint8[0] = m_lastFaultCode;
        break;
      case CPI_Indicator:
        *len = 1;
        data->uint8[0] = (int) m_indicatorState;
        break;
      case CPI_FINAL:
      default: return false;
    }
    return true;
  }


  bool ComsVirtualDeviceC::ReadParamAndReply(enum ComsParameterIndexT paramIndex)
  {
    struct PacketParam8ByteC reply;
    reply.m_header.m_packetType = CPT_ReportParam;
    reply.m_header.m_deviceId = m_deviceId;
    reply.m_header.m_index = paramIndex;
    int len = 0;
    if(!ReadParam(paramIndex,&len,&reply.m_data))
      return false;

    VirtSendPacket((uint8_t *)&reply,sizeof(reply.m_header) + len);
    return true;
  }

  void ComsVirtualDeviceC::VirtSendParamUpdate(enum ComsParameterIndexT paramIndex)
  {
    if(!ReadParamAndReply(paramIndex)) {
      VirtSendError(m_deviceId,CET_ParameterOutOfRange,CPT_ReadParam,(uint8_t) paramIndex);
    }
  }


  //! Process received packet.
  void ComsVirtualDeviceC::VirtProcessPacket(const uint8_t *data,int len)
  {
    enum ComsPacketTypeT pktType = (enum ComsPacketTypeT) data[0];
    ONDEBUG(m_log->debug("VirtualDevice. Processing packet '{}' for {}  ",DogBotN::ComsPacketTypeToString(pktType),(int) data[1]));
    switch(pktType)
    {
      case CPT_NoOp: break;
      case CPT_Ping: {
        if(len != sizeof(struct PacketPingPongC)) {
          VirtSendError(m_deviceId,CET_UnexpectedPacketSize,(uint8_t) pktType,len);
          break;
        }
        struct PacketPingPongC *ping = (struct PacketPingPongC *) data;
        struct PacketPingPongC pong;
        pong.m_deviceId = m_deviceId;
        pong.m_packetType = CPT_Pong;
        pong.m_payload = ping->m_payload;

        VirtSendPacket((uint8_t *)&pong,sizeof(pong));
        break;
      }
      case CPT_Sync:        break; // Just drop it.
      case CPT_Pong:        break; // Just drop it.
      case CPT_AnnounceId:  break; // Just drop it.
      case CPT_BridgeMode:  break; // Just drop it.
      case CPT_SetDeviceId: {
        struct PacketDeviceIdC *pkt = (struct PacketDeviceIdC *) data;
        if(pkt->m_uid[0] == m_nodeUId1 && pkt->m_uid[1] == m_nodeUId2) {
          m_deviceId = pkt->m_deviceId;
        } else
          break; // This SetDeviceId is not for us
      }
      // Fall through and publish our new id.
      // no break
      case CPT_QueryDevices: {
        struct PacketDeviceIdC pkt;
        pkt.m_packetType = CPT_AnnounceId;
        pkt.m_deviceId = m_deviceId;
        pkt.m_uid[0] = m_nodeUId1;
        pkt.m_uid[1] = m_nodeUId2;
        VirtSendPacket((uint8_t *)&pkt,sizeof(pkt));
      } break;
      case CPT_ReadParam: {
        if(len != sizeof(struct PacketReadParamC)) {
          VirtSendError(m_deviceId,CET_UnexpectedPacketSize,CPT_ReadParam,len);
          break;
        }
        struct PacketReadParamC *psp = (struct PacketReadParamC *) data;
        if(psp->m_deviceId == m_deviceId || psp->m_deviceId == 0) {
          if(!ReadParamAndReply((enum ComsParameterIndexT) psp->m_index)) {
            VirtSendError(m_deviceId,CET_ParameterOutOfRange,CPT_ReadParam,psp->m_index);
          }
        }
      } break;
      case CPT_SetParam: {
        struct PacketParam8ByteC *psp = (struct PacketParam8ByteC *) data;
        if(len < (int) sizeof(psp->m_header)) {
          VirtSendError(m_deviceId,CET_UnexpectedPacketSize,CPT_SetParam,len);
          break;
        }
        int dataLen = len-sizeof(psp->m_header);
        if(psp->m_header.m_deviceId == m_deviceId || psp->m_header.m_deviceId == 0) {
          if(!SetParam((enum ComsParameterIndexT) psp->m_header.m_index,&psp->m_data,dataLen)) {
            VirtSendError(m_deviceId,CET_ParameterOutOfRange,CPT_SetParam,psp->m_header.m_index);
          }
        }
      } break;
      default:
        VirtSendError(m_deviceId,CET_UnknownPacketType,data[0],len);
        break;
    }
  }


  void ComsVirtualDeviceC::VirtSendSync()
  {
    uint8_t buff[6];
    uint8_t at = 0;
    buff[at++] = CPT_Sync;
    VirtSendPacket(buff,at);
  }

  void ComsVirtualDeviceC::VirtSendMessage(const char *msg)
  {
    uint8_t buff[64];
    const char *txt = msg;
    uint8_t at = 0;
    buff[at++] = CPT_Message;
    buff[at++] = m_deviceId;
    while(*txt != 0 && at < 64) {
      buff[at++] = *(txt++);
    }
    VirtSendPacket(buff,at);
  }

}
