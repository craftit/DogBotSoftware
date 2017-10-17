
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
#include <fcntl.h>

#include <unistd.h>
#include <sys/termios.h>
#include "../include/dogbot/SerialComs.hh"

namespace DogBotN
{

  SerialComsC::SerialComsC(const char *portAddr)
  {
    Open(portAddr);
  }

  //! default
  SerialComsC::SerialComsC()
  {}

  // Disconnects and closes file descriptors
  SerialComsC::~SerialComsC()
  {
    if(m_fd >= 0)
      Close();

  }

  //! Close connection
  void SerialComsC::Close()
  {
    m_log->debug("Close called.");
    m_terminate = true;
    if(m_fd >= 0)
      close(m_fd);
    m_fd = -1;
    if(!m_mutexExitOk.try_lock_for(std::chrono::milliseconds(500))) {
      m_log->error("Failed to shutdown receiver thread.");
    }
    m_threadRecieve.join();
  }

  //! Is connection ready ?
  bool SerialComsC::IsReady() const
  {
    return m_fd >= 0;
  }


  //! Set the logger to use
  void SerialComsC::SetLogger(std::shared_ptr<spdlog::logger> &log)
  {
    m_log = log;
  }

  bool SerialComsC::Open(const char *portAddr)
  {
    m_terminate = false;
    if(m_fd >= 0) {
      m_log->info("Coms already open. ");
      return false;
    }

    m_log->info("Opening: '{}'",portAddr);

    m_fd = open(portAddr,O_RDWR | O_NONBLOCK);
    if(m_fd < 0) {
      m_log->error("Failed to open file {} Error:{}, {} ",portAddr,errno,strerror(errno));
      return false;
    }
    m_log->debug("Port opened ok  '{}'",portAddr);

    {
      termios termios_p;
      if (tcgetattr(m_fd,&termios_p) < 0) {
        m_log->error("Failed to read port parameters. ");
        return false;
      }

      termios_p.c_iflag = IGNBRK;
      termios_p.c_oflag = 0;
      termios_p.c_cflag = CLOCAL | CREAD;
      termios_p.c_lflag = 0;
      termios_p.c_cc[VMIN] = 1;
      termios_p.c_cc[VTIME] = 0;

      cfmakeraw(&termios_p);

      termios_p.c_cflag &= ~(CSIZE);  // Clear number of bits.
      termios_p.c_cflag |= CS8;       // 8 bits
      termios_p.c_cflag &= ~(CSTOPB); // 1 Stop bit

      termios_p.c_cflag &=~(PARENB);  // No parity
      termios_p.c_iflag &= ~(INPCK);

      cfsetispeed(&termios_p, B38400);
      cfsetospeed(&termios_p, B38400);

      if(tcsetattr(m_fd, TCSANOW, &termios_p ) < 0)  {
        m_log->error("Failed to configure serial port ");
        return false;
      }
    }
    if(!m_terminate) {
      if(!m_mutexExitOk.try_lock()) {
        m_log->error("Exit lock already locked, multiple threads attempting to open coms ?");
      } else {
        m_threadRecieve = std::move(std::thread { [this]{ RunRecieve(); } });
      }
    }
    return true;
  }


  int SerialComsC::SetHandler(ComsPacketTypeT packetId,const std::function<void (uint8_t *data,int )> &handler)
  {
    std::lock_guard<std::mutex> lock(m_accessPacketHandler);
    assert(packetId < 256);

    while(m_packetHandler.size() <= (int) packetId) {
      m_packetHandler.push_back(std::vector<std::function<void (uint8_t *data,int )> >());
    }
    std::vector<std::function<void (uint8_t *data,int )> > &list = m_packetHandler[(int) packetId];
    for(int i = 0;i < list.size();i++) {
      if(!list[i]) { // Found free slot.
        list[i] = handler;
        return i;
      }
    }
    int id = list.size();
    list.push_back(handler);
    return id;
  }

  //! Delete given handler
  void SerialComsC::DeleteHandler(ComsPacketTypeT packetType,int id)
  {
    assert(id >= 0);
    assert((unsigned) packetType < m_packetHandler.size());
    assert(id < m_packetHandler[(int) packetType].size());
    m_packetHandler[(int) packetType][id] = std::function<void (uint8_t *data,int )>();
  }


  void SerialComsC::AcceptByte(uint8_t sendByte)
  {
    switch(m_state)
    {
    case 0: // Wait for STX.
      if(sendByte == m_charSTX)
        m_state = 1;
      // Else remain in state 0.
      break;
    case 1: // Packet length.
      m_packetLen = sendByte;
      if(m_packetLen > 64) {
        printf("Packet to long! %d  ",m_packetLen);
        if(sendByte != m_charSTX)
          m_state = 0;
        break;
      }

      m_at = 0;
      m_checkSum = 0x55 + m_packetLen;
      m_state = 2;
      break;
    case 2: // Data
      m_checkSum += sendByte;
      m_data[m_at] = sendByte;
      m_at++;
      if(m_at >= m_packetLen)
        m_state = 3;
      break;
    case 3: { // CRC 1
      uint8_t cs1 = (m_checkSum & 0xff);
      //m_log->debug("Checksum1 : %d %d ",(int)  cs1 , (int) sendByte);
      if(cs1 != sendByte) {
        m_log->warn("Checksum 1 failed. ");
        if(sendByte == m_charSTX)
          m_state = 1;
        else
          m_state = 0;
        break;
      }

      m_state = 4;
    } break;
    case 4: { // CRC 1
      uint8_t cs2 = ((m_checkSum >> 8) & 0xff);
      //m_log->debug("Checksum2 : %d %d ",(int) ((m_checkSum >> 8) & 0xff) , (int) sendByte);
      if(cs2 != sendByte) {
        m_log->warn("Checksum 2 failed. ");
        if(sendByte == m_charSTX)
          m_state = 1;
        else
          m_state = 0;
        break;
      }

      m_state = 5;
    } break;
    case 5: // ETX.
      if(sendByte == m_charETX) {
        ProcessPacket();
      } else {
        m_log->warn("Packet corrupted. Len {} Type {} ",m_packetLen,(int) m_data[0]);
        if(sendByte == m_charSTX) {
          m_state = 1;
          break;
        }
      }
      m_state = 0;
      break;
    }
  }

  //! Process received packet.
  void SerialComsC::ProcessPacket()
  {
    std::string data;
#if 0
    for(int i = 0;i < m_packetLen;i++) {
      data += std::to_string(m_data[i]) + " ";
    }
    m_log->debug("Got packet [%d] %s ",m_packetLen,data.c_str());
#endif

    // m_data[0] //
    int packetId = m_data[0];
    switch(packetId)
    {
      case CPT_Ping: {
        uint8_t data[64];
        int at = 0;
        data[at++] = CPT_Pong;
        SendPacket(data,at);
      } break;
      case CPT_Sync:
        m_log->debug("Got sync. ");
        break;
      default: {
        std::lock_guard<std::mutex> lock(m_accessPacketHandler);
        if(packetId < m_packetHandler.size()) {
          bool hasHandler = false;
          //const std::function<void (uint8_t *data,int )> &handler;
          for(auto a : m_packetHandler[packetId]) {
            if(a) {
              hasHandler =  true;
              a(m_data,m_packetLen);
            }
          }
          if(hasHandler)
            return ;
        }
        // Fall back to the default handlers.
        switch(packetId) {
          case CPT_Pong: {
            if(m_packetLen != sizeof(struct PacketPingPongC)) {
              m_log->error("Unexpected Pong packet length {}, expected {} ",m_packetLen,sizeof(struct PacketPingPongC));
              return;
            }

            const PacketPingPongC *pkt = (const PacketPingPongC *) m_data;

            m_log->debug("Got pong from %d. ",(int) pkt->m_deviceId);
          } break;
          case CPT_Error: {
            if(m_packetLen != sizeof(struct PacketErrorC)) {
              m_log->error("Unexpected Error packet length {}, expected {} ",m_packetLen,sizeof(struct PacketErrorC));
              return;
            }
            const PacketErrorC *pkt = (const PacketErrorC *) m_data;

            m_log->error("Received error code from device {} : {}  Arg:{} ",
                         (int) pkt->m_deviceId,
                         (int) pkt->m_errorCode,
                         (int) pkt->m_errorData);
          } break;
          default:
            m_log->debug("Don't know how to handle packet {} (Of {}) ",packetId,(int) m_packetHandler.size());
        }
      } break;
    }
  }

  bool SerialComsC::RunRecieve()
  {
    assert(m_fd >= 0);
    uint8_t readBuff[1024];
    fd_set localFds;
    fd_set exceptFds;
    FD_ZERO(&localFds);
    FD_ZERO(&exceptFds);

    int theFd = m_fd;
    assert(theFd >= 0);
    m_log->debug("Running receiver fd:{}",theFd);
    while(!m_terminate && m_fd >= 0) {
      FD_SET(theFd,&localFds);
      FD_SET(theFd,&exceptFds);
      struct timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 300000;
      int x = select(theFd+1,&localFds,0,&exceptFds,&timeout);
      if(x < 0) {
        perror("Failed to select");
        continue;
      }
      // FIXME:- Read into a larger buffer.
      int n = read(theFd,readBuff,sizeof readBuff);
      if(n == 0)
        continue;
      if(n < 0) {
        if(m_terminate)
          break;
        if(errno == EAGAIN ||
            errno == EINTR
            )
          continue;
        perror("Failed to read data.");
        continue;
      }
#if 0
      m_log->debug("Got %d bytes: ",n);
      for(int i = 0;i < n;i++) {
        m_log->debug("%02X ",(int) readBuff[i]);
        AcceptByte(readBuff[i]);
      }
      m_log->debug("\n");
#else
      for(int i = 0;i < n;i++) {
        AcceptByte(readBuff[i]);
      }
#endif
    }
    m_mutexExitOk.unlock();
    m_log->debug("Exiting receiver fd {} ",m_fd);

    return true;
  }

  //! Send packet
  void SerialComsC::SendPacket(const uint8_t *buff,int len)
  {
    assert(m_fd >= 0);

    std::vector<uint8_t> packet;
    packet.reserve(len + 6);


    packet.push_back(m_charSTX);
    int crc = len + 0x55;
    packet.push_back(len);
    for(int i =0;i < len;i++) {
      uint8_t data = buff[i];
      packet.push_back(data);
      crc += data;
    }
    packet.push_back(crc);
    packet.push_back(crc >> 8);
    packet.push_back(m_charETX);

    std::lock_guard<std::mutex> lock(m_accessTx);
    int at = 0;
    fd_set localFds;
    FD_ZERO(&localFds);
    FD_SET(m_fd,&localFds);

    while(at < packet.size()) {
      int x = select(m_fd+1,0,&localFds,0,0);
      if(x < 0) {
        m_log->error("Failed to select write. ");
        break;
      }
      int n = write(m_fd,&packet[at],packet.size());
      if(n < 0) {
        if(errno == EAGAIN || errno == EINTR)
          continue;
      }
      at += n;
    }
  }


  //! Set a parameter
  void SerialComsC::SendSetParam(int deviceId,ComsParameterIndexT param,uint8_t value)
  {
    PacketParam8ByteC msg;
    msg.m_header.m_packetType = CPT_SetParam;
    msg.m_header.m_deviceId = deviceId;
    msg.m_header.m_index = (uint16_t) param;
    msg.m_data.uint8[0] = value;
    SendPacket((uint8_t*) &msg,sizeof(msg.m_header)+1);
  }

  //! Set a parameter
  void SerialComsC::SendSetParam(int deviceId,ComsParameterIndexT param,int value)
  {
    PacketParam8ByteC msg;
    msg.m_header.m_packetType = CPT_SetParam;
    msg.m_header.m_deviceId = deviceId;
    msg.m_header.m_index = (uint16_t) param;
    msg.m_data.uint8[0] = value;
    SendPacket((uint8_t*) &msg,sizeof(msg.m_header)+1);
  }



  //! Set a parameter
  void SerialComsC::SendSetParam(int deviceId,ComsParameterIndexT param,float value)
  {
    PacketParam8ByteC msg;
    msg.m_header.m_packetType = CPT_SetParam;
    msg.m_header.m_deviceId = deviceId;
    msg.m_header.m_index = (uint16_t) param;
    msg.m_data.float32[0] = value;
    SendPacket((uint8_t*) &msg,sizeof(msg.m_header)+4);
  }


  //! Set a parameter
  void SerialComsC::SendSetParam(int deviceId,ComsParameterIndexT param,BufferTypeT &buff,int len)
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
  void SerialComsC::SendQueryParam(int deviceId,ComsParameterIndexT param)
  {
    PacketReadParamC msg;
    msg.m_packetType = CPT_ReadParam;
    msg.m_deviceId = deviceId;
    msg.m_index = (uint16_t) param;
    SendPacket((uint8_t*) &msg,sizeof(msg));
  }


  //! Send query devices message
  void SerialComsC::SendQueryDevices()
  {
    uint8_t data[2];
    data[0] = CPT_QueryDevices;
    SendPacket(data,1);
  }


  //! Set a device id
  void SerialComsC::SendSetDeviceId(uint8_t deviceId,uint32_t uid0,uint32_t uid1)
  {
    PacketDeviceIdC pkg;
    pkg.m_packetType = CPT_SetDeviceId;
    pkg.m_deviceId = deviceId;
    pkg.m_uid[0] = uid0;
    pkg.m_uid[1] = uid1;
    SendPacket((uint8_t*) &pkg,sizeof(pkg));
  }

  //! Send a move command
  void SerialComsC::SendMoveWithEffort(int deviceId,float pos,float effort,enum PositionReferenceT posRef)
  {
    int at = 0;

    struct PacketServoC servoPkt;
    servoPkt.m_packetType = CPT_Servo;
    servoPkt.m_deviceId = deviceId;
    //while(pos < ()-2.0 * M_PI)
    //pos += 3.14159265359;
    servoPkt.m_position = pos * 65535.0 / (4.0 * M_PI);
    if(effort < 0) effort = 0;
    if(effort > 10.0) effort = 10.0;
    servoPkt.m_torqueLimit = effort * 65535.0 / (10.0);
    servoPkt.m_mode = (int) posRef;

    SendPacket((uint8_t *)&servoPkt,sizeof servoPkt);
  }

  //! Send a calibration zero
  void SerialComsC::SendCalZero(int deviceId)
  {
    struct PacketCalZeroC pkt;
    pkt.m_packetType = CPT_CalZero;
    pkt.m_deviceId = deviceId;
    SendPacket((uint8_t *)&pkt,sizeof pkt);
  }

  //! Send a move command
  void SerialComsC::SendPing(int deviceId)
  {
    PacketPingPongC pkt;
    pkt.m_packetType = CPT_Ping;
    pkt.m_deviceId = deviceId;
    SendPacket((uint8_t *)&pkt,sizeof(struct PacketPingPongC));
  }
}
