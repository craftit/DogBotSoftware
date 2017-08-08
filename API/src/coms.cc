
#include <sstream>

#include <thread>
#include <vector>
#include <functional>
#include <assert.h>
#include <mutex>

#include <stdio.h>

#include <fstream>
#include <iostream>
#include <fcntl.h>

#include "coms.hh"
#include <unistd.h>

#define ROS_DEBUG printf
#define ROS_ERROR printf

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
    if(m_fd >= 0)
      close(m_fd);
    m_fd = -1;
  }

  bool SerialComsC::Open(const char *portAddr)
  {
    if(m_fd >= 0) {
      Close();
    }

    std::cerr << "Opening: '" << portAddr << "' " << std::endl;
    m_fd = open(portAddr,O_RDWR | O_NONBLOCK);
    std::cerr << "Ret " << m_fd << std::endl;
    if(m_fd < 0) {
      std::cerr << "Failed to open file " << portAddr << " " << std::endl;
      perror("Failed to open file ");
      return false;
    }
    std::cerr << "Port opened ok  '" << portAddr << "' " << std::endl;

    m_threadRecieve = std::move(std::thread { [this]{ RunRecieve(); } });
    return true;
  }


  void SerialComsC::SetHandler(int packetId,const std::function<void (uint8_t *data,int )> &handler)
  {
    std::lock_guard<std::mutex> lock(m_accessPacketHandler);
    assert(packetId < 256);

    while(m_packetHandler.size() <= packetId) {
      m_packetHandler.push_back(std::function<void (uint8_t *data,int )>());
    }
    m_packetHandler[packetId] = handler;
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
      //ROS_DEBUG("Checksum1 : %d %d ",(int)  cs1 , (int) sendByte);
      if(cs1 != sendByte) {
        ROS_DEBUG("Checksum failed. ");
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
      //ROS_DEBUG("Checksum2 : %d %d ",(int) ((m_checkSum >> 8) & 0xff) , (int) sendByte);
      if(cs2 != sendByte) {
        std::cerr << "Checksum failed. \n";
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
        std::cerr << "Packet corrupted. \n";
      }
      m_state = 0;
      break;
    }
  }

  //! Process received packet.
  void SerialComsC::ProcessPacket()
  {
    std::string data;
    for(int i = 0;i < m_packetLen;i++) {
      data += std::to_string(m_data[i]) + " ";
    }
    ROS_DEBUG("Got packet [%d] %s ",m_packetLen,data.c_str());

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
        std::cerr << "Got sync. \n";
        break;
      case CPT_Pong:
        std::cerr << "Got pong. \n";
        break;
      case CPT_Error:
        ROS_ERROR("Received error code: %d  Arg:%d ",(int) m_data[1],(int) m_data[2]);
        return ;
        break;
      default: {
        std::lock_guard<std::mutex> lock(m_accessPacketHandler);
        if(packetId < m_packetHandler.size()) {
          const std::function<void (uint8_t *data,int )> &handler = m_packetHandler[packetId];
          if(handler) {
            handler(m_data,m_packetLen);
            return ;
          }
        }
        ROS_DEBUG("Don't know how to handle packet %d (Of %d) ",packetId,(int) m_packetHandler.size());
      } break;
  #if 0
    case 2: { // ADC Data.
      int current = ((int) m_data[2])  + (((int) m_data[3]) << 8);
      int volt = ((int) m_data[4])  + (((int) m_data[5]) << 8);
      ROS_DEBUG("I:%4d V:%4d  Phase:%d  PWM:%d ",current,volt,(int) m_data[6],(int) m_data[7]);
    } break;
    default:
      ROS_DEBUG("Unexpected packet type %d ",(int) m_data[1]);
      break;
  #endif
    }
  }

  bool SerialComsC::RunRecieve()
  {
    std::cerr << "Running receiver" << std::endl;
    assert(m_fd >= 0);
    uint8_t readBuff[1024];
    fd_set localFds;
    FD_ZERO(&localFds);

    while(!m_terminate && m_fd >= 0) {
      FD_SET(m_fd,&localFds);
      int x = select(m_fd+1,&localFds,0,0,0);
      if(x < 0) {
        perror("Failed to select");
        break;
      }
      // FIXME:- Read into a larger buffer.
      int n = read(m_fd,readBuff,sizeof readBuff);
      if(n == 0)
        break;
      if(n < 0) {
        if(errno == EAGAIN ||
            errno == EINTR
            )
          continue;
        perror("Failed to read data.");
        break;
      }
      //ROS_DEBUG("Got byte %d ",(int) readByte);
      for(int i = 0;i < n;i++)
        AcceptByte(readBuff[i]);
    }
    std::cerr << "Exiting receiver" << std::endl;

    return true;
  }

  //! Send packet
  void SerialComsC::SendPacket(uint8_t *buff,int len)
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
        perror("write select");
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


  //! Send a move command
  void SerialComsC::SendMove(int servoId,int pos)
  {
    uint8_t data[64];

    int at = 0;
    data[at++] = CPT_Servo;
    data[at++] = servoId;
    data[at++] = (int) pos;

    SendPacket(data,at);
  }

  //! Send a move command
  void SerialComsC::SendMoveWithEffort(float pos,float effort)
  {
    uint8_t data[64];

    int at = 0;
    data[at++] = CPT_Servo;
    data[at++] = 0; // SM_Position

    if(pos < 0)
      pos = 0;
    if(pos > 1.0)
      pos = 1.0;

    int xpos = 1024.0 - (pos * 1024.0 * 2.0 / 3.14159265359); // 0 to 45 degress in radians
    data[at++] = xpos;
    data[at++] = xpos >> 8;

    if(effort < 0)
      effort = 0;
    if(effort > 1.0)
      effort = 1.0;

    int xeffort = effort * 1024.0;
    data[at++] = xeffort;
    data[at++] = xeffort >> 8;

    // We don't use the velocity limit at the moment
    int xvelLimit = 1024;
    data[at++] = xvelLimit;
    data[at++] = xvelLimit >> 8;

    SendPacket(data,at);
  }


  //! Send a move command
  void SerialComsC::SendPing()
  {
    uint8_t data[64];

    int at = 0;
    data[at++] = CPT_Ping;

    SendPacket(data,at);
  }
}
