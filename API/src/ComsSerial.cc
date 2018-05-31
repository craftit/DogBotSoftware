
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

#include "../include/dogbot/ComsSerial.hh"
#include "../include/dogbot/DogBotAPI.hh"

namespace DogBotN
{

  ComsSerialC::ComsSerialC(const char *portAddr)
  {
    Open(portAddr);
  }

  //! default
  ComsSerialC::ComsSerialC()
  {}

  // Disconnects and closes file descriptors
  ComsSerialC::~ComsSerialC()
  {
    if(m_fd >= 0)
      Close();
  }

  //! Close connection
  void ComsSerialC::Close()
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
  bool ComsSerialC::IsReady() const
  {
    return m_fd >= 0;
  }


  bool ComsSerialC::Open(const std::string &portAddr)
  {
    m_terminate = false;
    if(m_fd >= 0) {
      m_log->info("Coms already open. ");
      return false;
    }

    m_name = portAddr;
    m_log->info("Opening: '{}'",portAddr);

    m_fd = open(portAddr.c_str(),O_RDWR | O_NONBLOCK);
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

      speed_t speed = B57600; // B38400;
      cfsetispeed(&termios_p, speed); //
      cfsetospeed(&termios_p, speed);

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
    // Send some messages to make sure receiver is in sync
    for(int i =0;i < 3;i++)
      SendSync();
    // If we're connecting over the serial port we want to be in bridge mode.
    SendEnableBridge(true);
    return true;
  }

  void ComsSerialC::AcceptByte(uint8_t sendByte)
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
        m_log->warn("Packet to long. {} ",m_packetLen);
        if(sendByte != m_charSTX)  // Otherwise stay in the current state.
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
        //m_log->info("Got packet {} {} len:{} ",(int) m_data[0],ComsPacketTypeToString((enum ComsPacketTypeT) m_data[0]),m_packetLen);
        ProcessPacket(m_data,m_packetLen);
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

  bool ComsSerialC::RunRecieve()
  {
    assert(m_fd >= 0);
    uint8_t readBuff[1024];
    fd_set localFds;
    fd_set exceptFds;
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
        perror("Failed to read data. device most likely disconnected: ");
        close(theFd);
        m_fd = 0;
        break;
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
  void ComsSerialC::SendPacketWire(const uint8_t *buff,int len)
  {
    assert(m_fd >= 0);

    //m_log->info("Sent packet {} {} len:{} ",(int) buff[0],ComsPacketTypeToString((enum ComsPacketTypeT) buff[0]),len);

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

}
