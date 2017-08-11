#ifndef DOGBOG_COMS_HEADER
#define DOGBOG_COMS_HEADER 1

#include <sstream>

#include <thread>
#include <vector>
#include <functional>
#include <assert.h>
#include <mutex>

#include "protocol.h"
#include <cstdint>

namespace DogBotN {

  class SerialComsC
  {
  public:
    SerialComsC(const char *portAddr);

    //! default
    SerialComsC();

    //! Destructor
    // Disconnects and closes file descriptors
    ~SerialComsC();

    //! Open a port.
    bool Open(const char *portAddr);

    //! Close connection
    void Close();

    //! Accept a byte
    void AcceptByte(uint8_t sendByte);

    //! Process received packet.
    void ProcessPacket();

    //! Send packet
    void SendPacket(const uint8_t *data,int len);

    //! Send a move command
    void SendMove(int servoId,int pos);

    //! Send a move command with an effort level.
    void SendMoveWithEffort(float pos,float effort);

    //! Set a parameter
    void SendSetParam(ComsParameterIndexT param,uint16_t value);

    //! Send a move command
    void SendPing();

    //! Set the handler for a particular type of packet.
    void SetHandler(ComsPacketTypeT packetType,const std::function<void (uint8_t *data,int )> &handler);

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

    std::thread m_threadRecieve;

    std::mutex m_accessPacketHandler;
    std::mutex m_accessTx;

    std::vector<std::function<void (uint8_t *data,int )> > m_packetHandler;
  };
}
#endif
