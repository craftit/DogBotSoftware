#ifndef DOGBOG_SERIALCOMS_HEADER
#define DOGBOG_SERIALCOMS_HEADER 1

#include "dogbot/Coms.hh"

namespace DogBotN {

  //! Low level serial communication over usb with the driver board

  class ComsSerialC
   : public ComsC
  {
  public:
    ComsSerialC(const char *portAddr);

    //! default
    ComsSerialC();

    //! Destructor
    // Disconnects and closes file descriptors
    virtual ~ComsSerialC();

    //! Open a port.
    virtual bool Open(const std::string &portAddr) override;

    //! Close connection
    virtual void Close() override;

    //! Is connection ready ?
    virtual bool IsReady() const override;

    //! Send packet
    virtual void SendPacketWire(const uint8_t *data,int len) override;

    //! Accept a byte
    void AcceptByte(uint8_t sendByte);


  protected:
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

    std::mutex m_accessTx;
    std::timed_mutex m_mutexExitOk;

  };
}
#endif
