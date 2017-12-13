#ifndef SERIALPACKET_HEADER
#define SERIALPACKET_HEADER 1

#include <stdint.h>

#include "coms.h"
#include "canbus.h"
#include "dogbot/protocol.h"
#include "chmboxes.h"
#include "pwm.h"
#include "mathfunc.h"
#include "motion.h"
#include "drv8503.h"
#include "hal_channels.h"

#include <string.h>

static const uint8_t g_charSTX = 0x02;
static const uint8_t g_charETX = 0x03;

class SerialDecodeC
{
public:

  //! Reset state machine
  void ResetStateMachine()
  {
    m_state = 0;
    m_at = 0;
  }

  //! Accept a byte
  void AcceptByte(uint8_t sendByte);

  //! Process received packet.
  void ProcessPacket();

  int m_state = 0;
  int m_checkSum = 0;
  int m_packetLen = 0;
  static const int m_maxPacketSize = 64;
  uint8_t m_data[m_maxPacketSize];
  int m_at = 0;

  BaseAsynchronousChannel *m_SDU = 0;

  // Packet structure.
  // x    STX
  // x    Len - Of data excluding STX,ETX and Checksum.
  // 0    Address
  // 1    Type
  // 2..n data.
  // n    2-CRC
  // n    ETX.
};

extern SerialDecodeC g_comsDecode;

#endif
