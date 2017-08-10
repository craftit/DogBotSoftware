
#include "coms.h"
#include "protocol.h"

#include <stdint.h>

const uint8_t g_charSTX = 0x02;
const uint8_t g_charETX = 0x03;


bool SendPacket(
    BaseSequentialStream *chp,
    uint8_t *buff,
    int len
    )
{

  uint8_t txbuff[32];
  uint8_t *at = txbuff;
  *(at++) = g_charSTX;
  *(at++) = len;
  int crc = len + 0x55;
  for(int i = 0;i < len;i++) {
    uint8_t data = buff[i];
    *(at++) = data;
    crc += data;
  }
  *(at++) = crc;
  *(at++) = crc>>8;
  *(at++) = g_charETX;
  int size = at - txbuff;
  chnWrite(chp, txbuff, size);
  //streamWrite();

  return true;
}

bool SendSync(
    BaseSequentialStream *chp
    )
{
  uint8_t buff[6];
  int at = 0;
  buff[at++] = CPT_Sync; // Type
  return SendPacket(chp,buff,at);
}

bool SendPing(
    BaseSequentialStream *chp
    )
{
  uint8_t buff[6];
  int at = 0;
  buff[at++] = CPT_Sync; // Type
  return SendPacket(chp,buff,at);
}

// Error codes
//  1 - Unexpected packet.
//  2 - Packet unexpected length.

void SendError(
    BaseSequentialStream *chp,
    uint8_t code,
    uint8_t data
    )
{
  uint8_t buff[16];
  int at = 0;
  buff[at++] = CPT_Error; // Error.
  buff[at++] = code; // Unexpected packet type.
  buff[at++] = data; // Type of packet.

  SendPacket(chp,buff,at);
}


bool SendServoState(BaseSequentialStream *chp,float position,float torque)
{
  uint8_t buff[16];
  int at = 0;
  buff[at++] = CPT_Servo; // Error.
  buff[at++] = 1;
  buff[at++] = 2;

  SendPacket(chp,buff,at);

  return true;
}



class SerialDecodeC
{
public:

  //! Accept a byte
  void AcceptByte(uint8_t sendByte);

  //! Process received packet.
  void ProcessPacket();

  int m_state = 0;
  int m_checkSum = 0;
  int m_packetLen = 0;
  uint8_t m_data[255];
  int m_at = 0;

  BaseSequentialStream  *m_SDU = 0;

  // Packet structure.
  // x    STX
  // x    Len - Of data excluding STX,ETX and Checksum.
  // 0    Address
  // 1    Type
  // 2..n data.
  // n    2-CRC
  // n    ETX.
} g_comsDecode;

void SerialDecodeC::AcceptByte(uint8_t sendByte)
{
  switch(m_state)
  {
  case 0: // Wait for STX.
    if(sendByte == g_charSTX)
      m_state = 1;
    // Else remain in state 0.
    break;
  case 1: // Packet length.
    m_packetLen = sendByte;
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
    //RavlDebug("Checksum1 : %d %d ",(int)  cs1 , (int) sendByte);
    if(cs1 != sendByte) {
      //RavlDebug("Checksum failed. ");
      if(sendByte == g_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 4;
  } break;
  case 4: { // CRC 2
    uint8_t cs2 = ((m_checkSum >> 8) & 0xff);
    //RavlDebug("Checksum2 : %d %d ",(int) ((m_checkSum >> 8) & 0xff) , (int) sendByte);
    if(cs2 != sendByte) {
      //RavlDebug("Checksum failed. ");
      if(sendByte == g_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 5;
  } break;
  case 5: // ETX.
    if(sendByte == g_charETX) {
      //RavlDebug("Got packet!");
      ProcessPacket();
    } else {
      //RavlDebug("Packet corrupted.");
    }
    m_state = 0;
    break;
  }
}

//! Process received packet.
void SerialDecodeC::ProcessPacket()
{
  // m_data[0] //
  switch(m_data[1])
  {
  case CPT_Ping: { // Ping.
    uint8_t buff[16];
    int at = 0;
    buff[at++] = 1; // Address
    buff[at++] = 1; // Type, ping reply.
    SendPacket(m_SDU,buff,at);
  } break;

  case CPT_Pong: { // Ping reply.
    // Drop it
    break;
  }
  case CPT_Sync: { // Sync.
    // Drop it
  } break;

  case CPT_Error: { // Error.
    // Drop it
  } break;

  case CPT_Servo: { // Goto position.
    if(m_packetLen != 6) {
      SendError(m_SDU,2,m_data[1]);
      break;
    }

    uint32_t pos = ((uint32_t) m_data[2])  +
                    (((uint32_t) m_data[3]) << 8) +
                    (((uint32_t) m_data[4]) << 16) +
                    (((uint32_t) m_data[5]) << 24);
    //GotoPosition(pos);
  } break;
  default: {
    //SendError(1,m_data[1]);
    //RavlDebug("Unexpected packet type %d ",(int) m_data[1]);
  } break;
  }
}


static THD_WORKING_AREA(waThreadComs, 512);
static THD_FUNCTION(ThreadComs, arg) {

  (void)arg;
  chRegSetThreadName("coms");
  while (true) {
    int ret = streamGet(g_comsDecode.m_SDU);
    if(ret == STM_RESET) {
      chThdSleepMilliseconds(500);
      continue;
    }
    g_comsDecode.AcceptByte(ret);
  }
}

BaseSequentialStream *g_packetStream = 0;

void InitComs()
{
  g_comsDecode.m_SDU = (BaseSequentialStream *)&SDU1;
  g_packetStream = (BaseSequentialStream *)&SDU1;
  //chThdCreateStatic(waThreadComs, sizeof(waThreadComs), NORMALPRIO, ThreadComs, NULL);

}

