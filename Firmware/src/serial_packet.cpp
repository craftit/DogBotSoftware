
#include "serial_packet.hh"


void SerialDecodeC::AcceptByte(uint8_t sendByte)
{
  switch(m_state)
  {
  default: // This can only be caused by a bug.
    g_usbErrorCount++;
    m_state = 0;
    /* no break */
  case 0: // Wait for STX.
    if(sendByte == g_charSTX)
      m_state = 1;
    // Else remain in state 0.
    break;
  case 1: // Packet length.
    m_packetLen = sendByte;
    if(m_packetLen >= m_maxPacketSize) {
      if(sendByte != g_charSTX) // This will always be false, but for good form.
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
      // FIXME Count corrupted packets ?
      //RavlDebug("Packet corrupted.");
      if(sendByte == g_charSTX) {
        m_state = 1;
        break;
      }
    }
    m_state = 0;
    break;
  }
}

//! Process received packet.

void SerialDecodeC::ProcessPacket()
{
  ::ProcessPacket(m_data,m_packetLen);
}

SerialDecodeC g_comsDecode;


