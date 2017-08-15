
#include "coms.h"
#include "protocol.h"
#include "chmboxes.h"
#include "pwm.h"
#include "mathfunc.h"

#include <stdint.h>
#include <string.h>

const uint8_t g_charSTX = 0x02;
const uint8_t g_charETX = 0x03;

bool g_canBridgeMode = false;

bool SendSync(void)
{
  uint8_t buff[6];
  int at = 0;
  buff[at++] = CPT_Sync; // Type
  return SendPacket(buff,at);
}

bool SendPing(void)
{
  uint8_t buff[6];
  int at = 0;
  buff[at++] = CPT_Ping; // Type
  return SendPacket(buff,at);
}

// Error codes
//  1 - Unexpected packet.
//  2 - Packet unexpected length.

void SendError(
    ComsErrorTypeT code,
    uint8_t data
    )
{
  uint8_t buff[16];
  int at = 0;
  buff[at++] = CPT_Error; // Error.
  buff[at++] = code; // Unexpected packet type.
  buff[at++] = data; // Type of packet.

  SendPacket(buff,at);
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
    if(m_packetLen > 64) {
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


bool SetParam(PacketSetParamC *psp)
{
  switch((ComsParameterIndexT) psp->m_index)
  {
    case CPI_FirmwareVersion:
      return false; // Can't set this
    case CPI_PWMState:
      if(psp->m_data > 0) {
        PWMRun();
      } else {
        PWMStop();
      }
      break;
    case CPI_PWMMode:
      if(psp->m_data >= (int) CM_Final)
        return false;
      g_controlMode = (PWMControlModeT) psp->m_data;
      break;
    case CPI_PWMFullReport:
      g_pwmFullReport = psp->m_data > 0;
      break;
    case CPI_CANBridgeMode:
      g_canBridgeMode = psp->m_data > 0;
      break;
    default:
      return false;
  }
  return true;
}

bool ReadParam(PacketReadParamC *psp)
{
  PacketSetParamC reply;
  reply.m_packetType = CPT_SetParam;

  switch((ComsParameterIndexT) psp->m_index)
  {
    case CPI_FirmwareVersion: {
      reply.m_data = 0;
    } break;
    case CPI_PWMState: {
      reply.m_data = g_pwmThreadRunning;
    } break;
    case CPI_PWMMode: {
      reply.m_data = g_controlMode;
    } break;
    case CPI_PWMFullReport: {
      reply.m_data = g_pwmFullReport;
    } break;
    case CPI_CANBridgeMode: {
      reply.m_data = g_canBridgeMode;
    } break;
    default:
      return false;
  }
  SendPacket((uint8_t *)&reply,sizeof(reply));

  return true;
}


//! Process received packet.
void SerialDecodeC::ProcessPacket()
{
  uint8_t buff[16];
  // m_data[0] //
  switch(m_data[0])
  {
  case CPT_Ping: { // Ping.
    int at = 0;
    buff[at++] = CPT_Pong; // Type, ping reply.
    SendPacket(buff,at);
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
  case CPT_ReadParam: {
    PacketReadParamC *psp = (PacketReadParamC *) m_data;
    if(!ReadParam(psp)) {
      SendError(CET_ParameterOutOfRange,m_data[1]);
    }
  } break;
  case CPT_SetParam: {
    PacketSetParamC *psp = (PacketSetParamC *) m_data;
    if(!SetParam(psp)) {
      SendError(CET_ParameterOutOfRange,m_data[1]);
    }
  } break;
  case CPT_Servo: { // Goto position.
    if(m_packetLen != sizeof( PacketServoC)) {
      SendError(CET_UnexpectedPacketSize,m_data[1]);
      break;
    }
    PacketServoC *ps = (PacketServoC *) m_data;
    g_torqueLimit = ((float) ps->m_torqueLimit) * 10.0 / (65535.0);
    g_demandPhasePosition = ((float) ps->m_position) * 7.0 * 21.0 * M_PI * 2.0/ 65535.0;
  } break;
  default: {
    SendError(CET_UnknownPacketType,m_data[1]);
    //RavlDebug("Unexpected packet type %d ",(int) m_data[1]);
  } break;
  }
}

#define PACKET_QUEUE_SIZE 8

static msg_t g_emptyPacketData[PACKET_QUEUE_SIZE];
static mailbox_t g_emptyPackets;
static msg_t g_fullPacketData[PACKET_QUEUE_SIZE];
static mailbox_t g_fullPackets;

static PacketT g_packetArray[PACKET_QUEUE_SIZE];


/* Get a free packet structure. */
struct PacketT *GetEmptyPacket(systime_t timeout)
{
  msg_t msg;
  if(chMBFetch(&g_emptyPackets,&msg,timeout) != MSG_OK)
    return 0;
  return (PacketT *)msg;
}

/* Post packet. */
bool PostPacket(struct PacketT *pkt)
{
  if(chMBPost(&g_fullPackets,(msg_t) pkt,TIME_IMMEDIATE) == MSG_OK)
    return true;

  // This shouldn't happen, as if we can't acquire an empty buffer
  // unless there is space, but we don't want to loose the buffer
  // so attempt to add it back to the free list
  if(chMBPost(&g_emptyPackets,(msg_t) pkt,TIME_IMMEDIATE) != MSG_OK) {
    // Things are screwy. Panic ?
  }
  return false;
}



static THD_WORKING_AREA(waThreadRxComs, 512);
static THD_FUNCTION(ThreadRxComs, arg) {

  (void)arg;
  chRegSetThreadName("rxcoms");
  while(true) {
    int ret = streamGet(g_comsDecode.m_SDU);
    if(ret == STM_RESET) {
      chThdSleepMilliseconds(500);
      continue;
    }
    g_comsDecode.AcceptByte(ret);
  }
}

static THD_WORKING_AREA(waThreadTxComs, 512);
static THD_FUNCTION(ThreadTxComs, arg) {

  (void)arg;
  chRegSetThreadName("txcoms");
  static uint8_t txbuff[261];
  while(true) {
    struct PacketT *packet = 0;
    if(chMBFetch(&g_fullPackets,(msg_t*) &packet,TIME_INFINITE) != MSG_OK)
      continue;

    if(SDU1.config->usbp->state == USB_ACTIVE) {

      uint8_t len = packet->m_len;
      uint8_t *buff = packet->m_data;

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

      chnWrite(g_packetStream, txbuff, size);
    }

    chMBPost(&g_emptyPackets,reinterpret_cast<msg_t>(packet),TIME_INFINITE);
  }
}


bool SendPacket(
    uint8_t *buff,
    int len
    )
{
  // Just truncate for the moment, it is better than overwriting memory.
  struct PacketT *pkt;
  //if(len > sizeof pkt->m_data) len = sizeof pkt->m_data;

  if((pkt = GetEmptyPacket(TIME_IMMEDIATE)) == 0)
    return false;

  pkt->m_len = len;
  memcpy(&pkt->m_data,buff,len);
  PostPacket(pkt);

  return true;
}

BaseSequentialStream *g_packetStream = 0;

void InitComs()
{
  static bool initDone = false;
  if(initDone)
    return ;
  initDone = true;

  g_comsDecode.m_SDU = (BaseSequentialStream *)&SDU1;
  g_packetStream = (BaseSequentialStream *)&SDU1;

  chMBObjectInit(&g_emptyPackets,g_emptyPacketData,PACKET_QUEUE_SIZE);
  for(int i = 0;i < PACKET_QUEUE_SIZE;i++) {
    chMBPost(&g_emptyPackets,reinterpret_cast<msg_t>(&g_packetArray[i]),TIME_IMMEDIATE);
  }
  chMBObjectInit(&g_fullPackets,g_fullPacketData,PACKET_QUEUE_SIZE);

  chThdCreateStatic(waThreadTxComs, sizeof(waThreadTxComs), NORMALPRIO+1, ThreadTxComs, NULL);
  chThdCreateStatic(waThreadRxComs, sizeof(waThreadRxComs), NORMALPRIO, ThreadRxComs, NULL);

}

