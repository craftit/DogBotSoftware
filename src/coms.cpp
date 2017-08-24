
#include <stdint.h>

#include "coms.h"
#include "canbus.h"
#include "protocol.h"
#include "chmboxes.h"
#include "pwm.h"
#include "mathfunc.h"
#include "motion.h"
#include "drv8503.h"

#include <string.h>

const uint8_t g_charSTX = 0x02;
const uint8_t g_charETX = 0x03;

bool g_canBridgeMode = false;

bool SendSync(void)
{
  uint8_t buff[2];
  int at = 0;
  buff[at++] = CPT_Sync; // Type
  return SendPacket(buff,at);
}

bool SendPing(uint8_t deviceId)
{
  PacketPingPongC pkt;
  pkt.m_packetType = CPT_Ping;
  pkt.m_deviceId = deviceId;
  return SendPacket((uint8_t *) &pkt,sizeof(struct PacketPingPongC));
}

// Error codes
//  1 - Unexpected packet.
//  2 - Packet unexpected length.

void SendError(
    ComsErrorTypeT code,
    uint8_t data
    )
{
  PacketErrorC pkt;
  pkt.m_packetType = CPT_Error;
  pkt.m_deviceId = g_deviceId;
  pkt.m_errorCode = code;
  pkt.m_errorData = data;

  SendPacket((uint8_t *) &pkt,sizeof(struct PacketErrorC));
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


bool SetParamMsg(struct PacketParamC *psp)
{
  return SetParam((enum ComsParameterIndexT) psp->m_header.m_index,psp->m_data);
}

bool SetParam(enum ComsParameterIndexT index,int data)
{
  switch(index )
  {
    case CPI_FirmwareVersion:
      return false; // Can't set this
    case CPI_PWMState:
      if(data > 0) {
        PWMRun();
      } else {
        PWMStop();
      }
      break;
    case CPI_PWMMode:
      if(data >= (int) CM_Final)
        return false;
      g_controlMode = (PWMControlModeT) data;
      break;
    case CPI_PWMFullReport:
      g_pwmFullReport = data > 0;
      break;
    case CPI_CANBridgeMode:
      g_canBridgeMode = data > 0;
      break;
    case CPI_BoardUID:
      return false;
    case CPI_DRV8305_01:
    case CPI_DRV8305_02:
    case CPI_DRV8305_03:
    case CPI_DRV8305_04:
    case CPI_DRV8305_05:
      return false;
    default:
      return false;
  }
  return true;
}


// Up to 8 bytes of data may be returned by this function.

bool ReadParam(enum ComsParameterIndexT index,int *len,union BufferTypeT *data)
{
  switch(index)
  {
    case CPI_FirmwareVersion:
      *len = 1;
      data->uint8[0] = 1;
      break;
    case CPI_PWMState:
      *len = 1;
      data->uint8[0] = g_pwmThreadRunning;
      break;
    case CPI_PWMMode:
      *len = 1;
      data->uint8[0] = g_controlMode;
      break;
    case CPI_PWMFullReport:
      *len = 1;
      data->uint8[0] = g_pwmFullReport;
      break;
    case CPI_CANBridgeMode:
      *len = 1;
      data->uint8[0] = g_canBridgeMode;
      break;
    case CPI_BoardUID:
      *len = 8;
      data->uint32[0] = g_nodeUId[0];
      data->uint32[1] = g_nodeUId[1];
      break;

    case CPI_DRV8305_01:
    case CPI_DRV8305_02:
    case CPI_DRV8305_03:
    case CPI_DRV8305_04:
    case CPI_DRV8305_05: {
      int reg = ((int) index - CPI_DRV8305)+1;
      *len = 2;
      data->uint16[0] = Drv8503ReadRegister(reg);
    } break;
    case CPI_TIM1_SR: {
      stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;
      *len = 2;
      data->uint16[0] = (uint16_t) tim->SR;
    } break;
    default:
      return false;
  }
  return true;
}

bool ReadParamAndReply(PacketReadParamC *psp)
{
  struct PacketParam8ByteC reply;
  reply.m_header.m_packetType = CPT_ReportParam;
  reply.m_header.m_deviceId = g_deviceId;
  reply.m_header.m_index = psp->m_index;
  int len = 0;
  if(!ReadParam((ComsParameterIndexT) psp->m_index,&len,&reply.m_data))
    return false;

  SendPacket((uint8_t *)&reply,sizeof(reply.m_header) + len);

  return true;
}


//! Process received packet.
void SerialDecodeC::ProcessPacket()
{
  // m_data[0] //
  switch(m_data[0])
  {
  case CPT_Ping: { // Ping.
    const PacketPingPongC *pkt = (const PacketPingPongC *) m_data;
    if(pkt->m_deviceId == g_deviceId ||
        pkt->m_deviceId == 0)
    {
      // If it is for this node, just reply
      PacketPingPongC pkt;
      pkt.m_packetType = CPT_Pong;
      pkt.m_deviceId = g_deviceId;
      SendPacket((uint8_t *) &pkt,sizeof(struct PacketPingPongC));
    } else {
      // In bridge mode forward this to the CAN bus
      if(g_canBridgeMode) {
        CANPing(CPT_Ping,pkt->m_deviceId);
      }
    }
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
    struct PacketReadParamC *psp = (struct PacketReadParamC *) m_data;
    if(psp->m_deviceId == g_deviceId) {
      if(!ReadParamAndReply(psp)) {
        SendError(CET_ParameterOutOfRange,m_data[1]);
      }
    } else {

    }
  } break;
  case CPT_SetParam: {
    struct PacketParamC *psp = (struct PacketParamC *) m_data;
    if(psp->m_header.m_deviceId == g_deviceId) {
      if(!SetParamMsg(psp)) {
        SendError(CET_ParameterOutOfRange,m_data[1]);
      }
    } else {
      if(g_canBridgeMode) {
        if(!CANSendSetParam(psp->m_header.m_deviceId,psp->m_header.m_index,psp->m_data)) {
          SendError(CET_CANTransmitFailed,m_data[0]);
        }
      }
    }
  } break;
  case CPT_ServoAbs:  // Goto position.
  case CPT_ServoRel: { // Goto position.
    if(m_packetLen != sizeof( PacketServoC)) {
      SendError(CET_UnexpectedPacketSize,m_data[1]);
      break;
    }
    PacketServoC *ps = (PacketServoC *) m_data;
    if(ps->m_deviceId == g_deviceId) {
      if(m_data[0] == CPT_ServoRel) {
        PWMSetPosition(ps->m_position,ps->m_torque);
      } else {
        MotionSetPosition(ps->m_position,ps->m_torque);
      }
    } else {
      if(g_canBridgeMode) {
        CANSendServo((ComsPacketTypeT) m_data[0],ps->m_deviceId,ps->m_position,ps->m_torque);
      }
    }
  } break;
  case CPT_QueryDevices: {
    // First report about myself
    struct PacketDeviceIdC pkt;
    pkt.m_packetType = CPT_AnnounceId;
    pkt.m_deviceId = g_deviceId;
    pkt.m_uid[0] = g_nodeUId[0];
    pkt.m_uid[1] = g_nodeUId[1];
    SendPacket((uint8_t *) &pkt,sizeof(struct PacketDeviceIdC));

    // Are we bridging ?
    if(g_canBridgeMode) {
      if(!CANSendQueryDevices())
        SendError(CET_CANTransmitFailed,m_data[0]);
    }
  } break;
  case CPT_AnnounceId:
    break;
  case CPT_SetDeviceId: {
    const struct PacketDeviceIdC *pkt = (const struct PacketDeviceIdC *) m_data;
    // Check if we're setting the id for this device.
    if(pkt->m_uid[0] == g_nodeUId[0] &&
        pkt->m_uid[1] == g_nodeUId[1]) {
      g_deviceId = pkt->m_deviceId;

      // Announce change.
      {
        struct PacketDeviceIdC rpkt;
        rpkt.m_packetType = CPT_AnnounceId;
        rpkt.m_deviceId = g_deviceId;
        rpkt.m_uid[0] = g_nodeUId[0];
        rpkt.m_uid[1] = g_nodeUId[1];
        SendPacket((uint8_t *) &rpkt,sizeof(struct PacketDeviceIdC));
      }
    } else {
      if(g_canBridgeMode) {
        CANSendSetDevice(pkt->m_deviceId,pkt->m_uid[0],pkt->m_uid[1]);
      }
    }
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
    } else {
      // If we've lost our connection turn bridge mode off.
      g_canBridgeMode = false;
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

  chThdCreateStatic(waThreadTxComs, sizeof(waThreadTxComs), NORMALPRIO, ThreadTxComs, NULL);
  chThdCreateStatic(waThreadRxComs, sizeof(waThreadRxComs), NORMALPRIO, ThreadRxComs, NULL);

}

