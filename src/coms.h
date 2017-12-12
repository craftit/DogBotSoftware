#ifndef COMS_HEADER
#define COMS_HEADER 1

#ifdef __cplusplus
extern "C" {
#endif

#include "usbcfg.h"
#include "dogbot/protocol.h"

  extern bool g_comsInitDone;

  void InitComs(void);

  extern BaseAsynchronousChannel *g_packetStream;

  struct PacketT {
    uint8_t m_len;
    uint8_t m_data[62];
  };

  /* Get a free packet structure. */
  struct PacketT *GetEmptyPacket(systime_t timeout);

  /* Post packet. */
  bool PostPacket(struct PacketT *pkt);

  bool USBSendPacket(uint8_t *buff,int len);

  bool USBSendSync(void);
  bool USBSendPing(uint8_t targetDevice);

  /* Process a set parameter request. */
  bool SetParam(enum ComsParameterIndexT index,union BufferTypeT *dataBuff,int len);

  /* Retrieve the requested parameter information */
  bool ReadParam(enum ComsParameterIndexT index,int *len,union BufferTypeT *data);

  bool USBReadParamAndReply(enum ComsParameterIndexT paramIndex);

  void USBSendError(uint8_t deviceId,enum ComsErrorTypeT code,uint8_t originalPacketType,uint8_t data);

  /* Broadcast new parameter value.
   * This will send the update message over the appropriate channels, and will likely broadcast
   * to all connected devices, so shouldn't be called frequently.
   * */
  void SendParamUpdate(enum ComsParameterIndexT paramIndex);

  extern bool g_canBridgeMode;

  extern uint8_t g_deviceId;

  extern int g_usbDropCount;
  extern int g_usbErrorCount;


#ifdef __cplusplus
}
#endif

#endif
