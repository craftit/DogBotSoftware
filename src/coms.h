#ifndef COMS_HEADER
#define COMS_HEADER 1

#ifdef __cplusplus
extern "C" {
#endif

#include "usbcfg.h"
#include "dogbot/protocol.h"

  void InitComs(void);

  extern BaseSequentialStream *g_packetStream;

  struct PacketT {
    uint8_t m_len;
    uint8_t m_data[62];
  };

  /* Get a free packet structure. */
  struct PacketT *GetEmptyPacket(systime_t timeout);

  /* Post packet. */
  bool PostPacket(struct PacketT *pkt);

  bool SendPacket(uint8_t *buff,int len);

  bool SendSync(void);
  bool SendPing(uint8_t targetDevice);

  /* Process a set parameter request. */
  bool SetParam(enum ComsParameterIndexT index,union BufferTypeT *dataBuff,int len);

  /* Retrieve the requested parameter information */
  bool ReadParam(enum ComsParameterIndexT index,int *len,union BufferTypeT *data);

  void SendError(uint8_t deviceId,enum ComsErrorTypeT code,uint8_t data);

  extern bool g_canBridgeMode;

  extern uint8_t g_deviceId;

#ifdef __cplusplus
}
#endif

#endif
