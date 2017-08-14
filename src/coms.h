#ifndef COMS_HEADER
#define COMS_HEADER 1

#ifdef __cplusplus
extern "C" {
#endif

  #include "usbcfg.h"

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
  bool SendPing(void);

  extern bool g_canBridgeMode;

#ifdef __cplusplus
}
#endif

#endif
