#ifndef COMS_HEADER
#define COMS_HEADER 1

#define BMC_MAXPACKETSIZE 63

#ifdef __cplusplus
extern "C" {
#endif

#include "hal.h"
#include "dogbot/protocol.h"

  extern bool g_comsInitDone;

  void InitUSB(void);
  void InitComs(void);


  struct PacketT {
    uint8_t m_len;
    uint8_t m_data[BMC_MAXPACKETSIZE];
  };

  /* Get a free packet structure. */
  struct PacketT *USBGetEmptyPacket(systime_t timeout);

  /* Post packet. */
  bool USBPostPacket(struct PacketT *pkt);

  /* Send packet with buffer */
  bool USBSendPacket(uint8_t *buff,int len);

  /* Process in an incoming packet. */
  void ProcessPacket(const uint8_t *m_data,int m_packetLen);

  /*************************************
   * Communication helper functions.
   */

  /* Send a sync message. */
  bool USBSendSync(void);

  /* Send a ping message to a device. */
  bool USBSendPing(uint8_t targetDevice);

  /* Send a boot-loader result message. */
  bool USBSendBootLoaderResult(uint8_t deviceId,uint8_t lastSeqNum,enum BootLoaderStateT state,enum FlashOperationStatusT result);

  /* Send boot-loader check-sum result. */
  bool USBSendBootLoaderCheckSumResult(uint8_t deviceId,uint8_t seqNum,uint32_t sum);

  /* Send bootloader data */
  bool USBSendBootLoaderData(uint8_t deviceId,uint8_t seqNum,uint8_t *data,uint8_t len);

  /* Process a set parameter request. */
  bool SetParam(enum ComsParameterIndexT index,union BufferTypeT *data,int len);

  /* Retrieve the requested parameter information */
  bool ReadParam(enum ComsParameterIndexT index,int *len,union BufferTypeT *data);

  /* Read a parameter and send its value back over USB. */
  bool USBReadParamAndReply(enum ComsParameterIndexT paramIndex);

  /* Report an error. */
  void USBSendError(uint8_t deviceId,enum ComsErrorTypeT code,uint8_t originalPacketType,uint8_t data);

  /* Report an error. */
  void SendError(enum ComsErrorTypeT code,uint8_t originalPacketType,uint8_t data);


  /* Broadcast new parameter value.
   * This will send the update message over the appropriate channels, and will likely broadcast
   * to all connected devices, so shouldn't be called frequently.
   * */
  void SendParamUpdate(enum ComsParameterIndexT paramIndex);

  /* Broadcast given parameter value.
   * This will send the update message over the appropriate channels, and will likely broadcast
   * to all connected devices, so shouldn't be called frequently.
   * */
  void SendParamData(enum ComsParameterIndexT paramIndex,const void *data,int len);

  /* Broadcast our device id.
   *  This will also announce our id with device 0 if it not set.
   *  */
  void SendAnnounceId(void);

  /* Count of USB messages dropped due to full buffers */
  extern int g_usbDropCount;

  /* Error encountered processing USB packets. */
  extern int g_usbErrorCount;

  /* True if this device is in bridge mode, relaying CAN messages back over the local USB connection. */
  extern bool g_canBridgeMode;



#ifdef __cplusplus
}
#endif

#endif
