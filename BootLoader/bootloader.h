#ifndef BOOTLOADER_HEADER
#define BOOTLOADER_HEADER 1

#include "dogbot/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

  extern enum FaultCodeT g_lastFaultCode;
  extern enum ControlStateT g_controlState;
  extern uint32_t g_faultState;
  extern uint8_t g_indicatorState;
  extern bool g_canBridgeMode;

  void SendParamUpdate(enum ComsParameterIndexT paramIndex);
  bool SetParam(enum ComsParameterIndexT index,union BufferTypeT *dataBuff,int len);
  bool ReadParam(enum ComsParameterIndexT index,int *len,union BufferTypeT *data);

  bool SendBootLoaderResult(uint8_t lastSeqNum,enum BootLoaderStateT state,enum FlashOperationStatusT result);

  int InitCAN(void);

#ifdef __cplusplus
}
#endif

#endif
