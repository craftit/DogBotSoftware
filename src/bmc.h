#ifndef BMC_HEADER
#define BMC_HEADER 1

#include <stdint.h>
#include "dogbot/protocol.h"

#define USE_PACKETUSB 1

#ifdef __cplusplus
extern "C" {
#endif

extern int ChangeControlState(enum ControlStateT newState);
extern void FaultDetected(enum FaultCodeT faultCode);

#ifdef __cplusplus
}
#endif

#endif
