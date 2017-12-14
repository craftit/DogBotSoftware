#ifndef BMC_HEADER
#define BMC_HEADER 1

#include "dogbot/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

bool ChangeControlState(enum ControlStateT newState);
extern void FaultDetected(enum FaultCodeT faultCode);

#ifdef __cplusplus
}
#endif

#endif
