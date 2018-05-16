#ifndef CAN_COMS_HEADER
#define CAN_COMS_HEADER 1

#include "hal.h"

#define CAN_MSG_TYPEBIT 6
#define CAN_MSG_NODE_MASK 0x3f
#define CAN_MSG_TYPEMASK 0x1f

bool CANRecieveFrame(CANRxFrame *rxmsg);

#endif
