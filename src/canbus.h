#ifndef CANBUS_HEADER
#define CANBUS_HEADER 1

#ifdef __cplusplus
extern "C" {
#endif

int InitCAN(void);

// nodeID == 0 is broadcast.
bool CANSetAddress(CANTxFrame *txmsg,int nodeId,int packetType);

bool CANPing(
    enum ComsPacketTypeT pktType, // Must be either ping or pong
    uint8_t deviceId
    );

bool CANSendServo(
    uint8_t deviceId,
    uint16_t position,
    uint16_t torque,
    uint8_t mode
    );

bool CANSendServoReport(
    uint8_t deviceId,
    uint16_t position,
    int16_t torque,
    uint8_t state
    );


bool CANSendSetParam(
    uint8_t deviceId,
    uint16_t index,
    union BufferTypeT *data,
    int len
    );

bool CANSendReadParam(
    uint8_t deviceId,
    uint16_t index
    );

bool CANSendQueryDevices(void);

bool CANSendError(
    uint16_t errorCode,
    uint16_t data
    );

bool CANSendSetDevice(
    uint8_t deviceId,
    uint32_t uid0,
    uint32_t uid1
    );

extern uint32_t g_nodeUId[2];

#ifdef __cplusplus
}
#endif

#endif

