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
    enum ComsPacketTypeT pktType,
    uint8_t deviceId,
    uint16_t position,
    uint16_t torque
    );

bool CANSendSetParam(
    uint8_t deviceId,
    uint16_t index,
    uint16_t data
    );

bool CANSendQueryDevices();

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

