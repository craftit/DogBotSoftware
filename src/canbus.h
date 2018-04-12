#ifndef CANBUS_HEADER
#define CANBUS_HEADER 1

#include <stdint.h>
#include "dogbot/protocol.h"
#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif


int InitCAN(void);

// nodeID == 0 is broadcast.
bool CANSetAddress(CANTxFrame *txmsg,int nodeId,int packetType);

// Send an emergency stop
bool CANEmergencyStop(uint8_t deviceId,enum StateChangeSourceT eStopSource);

bool CANSyncTime(void);

bool CANPing(
    enum ComsPacketTypeT pktType, // Must be either ping or pong
    uint8_t deviceId,
    uint16_t payload
    );

bool CANSendCalZero(
    uint8_t deviceId
    );

bool CANSendServo(
    uint8_t deviceId,
    int16_t position,
    uint16_t torque,
    uint8_t mode
    );

bool CANSendServoReport(
    uint8_t deviceId,
    int16_t position,
    int16_t velocity,
    int16_t torque,
    uint8_t state,
    uint8_t timeStamp
    );


bool CANSendSetParam(
    uint8_t deviceId,
    uint16_t index,
    union BufferTypeT *data,
    int len
    );

bool CANSendParam(
    enum ComsParameterIndexT index
    );

bool CANSendReadParam(
    uint8_t deviceId,
    uint16_t index
    );

bool CANSendQueryDevices(void);

bool CANSendError(
    uint16_t errorCode,
    uint8_t causeType,
    uint8_t data
    );

bool CANSendSetDevice(
    uint8_t deviceId,
    uint32_t uid0,
    uint32_t uid1
    );

bool CANSendStoredSetup(
    uint8_t deviceId,
    enum ComsPacketTypeT pktType // Must be either CPT_SaveSetup or CPT_LoadSetup
);

bool CANSendAnnounceId(void);


bool CANSendBootLoaderReset(uint8_t deviceId,bool enable);
bool CANSendBootLoaderResult(uint8_t deviceId,uint8_t lastSeqNum,enum BootLoaderStateT state,enum FlashOperationStatusT result);
bool CANSendBootLoaderErase(uint8_t deviceId,uint8_t seqNum,uint32_t blockAddr);
bool CANSendBootLoaderData(uint8_t deviceId,uint8_t seqNum,uint8_t *data,uint8_t len);
bool CANSendBootLoaderRead(uint8_t deviceId,uint8_t seqNum,uint32_t addr,uint16_t len);
bool CANSendBootLoaderWrite(uint8_t deviceId,uint8_t seqNum,uint32_t addr,uint16_t len);
bool CANSendBootLoaderCheckSum(uint8_t deviceId,uint8_t seqNum,uint32_t addr,uint16_t len);
bool CANSendBootLoaderCheckSumResult(uint8_t deviceId,uint8_t seqNum,uint32_t sum);


/* The local device id. */
extern uint8_t g_deviceId;

/* Count of CAN messages dropped due to full buffers */
extern int g_canDropCount;

/* Count of CAN errors encountered */
extern int g_canErrorCount;

extern uint32_t g_nodeUId[2];

#ifdef __cplusplus
}
#endif

#endif

