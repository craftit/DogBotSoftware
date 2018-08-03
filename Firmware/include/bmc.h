#ifndef BMC_HEADER
#define BMC_HEADER 1

#include <stdint.h>
#include "dogbot/protocol.h"

#define USE_PACKETUSB 1

#ifdef __cplusplus
extern "C" {
#endif


extern int ChangeControlState(enum ControlStateT newState,enum StateChangeSourceT changeSource);
extern void FaultDetected(enum FaultCodeT faultCode);
extern void FaultWarning(enum FaultCodeT faultCode);

extern uint16_t *ReadADCs(void);
extern float ReadSupplyVoltage(void); // Read supply voltage from ADC.
extern float ReadDriveTemperature(void); // Read driver temp
extern float ReadMotorTemperature(void); // Read driver temp
extern float Read5VRailVoltage(void);

extern void InitDrv8503(void);
extern void RunTerminal(void);
extern void InitUSB(void);
extern void InitADC(void);
extern void StopADC(void);

extern void EmergencyStopReceivedSafeFlag(int fromDeviceId);

extern unsigned g_mainLoopTimeoutCount;
extern enum FanModeT g_fanMode;
extern float g_fanTemperatureThreshold;
extern enum FaultCodeT g_lastFaultCode;

#ifdef __cplusplus
}
#endif

#endif
