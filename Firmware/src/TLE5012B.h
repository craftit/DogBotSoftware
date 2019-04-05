#ifndef TLE5012B_HEADER
#define TLE5012B_HEADER 1

#include "bmc.h"

#ifdef __cplusplus
extern "C" {
#endif

extern enum FaultCodeT InitTLE5012B(void);

uint16_t TLE5012ReadRegister(uint16_t address);

float TLE5012ReadAngleFloat(void);
int16_t TLE5012ReadAngleInt(void);

#ifdef __cplusplus
}
#endif

#endif
