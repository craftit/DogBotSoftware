#ifndef TLE5012B_HEADER
#define TLE5012B_HEADER 1

#include "bmc.h"

#ifdef __cplusplus
extern "C" {
#endif

extern enum FaultCodeT InitTLE5012B(void);

bool TLE5012ReadRegister(uint16_t address,uint16_t *value);
bool TLE5012ReadAngleFloat(float *angle); // Assigns angle in radians to 'angle'
bool TLE5012ReadAngleInt(int16_t *value);

#ifdef __cplusplus
}
#endif

#endif
