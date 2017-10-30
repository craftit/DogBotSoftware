#ifndef STORED_CONF_HEADER
#define STORED_CONF_HEADER 1

#include "ch.h"
#include "pwm.h"
//#include "stm32f4xx_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

struct StoredConfigT {
  uint16_t configState;  // 0-Unconfigured 1-Ready
  uint16_t deviceId;
  uint16_t phaseAngles[g_calibrationPointCount][3];
  uint8_t otherJointId;
  uint8_t m_motionPositionReference;
  float m_relativePositionGain;
  float m_relativePositionOffset;
  float m_phaseResistance;
  float m_phaseOffsetVoltage;
  float m_phaseInductance;
  float m_velocityLimit;
};

void StoredConf_Init(void);
bool StoredConf_Load(struct StoredConfigT *conf);
bool StoredConf_Save(struct StoredConfigT *conf);

extern bool g_eeInitDone;
extern struct StoredConfigT g_storedConfig;

#ifdef __cplusplus
}
#endif

#endif
