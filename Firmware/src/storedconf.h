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
  uint8_t otherJointId; // Obsolete
  uint8_t m_motionPositionReference;
  float m_relativePositionGain; // Obsolete
  float m_relativePositionOffset; // Obsolete
  float m_phaseResistance;
  float m_phaseOffsetVoltage;
  float m_phaseInductance;
  float m_velocityLimit; // Obsolete, this has now to be set by the control software.
  float m_absoluteMaxCurrent;
  float m_homeIndexPosition;
  float m_minSupplyVoltage;

  enum JointRoleT m_jointRole;
  bool m_endStopEnable;
  float m_endStopMin;
  float m_endStopStartBounce;
  float m_endStopMax;
  float m_endStopEndBounce;
  float m_endStopTargetBreakCurrent;
  float m_endStopMaxBreakCurrent;
  float m_jointInertia;
  enum SafetyModeT m_safetyMode;
  float m_supplyVoltageScale;
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
