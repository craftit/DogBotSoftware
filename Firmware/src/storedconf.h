#ifndef STORED_CONF_HEADER
#define STORED_CONF_HEADER 1

#include "ch.h"
#include "pwm.h"
//#include "stm32f4xx_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMC_STOREDCONFIG_VERSION 2

struct StoredConfigT {
  uint16_t configState;  // 0-Unconfigured otherwise configuration version
  uint16_t deviceId;
  uint8_t m_motionPositionReference;
  float m_phaseResistance;
  float m_phaseOffsetVoltage;
  float m_phaseInductance;
  float m_absoluteMaxCurrent;
  float m_homeIndexPosition;
  float m_minSupplyVoltage;

  float m_phaseEncoderZero;
  float m_phaseEncoderAngle;

  enum JointRoleT m_jointRole;
  bool m_endStopEnable;
  float m_endStopMin;
  float m_endStopMax;
  float m_endStopTargetBreakCurrent;
  float m_endStopMaxBreakCurrent;
  enum SafetyModeT m_safetyMode;
  float m_supplyVoltageScale;
  enum DeviceTypeT m_deviceType;

  uint16_t m_checksum; // Keep last!  This is the sum of all bytes before the checksum.
};

void StoredConf_Init(void);
bool StoredConf_Load(struct StoredConfigT *conf);
bool StoredConf_Save(struct StoredConfigT *conf);
void StoredConf_FactoryDefaults(struct StoredConfigT *conf);

extern struct StoredConfigT g_storedConfig;

#ifdef __cplusplus
}
#endif

#endif
