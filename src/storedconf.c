
#include "storedconf.h"
#include "eeprom.h"
#include <string.h>
#include "stm32f4xx_conf.h"

// EEPROM settings
#define EEPROM_BASE_GENERALCONF              1000


bool g_eeInitDone = false;
struct StoredConfigT g_storedConfig;


// Global variables
uint16_t VirtAddVarTab[NB_OF_VAR];

// Private variables
uint16_t g_defaultPhaseAngles[g_calibrationPointCount][3] = {
 { 2416, 2693, 2841 },
 { 2231, 2538, 2843 },
 { 2219, 2386, 2797 },
 { 2222, 2233, 2581 },
 { 2242, 2123, 2356 },
 { 2424, 2188, 2280 },
 { 2638, 2309, 2271 },
 { 2828, 2495, 2280 },
 { 2827, 2636, 2338 },
 { 2825, 2776, 2572 },
 { 2818, 2876, 2788 },
 { 2642, 2810, 2844 }
};



void StoredConf_Init(void)
{
  // First, make sure that all relevant virtual addresses are assigned for page swapping.
  memset(VirtAddVarTab, 0, sizeof(VirtAddVarTab));

  int ind = 0;
  for (unsigned int i = 0;i < (sizeof(struct StoredConfigT) / 2);i++) {
    VirtAddVarTab[ind++] = EEPROM_BASE_GENERALCONF + i;
  }

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                  FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  EE_Init();
}

bool StoredConf_Load(struct StoredConfigT *conf)
{
  bool is_ok = true;
  uint8_t *conf_addr = (uint8_t*)conf;
  uint16_t var;
  memset(conf,0,sizeof(struct StoredConfigT));
  for (unsigned int i = 0;i < (sizeof(struct StoredConfigT) / 2);i++) {
    if (EE_ReadVariable(EEPROM_BASE_GENERALCONF + i, &var) == 0) {
      conf_addr[2 * i] = (var >> 8) & 0xFF;
      conf_addr[2 * i + 1] = var & 0xFF;
    } else {
      is_ok = false;
      break;
    }
  }

  // Set the default configuration
  if (!is_ok) {
    memset(conf,0,sizeof(struct StoredConfigT));
    for(int i = 0;i < g_calibrationPointCount;i++) {
      conf->phaseAngles[i][0] = g_defaultPhaseAngles[i][0];
      conf->phaseAngles[i][1] = g_defaultPhaseAngles[i][1];
      conf->phaseAngles[i][2] = g_defaultPhaseAngles[i][2];
    }
    conf->deviceId = 0;
    conf->otherJointId = 0;
    conf->m_motionPositionReference = PR_Absolute;
    conf->m_relativePositionGain = 1.0;
    conf->m_relativePositionOffset = 0.0;
    conf->m_phaseResistance = 0.002;
    conf->m_phaseInductance = 1e-4;
    conf->m_phaseOffsetVoltage = 0.1;
    conf->m_velocityLimit = 100.0;
    conf->m_absoluteMaxCurrent = 20.0;
    conf->m_homeIndexPosition = 0.0;
    conf->m_minSupplyVoltage = 6.0;

    conf->m_jointRole = JR_Spare;
    conf->m_endStopEnable = false;
    conf->m_endStopStart = 0;
    conf->m_endStopStartBounce = 0;
    conf->m_endStopEnd = 0;
    conf->m_endStopEndBounce = 0;
    conf->m_endStopTargetBreakCurrent = 0;
    conf->m_endStopMaxBreakCurrent = 0;
    conf->m_jointInertia = 0.0;

    conf->m_safetyMode = SM_GlobalEmergencyStop;
  }

  return is_ok;
}

bool StoredConf_Save(struct StoredConfigT *conf)
{
  bool is_ok = true;
  uint8_t *conf_addr = (uint8_t*)conf;
  uint16_t var;

  FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                  FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  for (unsigned int i = 0;i < (sizeof(struct StoredConfigT) / 2);i++) {
    var = (conf_addr[2 * i] << 8) & 0xFF00;
    var |= conf_addr[2 * i + 1] & 0xFF;

    if (EE_WriteVariable(EEPROM_BASE_GENERALCONF + i, var) != FLASH_COMPLETE) {
      is_ok = false;
      break;
    }
  }

  return is_ok;
}
