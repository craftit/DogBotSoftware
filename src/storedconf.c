
#include "storedconf.h"
#include "eeprom.h"
#include <string.h>
#include "stm32f4xx_conf.h"

// EEPROM settings
#define EEPROM_BASE_GENERALCONF              1000

// Global variables
uint16_t VirtAddVarTab[NB_OF_VAR];

// Private variables
uint16_t g_defaultPhaseAngles[12][3] = {
 { 2465, 2399, 2079 },
 { 2471, 2255, 1865 },
 { 2418, 2043, 1819 },
 { 2186, 1839, 1790 },
 { 1930, 1710, 1786 },
 { 1833, 1754, 1981 },
 { 1820, 1833, 2204 },
 { 1821, 2020, 2430 },
 { 1878, 2215, 2460 },
 { 2145, 2409, 2487 },
 { 2397, 2528, 2499 },
 { 2464, 2479, 2323 }
};
#if 0
Cal 0 : 2804 2726 2368
Cal 1 : 2804 2556 2132
Cal 2 : 2743 2325 2087
Cal 3 : 2483 2105 2057
Cal 4 : 2203 1966 2060
Cal 5 : 2099 2016 2282
Cal 6 : 2086 2108 2527
Cal 7 : 2092 2322 2774
Cal 8 : 2165 2539 2806
Cal 9 : 2463 2749 2832
Cal 10 : 2736 2874 2840
Cal 11 : 2805 2819 2637

Cal 0 : 2814 2733 2364
Cal 1 : 2815 2559 2121
Cal 2 : 2750 2321 2072
Cal 3 : 2480 2093 2042
Cal 4 : 2190 1948 2044
Cal 5 : 2084 2001 2273
Cal 6 : 2074 2095 2525
Cal 7 : 2081 2317 2779
Cal 8 : 2156 2538 2812
Cal 9 : 2461 2754 2840
Cal 10 : 2738 2882 2846
Cal 11 : 2811 2826 2639
#endif

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
    for(int i = 0;i < 12;i++) {
      conf->phaseAngles[i][0] = g_defaultPhaseAngles[i][0];
      conf->phaseAngles[i][1] = g_defaultPhaseAngles[i][1];
      conf->phaseAngles[i][2] = g_defaultPhaseAngles[i][2];
    }
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
