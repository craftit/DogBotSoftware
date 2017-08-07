
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
#if 0
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
#else
uint16_t g_defaultPhaseAngles[12][3] = {
{2123,2290,2409},
{1997,2183,2409},
{1980,2079,2373},
{1972,1970,2223},
{1978,1890,2069},
{2102,1936,2013},
{2229,2008,1993},
{2367,2139,2001},
{2381,2233,2039},
{2391,2340,2206},
{2384,2412,2357},
{2265,2365,2397}
};
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
