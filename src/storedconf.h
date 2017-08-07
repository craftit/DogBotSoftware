#ifndef STORED_CONF_HEADER
#define STORED_CONF_HEADER 1

#include "ch.h"
//#include "stm32f4xx_conf.h"

struct StoredConfigT {
  uint16_t configState;  // 0-Unconfigured 1-Ready
  uint16_t controllerId;
  uint16_t phaseAngles[12][3];

};

void StoredConf_Init(void);
bool StoredConf_Load(struct StoredConfigT *conf);
bool StoredConf_Save(struct StoredConfigT *conf);

extern bool g_eeInitDone;
extern struct StoredConfigT g_storedConfig;

#endif
