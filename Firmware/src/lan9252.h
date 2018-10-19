#ifndef LAN9252_HEADER
#define LAN9252_HEADER 1

#include "bmc.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint16_t Lan9252ReadStatus(void);
extern uint32_t Lan9252ReadRegister32(uint16_t addr);
extern uint32_t Lan9252SetRegister32(uint16_t addr,uint32_t value);

extern enum FaultCodeT Lan9252Read(uint16_t addr,uint8_t *data,uint8_t len);
extern enum FaultCodeT Lan9252Write(uint16_t addr,const uint8_t *data,uint8_t len);

extern enum FaultCodeT Lan9252ReadCSR(uint16_t addr,uint8_t *data,uint8_t len);
extern enum FaultCodeT Lan9252WriteCSR(uint16_t addr,const uint8_t *data,uint8_t len);

extern uint16_t Lan9252Test(void);
extern enum FaultCodeT InitLan9252(void);

#ifdef __cplusplus
}
#endif

#endif
