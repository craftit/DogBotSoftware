#ifndef COMS_HEADER
#define COMS_HEADER 1

#ifdef __cplusplus
extern "C" {
#endif

  #include "usbcfg.h"

  void InitComs(void);

  bool SendPacket(BaseSequentialStream *chp,uint8_t *buff,int len);

  bool SendServoState(BaseSequentialStream *chp,float position,float torque);

  bool SendSync(BaseSequentialStream *chp);
  bool SendPing(BaseSequentialStream *chp);


#ifdef __cplusplus
}
#endif

#endif
