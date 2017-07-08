#ifndef PWM_HEADER
#define PWM_HEADER 1

#include "ch.h"

int InitPWM(void);

int PWMRun(void);
int PWMStop(void);
int PWMCal(BaseSequentialStream *chp);
int PWMSVMScan(BaseSequentialStream *chp);

void PWMUpdateDrive(int phase,int power);

extern uint16_t *ReadADCs(void);

float hallToAngle(uint16_t *sensors);


#endif
