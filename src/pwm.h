#ifndef PWM_HEADER
#define PWM_HEADER 1

#include "ch.h"

int InitPWM(void);

int PWMRun(void);
int PWMStop(void);
int PWMCal(BaseSequentialStream *chp);

void PWMUpdateDrive(int phase,int power);

extern adcsample_t *ReadADCs(void);

float hallToAngle(adcsample_t *sensors);


#endif
