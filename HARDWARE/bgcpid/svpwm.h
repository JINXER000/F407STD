#ifndef SVPWM
#define SVPWM

#include "bgc32.h"

#define SINARRAYSIZE 1024
#define SINARRAYSCALE 32767

extern short int sinDataI16[];
void initSinArray(void);
float fastSin(float x);
void setPWMFastTable(int *pwm, float angle, float power);
void setPWMData(int *target, int *pwm);
void setRollMotor(float phi, int power);
#endif /* FASTTRIG_H_ */
