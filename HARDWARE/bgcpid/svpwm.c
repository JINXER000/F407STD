#include "svpwm.h"


#define PWM_PERIOD 1000

#define MAX_CNT (PWM_PERIOD * 8 / 10)

#define BB_PERIPH_ADDR(addr, bit) ((vu32*)(PERIPH_BB_BASE + ((void*)(addr)-(void*)PERIPH_BASE) * 32 + (bit) * 4))

///////////////////////////////////////

static int pwmMotorDriverInitDone = false;

int timer1timer8deadTimeRegister = 200; // this is not just a delay value, check CPU reference manual for TIMx_BDTR DTG bit 0-7
int timer4timer5deadTimeDelay    = 80;  // in 18MHz ticks

static int rollPhase[3], pitchPhase[3], yawPhase[3];

int maxCnt[NUMAXIS];
int minCnt[NUMAXIS];
int irqCnt[NUMAXIS];

uint8_t oddEvenFrame = 0;



extern float          testPhase;

short int sinDataI16[SINARRAYSIZE];

void initSinArray(void)
{
    int i;

    for (i = 0; i < SINARRAYSIZE; i++)
    {
        float x = i * M_TWOPI / SINARRAYSIZE;
        sinDataI16[i] = (short int)round(sinf(x) * SINARRAYSCALE);
    }
}

float fastSin(float x)
{
    if (x >= 0)
    {
        int ix = ((int)(x / M_TWOPI * (float)SINARRAYSIZE)) % SINARRAYSIZE;
        return sinDataI16[ix] / (float)SINARRAYSCALE;
    }
    else
    {
        int ix = ((int)(-x / M_TWOPI * (float)SINARRAYSIZE)) % SINARRAYSIZE;
        return -sinDataI16[ix] / (float)SINARRAYSCALE;
    }
}

void setPWMFastTable(int *pwm, float angle, float power)
{
	 int iPower;
	int angleInt ;
    if (testPhase >= 0)
    {
        angle = testPhase;
    }

     angleInt = (int)round(angle / M_TWOPI * SINARRAYSIZE);

    angleInt = angleInt % SINARRAYSIZE;

    if (angleInt < 0)
    {
        angleInt = SINARRAYSIZE + angleInt;
    }

    //int iPower = 5 * (int)power;
     iPower = (int)((PWM_PERIOD / 2 - timer4timer5deadTimeDelay)  * power / 100);

    pwm[0] = (sinDataI16[ angleInt                               % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
    pwm[1] = (sinDataI16[(angleInt +  1 * SINARRAYSIZE / 3)      % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
    pwm[2] = (sinDataI16[(angleInt + (2 * SINARRAYSIZE + 1) / 3) % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
}
