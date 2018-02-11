#ifndef __GRADU_MOTOR_H
#define __GRADU_MOTOR_H

#include "sys.h"
#include "global_math.h"

#define MOTOR_BASIC_SPEED	((uint16_t)(60))	//unit: °/s
#define MOTOR_RUN_LIMIT		((uint16_t)(5))		//unit: °/s
#define TIM_PERIOD 4999			//duty_5ms
typedef enum {mdir_anticlk = 0, mdir_clkwise} mdir_t;
const static int8_t mdir_cal_factor[] = {-1, 1};

#define SINE_ARRAY_MAX_LEN 2048
static uint16_t pwmSin[SINE_ARRAY_MAX_LEN];

static uint16_t sineArraySize;
static uint16_t phaseShift;

typedef struct
{
	uint16_t stepA;
	uint16_t stepB;
	uint16_t stepC;
}mphase_t;
static mphase_t mPitch, mRoll, mYaw;

void Motor_Init(void);
void SineArray_Init(void);
void MotorPWM_Init(void);
void MotorPos_Init(void);
void Motor0_Run(mdir_t mdir, uint16_t speed);
void Motor0_SetPWM(void);
void Motor0_Shutdown(void);

#endif
