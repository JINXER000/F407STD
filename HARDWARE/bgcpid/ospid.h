#ifndef __OSPID_H__
#define __OSPID_H__

#include "sys.h"

typedef struct
{
	float P;
	float I;
	float D;
	
	float CurrentError;
	float LastError;
	float NextError;
	float Pout;
	float Iout;
	float Dout;
	float PIDout;
	
	float IMax;
	float PIDMax;
	int index;
//	portTickType LastTick;
}PID_Type;

void PIDinitconfig(void);
int16_t Control_RollPID(void);
int16_t Control_PitchPID(void);
//int16_t Increment_PitchPID(int pwmx);
int16_t keepangle(void);

#endif
