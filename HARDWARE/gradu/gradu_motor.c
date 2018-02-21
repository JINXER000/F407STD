#include "gradu_motor.h"
#include "pwm.h"
void Motor_Init(void)
{
	SineArray_Init();
	MotorPWM_Init();
	MotorPos_Init();
}

void SineArray_Init(void)
{
	uint16_t cntL = 0;
	sineArraySize = 1000 / SYSTEM_PERIOD / (MOTOR_BASIC_SPEED/360.0);//360.0 avoid divisor become zero    6sec,1round
	phaseShift = sineArraySize / 3;

	mPitch.stepA = 0;
	mPitch.stepB = mPitch.stepA + phaseShift;
	mPitch.stepC = mPitch.stepB + phaseShift;
	mRoll.stepA = 0;
	mRoll.stepB = mRoll.stepA + phaseShift;
	mRoll.stepC = mRoll.stepB + phaseShift;
	mYaw.stepA = 0;
	mYaw.stepB = mYaw.stepA + phaseShift;
	mYaw.stepC = mYaw.stepB + phaseShift;

	//DEBUG_PRINT("%d %d %d %d %d\r\n", sineArraySize, phaseShift, mPitch.stepA, mPitch.stepB, mPitch.stepC);
	for(cntL = 0; cntL < sineArraySize; cntL++)
	{
		pwmSin[cntL] = (uint16_t)( ( sin( (cntL+1)*1.0/sineArraySize*2*MATH_PI )+1 )*TIM_PERIOD/2 );
		//DEBUG_PRINT("[%d]%d\r\n", cntL, pwmSin[cntL]);
	}
}

void MotorPWM_Init(void)
{

	/**
		motor0 pitch	pb1 -> tim3_ch4
						pb0 -> tim3_ch3
						pa7 -> tim3_ch2
	*/
	/**
		motor1 roll		pa6 -> tim3_ch1
						pa3 -> tim2_ch4
						pa2 -> tim2_ch3
	*/
	/**
		motor2 yaw		pb9 -> tim4_ch4
						pa1 -> tim2_ch2
						pb8 -> tim4_ch3
	*/
	TIM3_PWM_Init(167,TIM_PERIOD);

}

void MotorPos_Init(void)
{
	//Motor0 Pitch
	Motor0_SetPWM();
}

void Motor0_Run(mdir_t mdir, uint16_t speed)		//speed unit: °/s
{
	static uint16_t preSpeed = 0;
	static uint16_t timeout = 0;
	static uint16_t cntTime = 0;
	static uint16_t growthFactor = 0;
/**
	if(speed <= MOTOR_RUN_LIMIT)
	{
		speed = 0;
	}
*/
	if(speed == 0)
	{
		//keep
	}
	else
	{
		if(speed != preSpeed)
		{
			if(speed < MOTOR_BASIC_SPEED)
			{
				timeout = MOTOR_BASIC_SPEED / speed - 1;	//faster speed shorter timeout
				growthFactor = 1;
			}
			else
			{
				timeout = 0;
				growthFactor = speed / MOTOR_BASIC_SPEED;
			}
			preSpeed = speed;
		}
		if(cntTime < timeout)
		{
			//DEBUG_PRINT("[%d]%d %d\r\n", cntTime, timeout, speed);
			cntTime += 1;
		}
		else
		{
			cntTime = 0;	
			//TIM_Cmd(TIM3, DISABLE);
			//Motor0_Disable();
			Motor0_SetPWM();
			//Motor0_Enable();
			//TIM_Cmd(TIM3, ENABLE);
		
			//uint16_t has not negative so add sineArraySize
			mPitch.stepA = (mPitch.stepA + sineArraySize + mdir_cal_factor[mdir]*growthFactor) % sineArraySize;
			mPitch.stepB = (mPitch.stepB + sineArraySize + mdir_cal_factor[mdir]*growthFactor) % sineArraySize;
			mPitch.stepC = (mPitch.stepC + sineArraySize + mdir_cal_factor[mdir]*growthFactor) % sineArraySize;
			//DEBUG_PRINT("%d %d %d\r\n", pwmSin[mPitch.stepA], pwmSin[mPitch.stepB], pwmSin[mPitch.stepC]);
		}
	}
}

void Motor0_SetPWM(void)
{
	TIM_SetCompare1(TIM3, pwmSin[mPitch.stepA]);
	TIM_SetCompare2(TIM3, pwmSin[mPitch.stepB]);
	TIM_SetCompare3(TIM3, pwmSin[mPitch.stepC]);
}
void Motor0_Shutdown(void)
{
	TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM3, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Disable);
}
