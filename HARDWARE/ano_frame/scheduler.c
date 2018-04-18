#include "scheduler.h"
#include "time.h"
#include "mpu6050_interrupt.h"
#include "mpu6050_driver.h"
#include "mpu6050_i2c.h"
#include "imu.h"
#include "out6050_interrupt.h"
#include "out6050_driver.h"
#include "out6050_i2c.h"
#include "outimu.h"
#include "cali.h"
//#include "niming.h"
#include "ANO-Tech.h"

//#include "svpwm.h"
#include "gradu_motor.h"
#include "pwm.h"
#include "ospid.h"
#define ROLLRUN
//#define ROLLTEST
s16 loop_cnt;


loop_t loop;
extern volatile MPU6050_RAW_DATA    MPU6050_Raw_Data;
extern  float pitchgoal,rollgoal, pitchnow,rollnow;
extern  PID_Type PitchOPID,PitchIPID,RollIPID,RollOPID,AnglePID ,YawIPID,YawOPID ;

int i;

float pitchSpeed = 0.0,rollSpeed = 0.0;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
	
//		LED_1ms_DRV( );								//20级led渐变显示
}

void Duty_1ms()
{
//	Get_Cycle_T(1)/1000000.0f;

	ANO_DT_Data_Exchange();												//数传通信定时调用
}

float test[5],magrms;
void Duty_2ms()
{
#if defined IMU_OB

	  isMPU6050_is_DRY = 1;   //
    GetPitchYawGxGyGz();//
		IMU_getYawPitchRoll(angle);
#endif
#if defined IMU_EX

		outisMPU6050_is_DRY = 1;   //
    outGetPitchYawGxGyGz();//
		outIMU_getYawPitchRoll(outangle);
#endif

//		printf("yaw=%f;yaw=%f;roll=%f/n",angle[0],angle[1],angle[2]);
		magrms=pow(MPU6050_Raw_Data.Mag_X,2)+pow(MPU6050_Raw_Data.Mag_Y,2);
	
//		usart1_report_imu(angle[0],angle[1],magrms,PWMC1,PWMC2,PWMC3,outangle[0],outangle[1],outangle[2]);
	 ANO_DT_Send_Senser(PWMC1,angle[1],angle[2], RollIPID.CurrentError, RollIPID.PIDout,MPU6050_Real_Data.Gyro_X,outangle[0],outangle[1],outangle[2]);
//		 ANO_DT_Send_Senser(angle[0],angle[1],angle[2],0,0,0,0,outangle[1],outangle[2]);

		CalibrateLoop();
	
//		setRollMotor(testPhase,(int)eepromConfig.rollPower);
	
	
}

void Duty_5ms()
{
#if defined ROLLRUN
	rollgoal=0;
	rollnow=angle[2];
	rollSpeed=Control_RollPID();
	//0-angle[2]>0(实际偏左),speed>0，逆时针转
	//顺序：当接入线测量点朝外，speed<0,从前端看顺时针
	rollSpeed=INTERVAL_CONSTRAINT(rollSpeed,10*MOTOR_BASIC_SPEED,-10*MOTOR_BASIC_SPEED);//MAX=10*60
	Motor0_Run((mdir_t)(rollSpeed > 0), (uint16_t)(fabs(rollSpeed)));
#elif defined ROLLTEST	
	Motor0_Run(0,20*MOTOR_BASIC_SPEED);
#endif
	//pitchSpeed = PID_Motor0(Mpu6050_Pitch, 0.0);//#1 pitch
	//pitchSpeed = INTERVAL_CONSTRAINT(pitchSpeed, ANGLE_MAX_SPEED, ANGLE_MAX_SPEED*(-1));
  //Motor0_Run((mdir_t)(pitchSpeed > 0), (uint16_t)(fabs(pitchSpeed)));
}

void Duty_10ms()
{

		
}

void Duty_20ms()
{
}

void Duty_50ms()
{
}


void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{

	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//周期1ms的任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		
		loop.check_flag = 0;		//循环运行完毕标志
	}
}




	/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
	

