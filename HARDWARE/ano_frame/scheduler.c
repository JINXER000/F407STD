/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：scheduler.c
 * 描述    ：任务调度
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "scheduler.h"
#include "time.h"
#include "mpu6050_interrupt.h"
#include "mpu6050_driver.h"
#include "mpu6050_i2c.h"
#include "imu.h"
#include "niming.h"
#include "svpwm.h"
#include "gradu_motor.h"
#include "pwm.h"
s16 loop_cnt;


loop_t loop;

int i;
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

//	ANO_DT_Data_Exchange();												//数传通信定时调用
}

float test[5];
void Duty_2ms()
{
	        isMPU6050_is_DRY = 1;   //mpu6050ÖÐ¶Ï±êÖ¾
        GetPitchYawGxGyGz();//¶ÁÈ¡×ËÌ¬Êý¾Ý,Êý¾ÝÒÑ¾­´¦Àí³ÉÁ¬Ðø·½Ê½	
		IMU_getYawPitchRoll(angle);
		printf("yaw=%f;yaw=%f;roll=%f/n",angle[0],angle[1],angle[2]);
		usart1_report_imu(angle[0],angle[1],angle[2],PWMC1,PWMC2,PWMC3,0,0,0);
	
	
	
//		setRollMotor(testPhase,(int)eepromConfig.rollPower);
	
	
}

void Duty_5ms()
{
	Motor0_Run(1,MOTOR_BASIC_SPEED);
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
	

