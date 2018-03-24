#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "imu.h"
#include "outimu.h"
#include "out6050_driver.h"
//#include "niming.h"
#include "ANO-Tech.h"

#include "DMA.h"
#include "Driver_ESP8266.h"
#include "pwm.h"
#include "bgc32.h"
#include "scheduler.h"
#include "time.h"
#include "svpwm.h"
#include "eecfg.h"
#include "gradu_motor.h"
#include "cali.h"
#include "anousart.h"

//rm
uint32_t system_micrsecond;   //系统时间 单位ms
extern volatile float angle[3];
//ano
u8 Init_Finish = 0;				//!! remmember to set 1 at the end of init
//bgc
float          testPhase      = 30.0f * D2R;
float          testPhaseDelta = 10.0f * D2R;



	
int main(void)
{ 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     
	uart_init(115200);		//初始化串口波特率为500000

	delay_init(168);		  //初始化延时函数
//	LED_Init();		        //初始化LED端口
	TIM2_Configuration();		
#if defined IMU_OB
	MPU6050_Initialize(); 
	MPU6050_IntConfiguration();     
	MPU6050_EnableInt();
#endif
#if defined IMU_EX
	outMPU6050_Initialize(); 
	outMPU6050_IntConfiguration();     
	outMPU6050_EnableInt();  
#endif
//  
//	TIM8_PWM_Init(167,5000);
#if defined WIFI
//	   BSP_DMA_InitConfig();
//    ESP8266_InitConfig();
#endif
	system_micrsecond = Get_Time_Micros();	
//	system_micrsecond=sysTickUptime;
	//ano frame
		SysTick_Configuration(); 	//
		Cycle_Time_Init();
		Init_Finish=1;
	//evvgc
//	checkFirstTime(1);
//	initSinArray();
//	pwmMotorDriverInit();
	//gradu
Motor_Init();	
enabledrv();
Usart2_Init(115200);
// get cali params
AppParamInit();
excallparaminit();



  while(1)
	{
//		IMU_getYawPitchRoll(angle);
//		printf("yaw=%f;yaw=%f;roll=%f/n",angle[0],angle[1],angle[2]);
//		usart1_report_imu(angle[0],angle[1],angle[2],0,0,0,0,0,0);
//		pwmtest();
		
				Duty_Loop(); 

	 }
}

	

