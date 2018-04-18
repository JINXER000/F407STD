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
#include "ospid.h"
#include "usart1.h"
#include "gun.h"
#include "can1.h"
#include "can.h"
//rm
uint32_t system_micrsecond,presenttime;   //系统时间 单位ms
extern volatile float angle[3];
//ano
u8 Init_Finish = 0;				//!! remmember to set 1 at the end of init
//bgc
float          testPhase      = 30.0f * D2R;
float          testPhaseDelta = 10.0f * D2R;

	u8 mode=0;//CAN工作模式;0,普通模式;1,环回模式
	u8 canbuf[8];



	
int main(void)
{ 

	u8 i=0,t=0;
	u8 cnt=0;
	u8 res;
//
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
#if defined USE_ATKU1
	uart_init(115200);		//初始化串口波特率为500000
#elif defined USE_DBUS
	USART1_Configuration(100000);
#endif

Usart2_Init(115200);
	delay_init(168);		  //初始化延时函数
	
//............WIFI		
#if defined WIFI
//	   BSP_DMA_InitConfig();
//    ESP8266_InitConfig();
#endif
	

//...........PWM
Motor_Init();	
	TIM1_PWM_Init(167,2500);
	TIM4_PWM_Init(167,2500);
//	dcmotorinit();
//	TIM8_PWM_Init(167,5000);

enabledrv();
//........TIM & IMU
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
	system_micrsecond = Get_Time_Micros();	
//	system_micrsecond=sysTickUptime;
//...............ano frame
		SysTick_Configuration(); 	//
		Cycle_Time_Init();
		Init_Finish=1;

//Uart4_Init(115200);
// ............get cali params
AppParamInit();	//read flash to struct
excallparaminit();			//put struct to default para
PIDinitconfig();

CAN1_Configuration();
//	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN初始化环回模式,波特率500Kbps    

  while(1)
	{
//		IMU_getYawPitchRoll(angle);
//		printf("yaw=%f;yaw=%f;roll=%f/n",angle[0],angle[1],angle[2]);
//		usart1_report_imu(angle[0],angle[1],angle[2],0,0,0,0,0,0);
//		pwmtest();
		
				Duty_Loop(); 

			Set_Gimbal_Current(CAN1,1024, 2048);     //yaw + pitch
		
//					for(i=0;i<8;i++)
//			{
//				canbuf[i]=cnt+i;//填充发送缓冲区
// 			}
//			res=CAN1_Send_Msg(canbuf,8);//发送8个字节 

//		t++; 
//		delay_ms(10);
//		if(t==20)
//		{
//			t=0;
//			cnt++;
//		}		   
	} 

}

	

