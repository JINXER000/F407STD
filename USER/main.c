#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "imu.h"
#include "niming.h"
#include "DMA.h"
#include "Driver_ESP8266.h"

uint32_t system_micrsecond;   //系统时间 单位ms
extern volatile float angle[3];


	
int main(void)
{ 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     
	uart_init(115200);		//初始化串口波特率为500000

	delay_init(168);		  //初始化延时函数
//	LED_Init();		        //初始化LED端口
	TIM2_Configuration();		
	MPU6050_Initialize(); 
	MPU6050_IntConfiguration();     
	MPU6050_EnableInt();  
	
//	   BSP_DMA_InitConfig();
//    ESP8266_InitConfig();


	system_micrsecond = Get_Time_Micros();				
	
  while(1)
	{
		IMU_getYawPitchRoll(angle);
		printf("yaw=%f;yaw=%f;roll=%f/n",angle[0],angle[1],angle[2]);
		usart1_report_imu(angle[0],angle[1],angle[2],0,0,0,0,0,0);
	 }
}

	

