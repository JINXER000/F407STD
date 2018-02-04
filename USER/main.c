#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "imu.h"
#include "niming.h"
#include "DMA.h"
#include "Driver_ESP8266.h"

uint32_t system_micrsecond;   //ϵͳʱ�� ��λms
extern volatile float angle[3];


	
int main(void)
{ 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ500000

	delay_init(168);		  //��ʼ����ʱ����
//	LED_Init();		        //��ʼ��LED�˿�
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

	

