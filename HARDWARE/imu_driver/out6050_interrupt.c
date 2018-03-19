#include "out6050_interrupt.h"
#include "out6050_driver.h"
#include "out6050_i2c.h"
#include "outimu.h"
uint8_t outisMPU6050_is_DRY = 0;   // mpu6050 interrupt�жϱ�־

void outMPU6050_IntConfiguration(void)
{
	GPIO_InitTypeDef    gpio;
    NVIC_InitTypeDef    nvic;
    EXTI_InitTypeDef    exti;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);   
	gpio.GPIO_Pin = GPIO_Pin_0;			//0
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,GPIO_PinSource4); 
    exti.EXTI_Line = EXTI_Line0;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;//�½����ж�
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    
    nvic.NVIC_IRQChannel = EXTI0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

//void EXTI4_IRQHandler(void)         //�ж�Ƶ��1KHz
//{   
//    if(EXTI_GetITStatus(EXTI_Line4) != RESET)
//    {    
//        EXTI_ClearFlag(EXTI_Line4);          
//        EXTI_ClearITPendingBit(EXTI_Line4);
//        //��ȡԭʼ����
////        isMPU6050_is_DRY = 1;   //mpu6050�жϱ�־
////        GetPitchYawGxGyGz();//��ȡ��̬����,�����Ѿ�������������ʽ	
//    }
//}
