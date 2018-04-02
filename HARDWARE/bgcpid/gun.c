#include "sys.h"
#include "gun.h"
/*-LEFT---(PB3---TIM2_CH2)--*/
/*-RIGHT--(PA15--TIM2_CH1)--*/

void PWM_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   //PCLK1=42MHz,TIM5 clk =84MHz

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&gpio);

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); 
    /* TIM5 */
    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2500;   //2.5ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&tim);
		
		
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 1000;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM5,&oc);
    TIM_OC2Init(TIM5,&oc);
    
    TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM5,ENABLE);
		
    TIM_Cmd(TIM5,ENABLE);
}

void dcmotorinit()
{
//	  GPIO_InitTypeDef          gpio;
//    TIM_TimeBaseInitTypeDef   tim;
//    TIM_OCInitTypeDef         oc;

//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   //PCLK1=42MHz,TIM5 clk =84MHz

//		gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 ;
//    gpio.GPIO_Mode = GPIO_Mode_AF;
//    gpio.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_Init(GPIOB,&gpio);

//	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10, GPIO_AF_TIM2);
//    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_TIM2); 

//		tim.TIM_Prescaler = 168-1;
//    tim.TIM_CounterMode = TIM_CounterMode_Up;
//    tim.TIM_Period = 1000;   //1ms,1KHz
//    tim.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseInit(TIM2,&tim);

//    oc.TIM_OCMode = TIM_OCMode_PWM1;
//    oc.TIM_OutputState = TIM_OutputState_Enable;
//    oc.TIM_OutputNState = TIM_OutputState_Disable;
//    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
//    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
//    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
//    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
//		oc.TIM_Pulse = 0;
//		TIM_OC3Init(TIM2,&oc);
//		TIM_OC4Init(TIM2,&oc);

//    TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
//		TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);

//    TIM_ARRPreloadConfig(TIM2,ENABLE);
//    TIM_Cmd(TIM2,ENABLE);
//		TIM_CtrlPWMOutputs(TIM2,ENABLE);			//important difference from normal tim!!!

}
