#include "pwm.h"
#include "bgc32.h"
#include "drv_irq.h"
// TIM8 Roll
// PC6, PC7, PC8 used for Roll,  TIM_OCPolarity_High
// PA7, PB0, PB1 used for RollN, TIM_OCPolarity_High

#define ROLL_A_GPIO   GPIOC
#define ROLL_A_PIN    GPIO_Pin_6
#define ROLL_B_GPIO   GPIOC
#define ROLL_B_PIN    GPIO_Pin_7
#define ROLL_C_GPIO   GPIOC
#define ROLL_C_PIN    GPIO_Pin_8

#define ROLL_AN_GPIO  GPIOA
#define ROLL_AN_PIN   GPIO_Pin_7
#define ROLL_BN_GPIO  GPIOB
#define ROLL_BN_PIN   GPIO_Pin_0
#define ROLL_CN_GPIO  GPIOB
#define ROLL_CN_PIN   GPIO_Pin_1
///////////////////////////////////////

#define PWM_PERIOD 1000

#define MAX_CNT (PWM_PERIOD * 8 / 10)

#define BB_PERIPH_ADDR(addr, bit) ((vu32*)(PERIPH_BB_BASE + ((void*)(addr)-(void*)PERIPH_BASE) * 32 + (bit) * 4))

///////////////////////////////////////

static int pwmMotorDriverInitDone = false;

extern int timer1timer8deadTimeRegister; // this is not just a delay value, check CPU reference manual for TIMx_BDTR DTG bit 0-7
extern int timer4timer5deadTimeDelay;  // in 18MHz ticks

static int rollPhase[3], pitchPhase[3], yawPhase[3];

extern int maxCnt[NUMAXIS];
extern int minCnt[NUMAXIS];
extern int irqCnt[NUMAXIS];

extern uint8_t oddEvenFrame;

/*inline */void updateCounter(uint8_t channel, int value)
{
    irqCnt[channel]++;

    if (value > maxCnt[channel])
    {
        maxCnt[channel] = value;
    }

    if (value < minCnt[channel])
    {
        minCnt[channel] = value;
    }
}

///////////////////////////////////////////////////////////////////////////////
//  IRQ Setup
///////////////////////////////////////////////////////////////////////////////

static void setupPWMIrq(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel                   = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //Preemption Priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}

///////////////////////////////////////////////////////////////////////////////
//  TIM8 IRQ Handler (ROLL)
///////////////////////////////////////////////////////////////////////////////

void TIM8_UP_IRQHandler(void) // roll axis
{
	unsigned short cnt;
    TIM8->SR &= ~TIM_SR_UIF; // clear UIF flag

    __disable_irq_nested();
      cnt = TIM8->CNT;
    updateCounter(ROLL, cnt);

    if (cnt < MAX_CNT)
    {
        // make sure there is enough time to make all changes
        if (eepromConfig.rollEnabled)
        {
            TIM8->CCR1 = rollPhase[0];
            TIM8->CCR2 = rollPhase[1];
            TIM8->CCR3 = rollPhase[2];
        }
        else
        {
            TIM8->CCR1 = 0;
            TIM8->CCR2 = 0;
            TIM8->CCR3 = 0;
        }

        TIM8->DIER &= ~TIM_DIER_UIE; // disable update interrupt
    }

    __enable_irq_nested();
}


static void timerChannelConfig(TIM_TypeDef *tim, TIM_OCInitTypeDef *OCInitStructure)
{
    TIM_OC1Init(tim, OCInitStructure);
    TIM_OC2Init(tim, OCInitStructure);
    TIM_OC3Init(tim, OCInitStructure);

    TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
}

///////////////////////////////////////////////////////////////////////////////
//  Advanced Timer Configuration
///////////////////////////////////////////////////////////////////////////////

static void timerPWMadvancedConfig(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef           TIM_OCInitStructure;
    TIM_BDTRInitTypeDef         TIM_BDTRInitStructure;

    //Time Base configuration
    TIM_TimeBaseInitStructure.TIM_Prescaler         = (4 - 1);                 // 72 Mhz / (3 + 1) = 18 MHz
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period            = PWM_PERIOD - 1;          // 18 Mhz / 1000 = 18 kHz
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(tim, &TIM_TimeBaseInitStructure);

    //Automatic Output enable, Break, dead time and lock configuration
    TIM_BDTRInitStructure.TIM_OSSRState       = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState       = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel       = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime        = timer1timer8deadTimeRegister;
    TIM_BDTRInitStructure.TIM_Break           = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity   = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

    TIM_BDTRConfig(tim, &TIM_BDTRInitStructure);

    //Configuration in PWM mode
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse        = 0;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    timerChannelConfig(tim, &TIM_OCInitStructure);
}

//void TIM8_PWM_Init(int psc,int prd)
//{
//		GPIO_InitTypeDef         GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef				 TIM8_OCInitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 , ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

//	//PWM1
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
//	
//	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	
//	TIM_TimeBaseStructure.TIM_Prescaler         = psc;
//	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_Period            = prd;               
//	TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
//	
//	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
//	TIM8_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
//	TIM8_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
//	TIM8_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//	TIM8_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;         
//	TIM8_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
//	TIM8_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
//	TIM8_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
//		TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
//	TIM_OC1Init(TIM8, &TIM8_OCInitStructure);  
//	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
//	TIM_OC2Init(TIM8, &TIM8_OCInitStructure);        
//	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
//	TIM_OC3Init(TIM8, &TIM8_OCInitStructure);  
//	TIM_Cmd(TIM8, ENABLE);

//}

void TIM3_PWM_Init(int psc,int prd)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef				 TIM3_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	//PWM1
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler         = psc;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = prd;               
	TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM3_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
	TIM3_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM3_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM3_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;         
	TIM3_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
	TIM3_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	TIM3_OCInitStructure.TIM_Pulse=0;
		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC1Init(TIM3, &TIM3_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3, &TIM3_OCInitStructure);        
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3, &TIM3_OCInitStructure);  
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3, &TIM3_OCInitStructure);  	
	TIM_Cmd(TIM3, ENABLE);
}


void pwmtest()
{
	PWMC1=500;
	PWMC2=1500;
	PWMC3=2000;
}

