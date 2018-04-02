#ifndef PWM_H
#define PWM_H

#include "sys.h"

#define PWMA1 TIM1->CCR1
#define PWMA2 TIM1->CCR2
#define PWMA3 TIM1->CCR3

#define PWMC1 TIM3->CCR1
#define PWMC2 TIM3->CCR2
#define PWMC3 TIM3->CCR3

/*inline */void updateCounter(uint8_t channel, int value);
static void setupPWMIrq(uint8_t irq);
static void timerChannelConfig(TIM_TypeDef *tim, TIM_OCInitTypeDef *OCInitStructure);
static void timerPWMadvancedConfig(TIM_TypeDef *tim);
void pwmMotorDriverInit(void);
void activateIRQ(TIM_TypeDef *tim);
void enabledrv(void);





//void TIM8_PWM_Init(int psc,int prd);
void TIM3_PWM_Init(int psc,int prd);
void TIM1_PWM_Init(int psc,int prd);
void TIM2_PWM_Init(int psc,int prd);

void pwmtest(void);
#endif

