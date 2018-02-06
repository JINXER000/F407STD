#ifndef PWM_H
#define PWM_H

#include "sys.h"

#define PWMA1 TIM1->CCR1
#define PWMA2 TIM1->CCR2
#define PWMA3 TIM1->CCR3

#define PWMC1 TIM3->CCR1
#define PWMC2 TIM3->CCR2
#define PWMC3 TIM3->CCR3

void TIM8_PWM_Init(int psc,int prd);
void TIM3_PWM_Init(int psc,int prd);

void pwmtest();
#endif

