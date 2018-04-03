#ifndef __GUN_H__
#define __GUN_H__

#ifndef FRICTION_WHEEL
#define FRICTION_WHEEL
#endif 

#if defined(FRICTION_WHEEL)

#define PWM1  TIM5->CCR1
#define PWM2  TIM5->CCR2
#define PWM3  TIM4->CCR3
#define PWM4  TIM4->CCR4

#define InitFrictionWheel()     \
        PWM1 = 1000;             \
        PWM2 = 1000;
#define SetFrictionWheelSpeed(x) \
        PWM1 = x;                \
        PWM2 = x;

#endif 

void PWM_Configuration(void);
void dcmotorinit(void);

#endif /* __GUN_H__*/

