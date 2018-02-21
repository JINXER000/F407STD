#ifndef __OUT6050_INTERRUPT_H__
#define __OUT6050_INTERRUPT_H__
#include "sys.h"
#include "delay.h"
#include "usart.h"


void outMPU6050_IntConfiguration(void);
extern uint8_t outisMPU6050_is_DRY;
#endif
