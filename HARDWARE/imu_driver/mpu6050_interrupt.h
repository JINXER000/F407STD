#ifndef __MPU6050_INTERRUPT_H__
#define __MPU6050_INTERRUPT_H__
#include "sys.h"
#include "delay.h"
#include "usart.h"


void MPU6050_IntConfiguration(void);
extern uint8_t isMPU6050_is_DRY;
#endif
