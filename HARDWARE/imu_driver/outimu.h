#ifndef _OUTIMU_H_
#define _OUTIMU_H_
#include <math.h>
#include "out6050_interrupt.h"
#include "out6050_driver.h"
#include "out6050_i2c.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"

#define M_PI  (float)3.1415926535
	
void outInit_Quaternion(void);
void outIMU_getYawPitchRoll(volatile float * ypr); //������̬
void outGetPitchYawGxGyGz(void);
extern int16_t outMPU6050_FIFO[6][11];//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
extern int16_t outHMC5883_FIFO[3][11];//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ ע���Ŵ������Ĳ���Ƶ���������Ե����г�

extern volatile float outangle[3];
extern volatile float outyaw_angle,outpitch_angle,outroll_angle; //ʹ�õ��ĽǶ�ֵ

#endif

