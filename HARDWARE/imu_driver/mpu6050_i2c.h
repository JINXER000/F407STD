#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "sys.h"
#include "delay.h"
#include "usart.h"


void IIC_GPIO_Init(void);
int IIC_WriteData(u8 dev_addr,u8 reg_addr,u8 data);
int IIC_ReadData(u8 dev_addr,u8 reg_addr,u8 *pdata,u8 count);
void HEAT_Configuration(void);

#endif
