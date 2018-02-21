#ifndef __OUT6050_H__
#define __OUT6050_H__

#include "sys.h"
#include "delay.h"
#include "usart.h"


void IIC2_GPIO_Init(void);
int IIC2_WriteData(u8 dev_addr,u8 reg_addr,u8 data);
int IIC2_ReadData(u8 dev_addr,u8 reg_addr,u8 *pdata,u8 count);

#endif
