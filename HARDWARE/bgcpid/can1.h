#ifndef __CAN1_H__
#define __CAN1_H__
#include "sys.h"

void CAN1_Configuration(void);
void GYRO_RST(void);
u8 CAN1_Receive_Msg(u8 *buf);
u8 CAN1_Send_Msg(u8* msg,u8 len);

void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq);

#endif 
