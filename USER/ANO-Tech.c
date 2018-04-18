 
#include "usart.h"
#include "anousart.h"
#include "ANO-Tech.h"
#include "ospid.h"
#include "cali.h"

#define CH_NUM 				(8) 	//chanel num of remote control

#define ANO_U2
// Note: v2.6 is more stable than v4.6
/////////////////////////////////////////////////////////////////////////////////////
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t anoflag;					//?????????
u8 data_to_send[50];	//??????
u8 checkdata_to_send,checksum_to_send;
extern  PID_Type PitchOPID,PitchIPID,RollIPID,RollOPID,AnglePID,YawIPID,YawOPID  ;

/**
  **********************************
  * @brief  ����1����1���ֽ�
  * @param  c����������
  * @retval None
  **********************************
*/
void usart1_send_char(u8 c)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); //ѭ������,ֱ���������   
	USART_SendData(USART1, c);
}

/**
  **********************************
  * @brief  �������ݸ�����������λ�����(V2.6�汾)
  * @param  fun:������. 0XA0~0XAF  
						data:���ݻ�����,���28�ֽ�!!
						len:data����Ч���ݸ���
  * @retval None
  **********************************
*/
void usart1_niming_report(u8 fun, u8*data, u8 len)
{
	u8 send_buf[32];
	u8 i;
	if (len>28)return;	//���28�ֽ����� 
	send_buf[len + 3] = 0;	//У��������
	send_buf[0] = 0X88;	//֡ͷ
	send_buf[1] = fun;	//������
	send_buf[2] = len;	//���ݳ���
	for (i = 0;i<len;i++)send_buf[3 + i] = data[i];			//��������
	for (i = 0;i<len + 3;i++)send_buf[len + 3] += send_buf[i];	//����У���	
	for (i = 0;i<len + 4;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
}

/**
  **********************************
  * @brief  ���ͼ��ٶȴ��������ݺ�����������
  * @param  aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ  
						gyrox,gyroy,gyroz:x,y,z�������������������ֵ
  * @retval None
  **********************************
*/
void mpu6050_send_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
	u8 tbuf[12];
	tbuf[0] = (aacx >> 8) & 0XFF;
	tbuf[1] = aacx & 0XFF;
	tbuf[2] = (aacy >> 8) & 0XFF;
	tbuf[3] = aacy & 0XFF;
	tbuf[4] = (aacz >> 8) & 0XFF;
	tbuf[5] = aacz & 0XFF;
	tbuf[6] = (gyrox >> 8) & 0XFF;
	tbuf[7] = gyrox & 0XFF;
	tbuf[8] = (gyroy >> 8) & 0XFF;
	tbuf[9] = gyroy & 0XFF;
	tbuf[10] = (gyroz >> 8) & 0XFF;
	tbuf[11] = gyroz & 0XFF;
	usart1_niming_report(0XA1, tbuf, 12);//�Զ���֡,0XA1
}

/**
  **********************************
  * @brief  ͨ������1�ϱ���������̬���ݸ�����
  * @param  aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
						gyrox,gyroy,gyroz:x,y,z�������������������ֵ
						roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
						pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
						yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
  * @retval None
  **********************************
*/
void usart1_report_imu(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
	u8 tbuf[28];
	u8 i;
	for (i = 0;i<28;i++)tbuf[i] = 0;//��0
	tbuf[0] = (aacx >> 8) & 0XFF;
	tbuf[1] = aacx & 0XFF;
	tbuf[2] = (aacy >> 8) & 0XFF;
	tbuf[3] = aacy & 0XFF;
	tbuf[4] = (aacz >> 8) & 0XFF;
	tbuf[5] = aacz & 0XFF;
	tbuf[6] = (gyrox >> 8) & 0XFF;
	tbuf[7] = gyrox & 0XFF;
	tbuf[8] = (gyroy >> 8) & 0XFF;
	tbuf[9] = gyroy & 0XFF;
	tbuf[10] = (gyroz >> 8) & 0XFF;
	tbuf[11] = gyroz & 0XFF;
	tbuf[18] = (roll >> 8) & 0XFF;
	tbuf[19] = roll & 0XFF;
	tbuf[20] = (pitch >> 8) & 0XFF;
	tbuf[21] = pitch & 0XFF;
	tbuf[22] = (yaw >> 8) & 0XFF;
	tbuf[23] = yaw & 0XFF;
	usart1_niming_report(0XAF, tbuf, 28);//�ɿ���ʾ֡,0XAF
}

/**
  **********************************
  * @brief  �������ݸ���������վ���(V4.0�汾)
  * @param  data:���ݻ�����,���28�ֽ�!!
						len:data����Ч���ݸ���
  * @retval None
  **********************************
*/
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
#if defined ANO_U1
	u8 i;
	if (length>28)return;	//���28�ֽ����� 
	for (i = 0;i<length;i++)usart1_send_char(dataToSend[i]);	//�������ݵ�����1

#elif defined ANO_U2
	Usart2_Send(data_to_send, length);
#elif defined ANO_U4
	Uart4_Send(data_to_send, length);

#endif
}

static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

static void ANO_DT_Send_Msg(u8 id, u8 data)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEE;
	data_to_send[3]=2;
	data_to_send[4]=id;
	data_to_send[5]=data;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}
void ANO_DT_Data_Exchange(void)
{
	static u8 cnt = 0;
	static u8 senser_cnt 	= 10;
	static u8 senser2_cnt = 50;
	static u8 user_cnt 	  = 10;
	static u8 status_cnt 	= 15;
	static u8 rcdata_cnt 	= 20;
	static u8 motopwm_cnt	= 20;
	static u8 power_cnt		=	50;
	static u8 speed_cnt   = 50;
	static u8 location_cnt   = 200;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		anoflag.send_senser = 1;

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		anoflag.send_senser2 = 1;	

	if((cnt % user_cnt) == (user_cnt-2))
		anoflag.send_user = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))
		anoflag.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		anoflag.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-2))
		anoflag.send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-2))
		anoflag.send_power = 1;		
	
	if((cnt % speed_cnt) == (speed_cnt-3))
		anoflag.send_speed = 1;		
	
	if((cnt % location_cnt) == (location_cnt-3))
	{
		anoflag.send_location += 1;		
	}
	
	if(++cnt>200) cnt = 0;
/////////////////////////////////////////////////////////////////////////////////////
	if(anoflag.msg_id)
	{
		ANO_DT_Send_Msg(anoflag.msg_id,anoflag.msg_data);
		anoflag.msg_id = 0;
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(anoflag.send_check)
	{
		anoflag.send_check = 0;
		ANO_DT_Send_Check(checkdata_to_send,checksum_to_send);
	}
/////////////////////////////////////////////////////////////////////////////////////
//	else if(anoflag.send_version)
//	{
//		anoflag.send_version = 0;
//		ANO_DT_Send_Version(4,300,100,400,0);
//	}
///////////////////////////////////////////////////////////////////////////////////////
//	else if(anoflag.send_status)
//	{
//		anoflag.send_status = 0;
//		ANO_DT_Send_Status(Roll,Pitch,Yaw,(0.1f *baro_fusion.fusion_displacement.out),0,fly_ready);	
//	}	
///////////////////////////////////////////////////////////////////////////////////////
//	else if(anoflag.send_speed)
//	{
//		anoflag.send_speed = 0;
//		ANO_DT_Send_Speed(0,0,wz_speed);
//	}
///////////////////////////////////////////////////////////////////////////////////////
//	else if(anoflag.send_user)
//	{
//		anoflag.send_user = 0;
//		ANO_DT_Send_User();
//	}
///////////////////////////////////////////////////////////////////////////////////////
//	else if(anoflag.send_senser)
//	{
//		anoflag.send_senser = 0;
//		ANO_DT_Send_Senser(mpu6050.Acc.x,mpu6050.Acc.y,mpu6050.Acc.z,
//												mpu6050.Gyro.x,mpu6050.Gyro.y,mpu6050.Gyro.z,
//												ak8975.Mag_Val.x,ak8975.Mag_Val.y,ak8975.Mag_Val.z);
//	}	
///////////////////////////////////////////////////////////////////////////////////////
//	else if(anoflag.send_senser2)
//	{
//		anoflag.send_senser2 = 0;
//		ANO_DT_Send_Senser2(baro_p.displacement/10,ultra.height);//????
//	}	
///////////////////////////////////////////////////////////////////////////////////////
//	else if(anoflag.send_rcdata)
//	{
//		anoflag.send_rcdata = 0;
//		ANO_DT_Send_RCData(CH[2]+1500,CH[3]+1500,CH[0]+1500,CH[1]+1500,CH[4]+1500,CH[5]+1500,CH[6]+1500,CH[7]+1500,0 +1500,0 +1500);
//	}	
///////////////////////////////////////////////////////////////////////////////////////	
//	else if(anoflag.send_motopwm)
//	{
//		anoflag.send_motopwm = 0;
//		ANO_DT_Send_MotoPWM(motor[0],motor[1],motor[2],motor[3],0,0,0,0);
//	}	
///////////////////////////////////////////////////////////////////////////////////////
//	else if(anoflag.send_power)
//	{
//		anoflag.send_power = 0;
//		ANO_DT_Send_Power(123,456);
//	}
///////////////////////////////////////////////////////////////////////////////////////
//	else if(anoflag.send_pid1)
//	{
//		anoflag.send_pid1 = 0;
//		ANO_DT_Send_PID(1,ctrl_1.PID[PIDROLL].kp,ctrl_1.PID[PIDROLL].ki,ctrl_1.PID[PIDROLL].kd,
//											ctrl_1.PID[PIDPITCH].kp,ctrl_1.PID[PIDPITCH].ki,ctrl_1.PID[PIDPITCH].kd,
//											ctrl_1.PID[PIDYAW].kp,ctrl_1.PID[PIDYAW].ki,ctrl_1.PID[PIDYAW].kd);
//	}	
///////////////////////////////////////////////////////////////////////////////////////
//	else if(anoflag.send_pid2)
//	{
//		anoflag.send_pid2 = 0;
//		ANO_DT_Send_PID(2,ctrl_2.PID[PIDROLL].kp,ctrl_2.PID[PIDROLL].ki,ctrl_2.PID[PIDROLL].kd,
//											ctrl_2.PID[PIDPITCH].kp,ctrl_2.PID[PIDPITCH].ki,ctrl_2.PID[PIDPITCH].kd,
//											ctrl_2.PID[PIDYAW].kp,ctrl_2.PID[PIDYAW].ki,ctrl_2.PID[PIDYAW].kd);
//	}
///////////////////////////////////////////////////////////////////////////////////////
//	else if(anoflag.send_pid3)
//	{
//		anoflag.send_pid3 = 0;
//		ANO_DT_Send_PID(3,pid_setup.groups.hc_sp.kp,pid_setup.groups.hc_sp.ki,pid_setup.groups.hc_sp.kd,
//											pid_setup.groups.hc_height.kp,pid_setup.groups.hc_height.ki,pid_setup.groups.hc_height.kd,
//											pid_setup.groups.ctrl3.kp,pid_setup.groups.ctrl3.ki,pid_setup.groups.ctrl3.kd);
//	}
//	else if(anoflag.send_pid4)
//	{
//		anoflag.send_pid4 = 0;
//		ANO_DT_Send_PID(4,pid_setup.groups.ctrl4.kp,pid_setup.groups.ctrl4.ki,pid_setup.groups.ctrl4.kd,
//											0						,0						,0						,
//											0						,0						,0						);
//	}
//	else if(anoflag.send_pid5)
//	{
//		anoflag.send_pid5 = 0;
//		ANO_DT_Send_PID(5,0,0,0,0,0,0,0,0,0);
//	}
//	else if(anoflag.send_pid6)
//	{
//		anoflag.send_pid6 = 0;
//		ANO_DT_Send_PID(6,0,0,0,0,0,0,0,0,0);
//	}
//	else if(anoflag.send_location == 2)
//	{
//		
//		anoflag.send_location = 0;
//		ANO_DT_Send_Location(0,0,0 *10000000,0  *10000000,0);
//		
//	}
}

/**
  **********************************
  * @brief  ���ͼ��ٶȴ��������ݺ�����������
  * @param  aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ  
						gyrox,gyroy,gyroz:x,y,z�������������������ֵ
  * @retval None
  **********************************
*/
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser2(s32 bar_alt,u16 csb_alt)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE1(csb_alt);
	data_to_send[_cnt++]=BYTE0(csb_alt);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


/////////////////////////////////////////////////////////////////////////////////////
 u8 RxBuffer[50];
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
u16 flash_save_en_cnt = 0;
u16 RX_CH[CH_NUM];

void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		// sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//head
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{
//			mpu6050.Acc_CALIBRATE = 1;
			//mpu6050.Cali_3d = 1;
		}
//		else if(*(data_buf+4)==0X02)
//			mpu6050.Gyro_CALIBRATE = 1;
		else if(*(data_buf+4)==0X03)
		{
//			mpu6050.Acc_CALIBRATE = 1;		
//			mpu6050.Gyro_CALIBRATE = 1;			
		}
		else if(*(data_buf+4)==0X04)
		{
//			Mag_CALIBRATED = 1;
		}
		else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26))
		{
			//acc_3d_calibrate_f = 1;
		}
		else if(*(data_buf+4)==0X20)
		{
			//acc_3d_step = 0; //??,6?????0
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			anoflag.send_pid1 = 1;
			anoflag.send_pid2 = 1;
			anoflag.send_pid3 = 1;
			anoflag.send_pid4 = 1;
			anoflag.send_pid5 = 1;
			anoflag.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//READ VERSION
		{
			anoflag.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//reset to default
		{
//			Para_ResetToFactorySetup();
		}
	}

	if(*(data_buf+2)==0X03)			//MODIFY these block and control the car by ano-base
	{
//		if( NS != 1 )
//		{
//			Feed_Rc_Dog(2);
//		}

//		RX_CH[THR] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ;
//		RX_CH[YAW] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;
//		RX_CH[ROL] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;
//		RX_CH[PIT] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ;
//		RX_CH[AUX1] = (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
//		RX_CH[AUX2] = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
//		RX_CH[AUX3] = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;
//		RX_CH[AUX4] = (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;
	}

	if(*(data_buf+2)==0X10)								//PID1
    {
        PitchOPID.P    = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PitchOPID.I  = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PitchOPID.D  = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        PitchIPID.P= ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PitchIPID.I = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        PitchIPID.D = ( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        ctrl_1.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        ctrl_1.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        ctrl_1.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
				if(anoflag.send_check == 0)
				{
					anoflag.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
				mysetpid(&PitchOPID);
				mysetpid(&PitchIPID);
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
        RollOPID.P  = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        RollOPID.I  = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        RollOPID.D  = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        RollIPID.P = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        RollIPID.I = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        RollIPID.D = ( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        ctrl_2.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        ctrl_2.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        ctrl_2.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        if(anoflag.send_check == 0)
				{
					anoflag.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
				mysetpid(&RollOPID);
				mysetpid(&RollIPID);
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
        YawOPID.P  = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        YawOPID.I  = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        YawOPID.D  = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			
        YawIPID.P = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        YawIPID.I = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
       YawIPID.D = ( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
			
//        pid_setup.groups.ctrl3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        pid_setup.groups.ctrl3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        pid_setup.groups.ctrl3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        if(anoflag.send_check == 0)
				{
					anoflag.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
				mysetpid(&YawOPID);
				mysetpid(&YawIPID);
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
//		    pid_setup.groups.ctrl4.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//        pid_setup.groups.ctrl4.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        pid_setup.groups.ctrl4.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			
//         pid_setup.groups.hc_height.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//         pid_setup.groups.hc_height.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//         pid_setup.groups.hc_height.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
// 			
//         pid_setup.groups.ctrl3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//         pid_setup.groups.ctrl3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//         pid_setup.groups.ctrl3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(anoflag.send_check == 0)
		{
			anoflag.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
		PIDinitconfig();
		flash_save_en_cnt = 1;
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		if(anoflag.send_check == 0)
		{
			anoflag.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		if(anoflag.send_check == 0)
		{
			anoflag.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
	}
}
