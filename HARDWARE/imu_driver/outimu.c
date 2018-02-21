#include "out6050_interrupt.h"
#include "out6050_driver.h"
#include "out6050_i2c.h"
#include "outimu.h"
#include "timer.h"
#include "time.h"

volatile float outexInt, outeyInt, outezInt;  // ������
volatile float outq0 = 1.0f;
volatile float outq1 = 0.0f;
volatile float outq2 = 0.0f;
volatile float outq3 = 0.0f;

volatile float outmygetqval[9];	//���ڴ�Ŵ�����ת�����������
static volatile float outgx, outgy, outgz, outax, outay, outaz, outmx, outmy, outmz;   //��������ڴ��ļ���

static volatile float outq[4]; //����Ԫ��
volatile uint32_t outlastUpdate, outnow; // �������ڼ��� ��λ us
volatile float outangle[3] = {0};
volatile float outyaw_temp,outpitch_temp,outroll_temp;
volatile float outlast_yaw_temp,outlast_pitch_temp,outlast_pitch_temp;
volatile float outyaw_angle,outpitch_angle,outpitch_angle; //ʹ�õ��ĽǶ�ֵ

// Fast inverse square-root
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float outinvSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
float outinvSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void outInit_Quaternion
*��������:	 ��ʼ����Ԫ��
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/
//��ʼ��IMU����
#define BOARD_DOWN 1   //�������泯�°ڷ�

void outInit_Quaternion()//���ݲ������ݣ���ʼ��outq0,outq1,outq2.outq3���Ӷ��ӿ������ٶ�
{
	int16_t hx,hy,hz;
	outHMC58X3_getlastValues(&hx,&hy,&hz);
	#ifdef BOARD_DOWN
	if(hx<0 && hy <0)   //OK
	{
		if(fabs(hx/hy)>=1)
		{
			outq0 = -0.005;
			outq1 = -0.199;
			outq2 = 0.979;
			outq3 = -0.0089;
		}
		else
		{
			outq0 = -0.008;
			outq1 = -0.555;
			outq2 = 0.83;
			outq3 = -0.002;
		}
		
	}
	else if (hx<0 && hy > 0) //OK
	{
		if(fabs(hx/hy)>=1)   
		{
			outq0 = 0.005;
			outq1 = -0.199;
			outq2 = -0.978;
			outq3 = 0.012;
		}
		else
		{
			outq0 = 0.005;
			outq1 = -0.553;
			outq2 = -0.83;
			outq3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)   //OK
	{
		if(fabs(hx/hy)>=1)
		{
			outq0 = 0.0012;
			outq1 = -0.978;
			outq2 = -0.199;
			outq3 = -0.005;
		}
		else
		{
			outq0 = 0.0023;
			outq1 = -0.83;
			outq2 = -0.553;
			outq3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)     //OK
	{
		if(fabs(hx/hy)>=1)
		{
			outq0 = 0.0025;
			outq1 = 0.978;
			outq2 = -0.199;
			outq3 = 0.008;			
		}
		else
		{
			outq0 = 0.0025;
			outq1 = 0.83;
			outq2 = -0.56;
			outq3 = 0.0045;
		}		
	}
	#else
		if(hx<0 && hy <0)
	{
		if(fabs(hx/hy)>=1)
		{
			outq0 = 0.195;
			outq1 = -0.015;
			outq2 = 0.0043;
			outq3 = 0.979;
		}
		else
		{
			outq0 = 0.555;
			outq1 = -0.015;
			outq2 = 0.006;
			outq3 = 0.829;
		}
		
	}
	else if (hx<0 && hy > 0)
	{
		if(fabs(hx/hy)>=1)
		{
			outq0 = -0.193;
			outq1 = -0.009;
			outq2 = -0.006;
			outq3 = 0.979;
		}
		else
		{
			outq0 = -0.552;
			outq1 = -0.0048;
			outq2 = -0.0115;
			outq3 = 0.8313;
		}
		
	}
	else if (hx>0 && hy > 0)
	{
		if(fabs(hx/hy)>=1)
		{
			outq0 = -0.9785;
			outq1 = 0.008;
			outq2 = -0.02;
			outq3 = 0.195;
		}
		else
		{
			outq0 = -0.9828;
			outq1 = 0.002;
			outq2 = -0.0167;
			outq3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs(hx/hy)>=1)
		{
			outq0 = -0.979;
			outq1 = 0.0116;
			outq2 = -0.0167;
			outq3 = -0.195;			
		}
		else
		{
			outq0 = -0.83;
			outq1 = 0.014;
			outq2 = -0.012;
			outq3 = -0.556;
		}		
	}
	#endif
	
	//����hx hy hz���ж�outq��ֵ��ȡ�ĸ������ֵ���ƽ�����,��ʼֵ������ŷ����ת������Ԫ������õ�
	 
}


	


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void outIMU_getValues(volatile float * values)
*��������:	 ��ȡ���ٶ� ������ ������ �ĵ�ǰֵ  
��������� �������ŵ������׵�ַ
���ٶ�ֵ��ԭʼ���ݣ�-8192-+8192
���ٶ�ֵ��deg/s
������ֵ��ԭʼ����
���������û��
*******************************************************************************/
void outIMU_getValues(volatile float * values) {  
		int16_t accgyroval[6];
		int i;
	//��ȡ���ٶȺ������ǵĵ�ǰADC
  outMPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
	outMPU6050_Raw_Data.Accel_X = accgyroval[0];
	outMPU6050_Raw_Data.Accel_Y = accgyroval[1];
	outMPU6050_Raw_Data.Accel_Z = accgyroval[2];
	outMPU6050_Raw_Data.Gyro_X = accgyroval[3];
	outMPU6050_Raw_Data.Gyro_Y = accgyroval[4];
	outMPU6050_Raw_Data.Gyro_Z = accgyroval[5];
	
	
    for(i = 0; i<6; i++) {
      if(i < 3) {
        values[i] = (float) accgyroval[i];
      }
      else {
        values[i] = ((float) accgyroval[i]) / 32.8f; //ת�ɶ�ÿ��
		//�����Ѿ������̸ĳ��� 1000��ÿ��  32.8 ��Ӧ 1��ÿ��
      }
    }
    outHMC58X3_mgetValues(&values[6]);	//��ȡ�����Ƶ�ADCֵ
		outMPU6050_Raw_Data.Mag_X = values[6];
		outMPU6050_Raw_Data.Mag_Y = values[7];
		outMPU6050_Raw_Data.Mag_Z = values[8];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void outIMU_AHRSupdate
*��������:	 ����AHRS ������Ԫ�� 
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/
#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases
void outIMU_AHRSupdate(void) {
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez,halfT;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = outq0*outq0;
    float q0q1 = outq0*outq1;
    float q0q2 = outq0*outq2;
    float q0q3 = outq0*outq3;
    float q1q1 = outq1*outq1;
    float q1q2 = outq1*outq2;
    float q1q3 = outq1*outq3;
    float q2q2 = outq2*outq2;   
    float q2q3 = outq2*outq3;
    float q3q3 = outq3*outq3;   

    outgx = outmygetqval[3] * M_PI/180;
    outgy = outmygetqval[4] * M_PI/180;
    outgz = outmygetqval[5] * M_PI/180;
    outax = outmygetqval[0];
    outay = outmygetqval[1];
    outaz = outmygetqval[2];
    outmx = outmygetqval[6];
    outmy = outmygetqval[7];
    outmz = outmygetqval[8];		

    outnow = Get_Time_Micros();  //��ȡʱ�� ��λ��us   
//	outnow=sysTickUptime;
    if(outnow<outlastUpdate)
    {
    //halfT =  ((float)(outnow + (0xffffffff- outlastUpdate)) / 2000000.0f);   //  uint 0.5s
    }
    else	
    {
        halfT =  ((float)(outnow - outlastUpdate) / 2000000.0f);
    }
    outlastUpdate = outnow;	//����ʱ��
    //������ƽ�����㷨
    norm = outinvSqrt(outax*outax + outay*outay + outaz*outaz);       
    outax = outax * norm;
    outay = outay * norm;
    outaz = outaz * norm;
    //�ѼӼƵ���ά����ת�ɵ�λ������
    norm = outinvSqrt(outmx*outmx + outmy*outmy + outmz*outmz);          
    outmx = outmx * norm;
    outmy = outmy * norm;
    outmz = outmz * norm; 
    // compute reference direction of flux
    hx = 2.0f*outmx*(0.5f - q2q2 - q3q3) + 2.0f*outmy*(q1q2 - q0q3) + 2.0f*outmz*(q1q3 + q0q2);
    hy = 2.0f*outmx*(q1q2 + q0q3) + 2.0f*outmy*(0.5f - q1q1 - q3q3) + 2.0f*outmz*(q2q3 - q0q1);
    hz = 2.0f*outmx*(q1q3 - q0q2) + 2.0f*outmy*(q2q3 + q0q1) + 2.0f*outmz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (outay*vz - outaz*vy) + (outmy*wz - outmz*wy);
    ey = (outaz*vx - outax*vz) + (outmz*wx - outmx*wz);
    ez = (outax*vy - outay*vx) + (outmx*wy - outmy*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        outexInt = outexInt + ex * Ki * halfT;
        outeyInt = outeyInt + ey * Ki * halfT;	
        outezInt = outezInt + ez * Ki * halfT;
        // �ò���������PI����������ƫ
        outgx = outgx + Kp*ex + outexInt;
        outgy = outgy + Kp*ey + outeyInt;
        outgz = outgz + Kp*ez + outezInt;
    }
    // ��Ԫ��΢�ַ���
    tempq0 = outq0 + (-outq1*outgx - outq2*outgy - outq3*outgz)*halfT;
    tempq1 = outq1 + (outq0*outgx + outq2*outgz - outq3*outgy)*halfT;
    tempq2 = outq2 + (outq0*outgy - outq1*outgz + outq3*outgx)*halfT;
    tempq3 = outq3 + (outq0*outgz + outq1*outgy - outq2*outgx)*halfT;  

    // ��Ԫ���淶��
    norm = outinvSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    outq0 = tempq0 * norm;
    outq1 = tempq1 * norm;
    outq2 = tempq2 * norm;
    outq3 = tempq3 * norm;

}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void outIMU_getQ(float * outq)
*��������:	 ������Ԫ�� ���ص�ǰ����Ԫ����ֵ
��������� ��Ҫ�����Ԫ���������׵�ַ
���������û��
*******************************************************************************/

void outIMU_getQ(volatile float * outq) {

    outIMU_getValues(outmygetqval);	 //��ȡԭʼ����,���ٶȼƺʹ�������ԭʼֵ��������ת������deg/s
    outIMU_AHRSupdate();
    outq[0] = outq0; //���ص�ǰֵ
    outq[1] = outq1;
    outq[2] = outq2;
    outq[3] = outq3;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void outIMU_getYawPitchRoll(float * angles)
*��������:	 ������Ԫ�� ���ص�ǰ��������̬����
��������� ��Ҫ�����̬�ǵ������׵�ַ
���������û��
*******************************************************************************/
void outIMU_getYawPitchRoll(volatile float * angles) 
{  
    // volatile float outgx=0.0, outgy=0.0, outgz=0.0; //������������
    outIMU_getQ(outq); //����ȫ����Ԫ��
    //��Ԫ��ת����ŷ���ǣ��������Ǻ������㼴��
    angles[0] = -atan2(2 * outq[1] * outq[2] + 2 * outq[0] * outq[3], -2 * outq[2]*outq[2] - 2 * outq[3] * outq[3] + 1)* 180/M_PI; // yaw        -pi----pi
    angles[1] = -asin(-2 * outq[1] * outq[3] + 2 * outq[0] * outq[2])* 180/M_PI; // pitch    -pi/2    --- pi/2 
    angles[2] = atan2(2 * outq[2] * outq[3] + 2 * outq[0] * outq[1], -2 * outq[1] * outq[1] - 2 * outq[2] * outq[2] + 1)* 180/M_PI; // roll       -pi-----pi  
}

static int outyaw_count = 0;
void outGetPitchYawGxGyGz()
{
	outMPU6050_Real_Data.Gyro_X = outmygetqval[3];
	outMPU6050_Real_Data.Gyro_Y = -outmygetqval[4];
	outMPU6050_Real_Data.Gyro_Z = outmygetqval[5];

	outlast_yaw_temp = outyaw_temp;
	outyaw_temp = outangle[0]; 
	if(outyaw_temp-outlast_yaw_temp>=330)  //yaw��ǶȾ����������������
	{
		outyaw_count--;
	}
	else if (outyaw_temp-outlast_yaw_temp<=-330)
	{
		outyaw_count++;
	}
	outyaw_angle = outyaw_temp + outyaw_count*360;  //yaw��Ƕ�
	outpitch_angle = outangle[1];
    outpitch_angle = outangle[2];	
}
