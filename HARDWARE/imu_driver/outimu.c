#include "out6050_interrupt.h"
#include "out6050_driver.h"
#include "out6050_i2c.h"
#include "outimu.h"
#include "timer.h"
#include "time.h"

volatile float outexInt, outeyInt, outezInt;  // 误差积分
volatile float outq0 = 1.0f;
volatile float outq1 = 0.0f;
volatile float outq2 = 0.0f;
volatile float outq3 = 0.0f;

volatile float outmygetqval[9];	//用于存放传感器转换结果的数组
static volatile float outgx, outgy, outgz, outax, outay, outaz, outmx, outmy, outmz;   //作用域仅在此文件中

static volatile float outq[4]; //　四元数
volatile uint32_t outlastUpdate, outnow; // 采样周期计数 单位 us
volatile float outangle[3] = {0};
volatile float outyaw_temp,outpitch_temp,outroll_temp;
volatile float outlast_yaw_temp,outlast_pitch_temp,outlast_pitch_temp;
volatile float outyaw_angle,outpitch_angle,outpitch_angle; //使用到的角度值

// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float outinvSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
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

/**************************实现函数********************************************
*函数原型:	   void outInit_Quaternion
*功　　能:	 初始化四元数
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
//初始化IMU数据
#define BOARD_DOWN 1   //板子正面朝下摆放

void outInit_Quaternion()//根据测量数据，初始化outq0,outq1,outq2.outq3，从而加快收敛速度
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
	
	//根据hx hy hz来判断outq的值，取四个相近的值做逼近即可,初始值可以由欧拉角转换到四元数计算得到
	 
}


	


/**************************实现函数********************************************
*函数原型:	   void outIMU_getValues(volatile float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值  
输入参数： 将结果存放的数组首地址
加速度值：原始数据，-8192-+8192
角速度值：deg/s
磁力计值：原始数据
输出参数：没有
*******************************************************************************/
void outIMU_getValues(volatile float * values) {  
		int16_t accgyroval[6];
		int i;
	//读取加速度和陀螺仪的当前ADC
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
        values[i] = ((float) accgyroval[i]) / 32.8f; //转成度每秒
		//这里已经将量程改成了 1000度每秒  32.8 对应 1度每秒
      }
    }
    outHMC58X3_mgetValues(&values[6]);	//读取磁力计的ADC值
		outMPU6050_Raw_Data.Mag_X = values[6];
		outMPU6050_Raw_Data.Mag_Y = values[7];
		outMPU6050_Raw_Data.Mag_Z = values[8];
}

/**************************实现函数********************************************
*函数原型:	   void outIMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
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

    outnow = Get_Time_Micros();  //读取时间 单位是us   
//	outnow=sysTickUptime;
    if(outnow<outlastUpdate)
    {
    //halfT =  ((float)(outnow + (0xffffffff- outlastUpdate)) / 2000000.0f);   //  uint 0.5s
    }
    else	
    {
        halfT =  ((float)(outnow - outlastUpdate) / 2000000.0f);
    }
    outlastUpdate = outnow;	//更新时间
    //快速求平方根算法
    norm = outinvSqrt(outax*outax + outay*outay + outaz*outaz);       
    outax = outax * norm;
    outay = outay * norm;
    outaz = outaz * norm;
    //把加计的三维向量转成单位向量。
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
        // 用叉积误差来做PI修正陀螺零偏
        outgx = outgx + Kp*ex + outexInt;
        outgy = outgy + Kp*ey + outeyInt;
        outgz = outgz + Kp*ez + outezInt;
    }
    // 四元数微分方程
    tempq0 = outq0 + (-outq1*outgx - outq2*outgy - outq3*outgz)*halfT;
    tempq1 = outq1 + (outq0*outgx + outq2*outgz - outq3*outgy)*halfT;
    tempq2 = outq2 + (outq0*outgy - outq1*outgz + outq3*outgx)*halfT;
    tempq3 = outq3 + (outq0*outgz + outq1*outgy - outq2*outgx)*halfT;  

    // 四元数规范化
    norm = outinvSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    outq0 = tempq0 * norm;
    outq1 = tempq1 * norm;
    outq2 = tempq2 * norm;
    outq3 = tempq3 * norm;

}


/**************************实现函数********************************************
*函数原型:	   void outIMU_getQ(float * outq)
*功　　能:	 更新四元数 返回当前的四元数组值
输入参数： 将要存放四元数的数组首地址
输出参数：没有
*******************************************************************************/

void outIMU_getQ(volatile float * outq) {

    outIMU_getValues(outmygetqval);	 //获取原始数据,加速度计和磁力计是原始值，陀螺仪转换成了deg/s
    outIMU_AHRSupdate();
    outq[0] = outq0; //返回当前值
    outq[1] = outq1;
    outq[2] = outq2;
    outq[3] = outq3;
}

/**************************实现函数********************************************
*函数原型:	   void outIMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void outIMU_getYawPitchRoll(volatile float * angles) 
{  
    // volatile float outgx=0.0, outgy=0.0, outgz=0.0; //估计重力方向
    outIMU_getQ(outq); //更新全局四元数
    //四元数转换成欧拉角，经过三角函数计算即可
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
	if(outyaw_temp-outlast_yaw_temp>=330)  //yaw轴角度经过处理后变成连续的
	{
		outyaw_count--;
	}
	else if (outyaw_temp-outlast_yaw_temp<=-330)
	{
		outyaw_count++;
	}
	outyaw_angle = outyaw_temp + outyaw_count*360;  //yaw轴角度
	outpitch_angle = outangle[1];
    outpitch_angle = outangle[2];	
}
