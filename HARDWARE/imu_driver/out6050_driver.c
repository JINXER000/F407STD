//#include "mpu6050_interrupt.h"
#include "out6050_driver.h"
#include "out6050_i2c.h"
#include "outimu.h"
#include "cali.h"
#include "mpu6050_driver.h"

extern volatile MPU6050_RAW_DATA    outMPU6050_Raw_Data;    //ԭʼ����
extern volatile MPU6050_REAL_DATA   outMPU6050_Real_Data;
AHRS outahrs;
uint8_t outmpu_buf[20]={0};       //save the data of acc gyro & mag using iic

int16_t outMPU6050_FIFO[6][11] = {0};//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
int16_t outHMC5883_FIFO[3][11] = {0};//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ ע���Ŵ������Ĳ���Ƶ���������Ե����г�
outMagMaxMinData_t outMagMaxMinData;
// extern MagMaxMinData_t outMagMaxMinData;


float outHMC5883_lastx,outHMC5883_lasty,outHMC5883_lastz;
//MPU6050 ��ʼ�����ɹ�����0  ʧ�ܷ��� 0xff
int mpuid;

extern MagCaliStruct_t outMagSavedCaliData;			    //Mag offset data

int outMPU6050_Init(void)
{
    unsigned char temp_data = 0x00;

    IIC2_GPIO_Init();  //��ʼ��IIC�ӿ�
//    HEAT_Configuration();
    
    if(IIC2_ReadData(MPU6050_DEVICE_ADDRESS,WHO_AM_I,&temp_data,1)==0) //ȷ��IIC�����Ϲҽӵ��Ƿ���MPU6050
    {
        if(temp_data != OUTMPU6050_ID)
        {
            printf("error 1A\r\n+ 0x%x",temp_data);
						mpuid=temp_data;
            return 0xff; //У��ʧ�ܣ�����0xff
        }
    }
    else
    {
        printf("error 1B\r\n");
        return 0xff; //��ȡʧ�� ����0xff
    }
    
    if(IIC2_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01) == 0xff)    //�������״̬
    {
        printf("error 1C\r\n");
        return 0xff;
    }
		

    if(IIC2_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x03) == 0xff)         //Digital Low-Pass Filter:DLPF_CFG is 3, Fs is 1khz 
    {                                                                     //acc bandwidth 44Hz,gyro 42Hz
        printf("error 1E\r\n");
        return 0xff;
    }
    if(IIC2_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x10) == 0xff)    //FS_SEL 3 : gyroscope full scale range is +-1000degs/s 
    {
        printf("error 1F\r\n");
        return 0xff;
    }
    if(IIC2_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x00) == 0xff)   //AFS_SEL 1: accelerometer full scale range is +-2g
    {
        printf("error 1G\r\n");
        return 0xff;
    }
    if(IIC2_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x02) == 0xff)    //logic level for the INT pin is active high
                                                                          //the INT pin emits a 50us long pulse, not latched    bypass mode enabled
    {
        printf("error 1H\r\n");
        return 0xff;
    }
    if(IIC2_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x00) == 0xff)      //disable data ready interrupt
    {
        printf("error 1I\r\n");
        return 0xff;
    }
		
		//����mpu6050 IIC masters mode  disabled ����mpu6050����aux IIC�ӿ�
		if(IIC2_WriteData(MPU6050_DEVICE_ADDRESS,MPU6050_RA_USER_CTRL,0x00) == 0xff)      //disable data ready interrupt
    {
        printf("error 1I\r\n");
        return 0xff;
    }
		
		//����IIC masters mode Ϊ bypass mode enabled����INT_PIN_CFG������
		

		
//		//5883��ʼ��
		if(outHMC5883_Init() == 0xff)
		{
			  printf("error 1K\r\n");
        return 0xff;
		}
		
    delay_ms(500);
    //MPU6050_GyroCalibration();
    return 0;
}

int outMPU6050_EnableInt(void)
{
	  if(IIC2_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x01)==0xff)      //Sample Rate: Gyro output rate / (1 + 1) = 500Hz
	  {
        printf("Cannot enable interrupt successfully.\r\n");
        return 0xff;
	  }
    printf("MPU6050 set sample rate done.\n");
	  delay_ms(10);
	  if(IIC2_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01) == 0xff)     //enable data ready interrupt 
    {
      printf("error 1I\r\n");
      return 0xff;
    } 
#if 0
    printf("MPU6050 enable interrupt done.\n"); 
//	unsigned char  buf[14] ={0};    
//    delay_ms(2);
//	IIC2_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14);    //dummy read to clear the data registers
//	delay_ms(10);
#endif
	return 0;
}

void outMPU6050_Initialize(void)
{
    while(outMPU6050_Init() == 0xff) 
    {                       
       delay_ms(200);              
    }
}

//MPU6050  ���ݶ�ȡ���ɹ�����0  ʧ�ܷ��� 0xff



int outMPU6050_ReadData(uint8_t Slave_Addr, uint8_t Reg_Addr, uint8_t * Data, uint8_t Num)
{    
	//IIC2_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14)
    if(IIC2_ReadData(Slave_Addr,Reg_Addr,Data,Num) == 0xff)
    {
        printf("error 1J\r\n");
        return 0xff;
    }
   
    return 0;
}

/**********************************************************************************/
/*��MPU6050_ax,MPU6050_ay, MPU6050_az,MPU6050_gx, MPU6050_gy, MPU6050_gz�����洢*/
/**********************************************************************************/
void outMPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
{
	uint8_t i = 0;
	int32_t sum=0;
	
	for(i=1;i<10;i++)
	{
		outMPU6050_FIFO[0][i-1]=outMPU6050_FIFO[0][i];
		outMPU6050_FIFO[1][i-1]=outMPU6050_FIFO[1][i];
		outMPU6050_FIFO[2][i-1]=outMPU6050_FIFO[2][i];
		outMPU6050_FIFO[3][i-1]=outMPU6050_FIFO[3][i];
		outMPU6050_FIFO[4][i-1]=outMPU6050_FIFO[4][i];
		outMPU6050_FIFO[5][i-1]=outMPU6050_FIFO[5][i];
	}
	
	outMPU6050_FIFO[0][9]=ax;//���µ����ݷ��õ� ���ݵ������
	outMPU6050_FIFO[1][9]=ay;
	outMPU6050_FIFO[2][9]=az;
	outMPU6050_FIFO[3][9]=gx;
	outMPU6050_FIFO[4][9]=gy;
	outMPU6050_FIFO[5][9]=gz;
	
	for(i=0;i<10;i++)//��ǰ����ĺϣ���ȡƽ��ֵ
	{	
		 sum+=outMPU6050_FIFO[0][i];
	}
	outMPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=outMPU6050_FIFO[1][i];
	}
	outMPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=outMPU6050_FIFO[2][i];
	}
	outMPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=outMPU6050_FIFO[3][i];
	}
	outMPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=outMPU6050_FIFO[4][i];
	}
	outMPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=outMPU6050_FIFO[5][i];
	}
	outMPU6050_FIFO[5][10]=sum/10;
	
}


int16_t outMPU6050_Lastax,outMPU6050_Lastay,outMPU6050_Lastaz
				,outMPU6050_Lastgx,outMPU6050_Lastgy,outMPU6050_Lastgz;
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
*��������:	    ��ȡ MPU6050�ĵ�ǰ����ֵ
*******************************************************************************/
void outMPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  
	if(outisMPU6050_is_DRY)
	{
		outisMPU6050_is_DRY = 0;
		outMPU6050_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,outmpu_buf,14);  //
		outHMC58X3_ReadData(&(outmpu_buf[14]));  //14-19Ϊ����������
		outMPU6050_Lastax=(((int16_t)outmpu_buf[0]) << 8) | outmpu_buf[1];
		outMPU6050_Lastay=(((int16_t)outmpu_buf[2]) << 8) | outmpu_buf[3];
		outMPU6050_Lastaz=(((int16_t)outmpu_buf[4]) << 8) | outmpu_buf[5];
		//�����¶�ADC
		outMPU6050_Lastgx=(((int16_t)outmpu_buf[8]) << 8) | outmpu_buf[9];
		outMPU6050_Lastgy=(((int16_t)outmpu_buf[10]) << 8) | outmpu_buf[11];
		outMPU6050_Lastgz=(((int16_t)outmpu_buf[12]) << 8) | outmpu_buf[13];
			
		outMPU6050_DataSave(outMPU6050_Lastax,outMPU6050_Lastay,outMPU6050_Lastaz,outMPU6050_Lastgx,outMPU6050_Lastgy,outMPU6050_Lastgz);  		
		*ax  =outMPU6050_FIFO[0][10];
		*ay  =outMPU6050_FIFO[1][10];
		*az = outMPU6050_FIFO[2][10];
		*gx  =outMPU6050_FIFO[3][10];
		*gy = outMPU6050_FIFO[4][10];
		*gz = outMPU6050_FIFO[5][10];
	} 
	else
	{       //��ȡ��һ�ε�ֵ
		*ax = outMPU6050_FIFO[0][10];//=outMPU6050_FIFO[0][10];
		*ay = outMPU6050_FIFO[1][10];//=outMPU6050_FIFO[1][10];
		*az = outMPU6050_FIFO[2][10];//=outMPU6050_FIFO[2][10];
		*gx = outMPU6050_FIFO[3][10];//=outMPU6050_FIFO[3][10];
		*gy = outMPU6050_FIFO[4][10];//=outMPU6050_FIFO[4][10];
		*gz = outMPU6050_FIFO[5][10];//=outMPU6050_FIFO[5][10];
	}
}

void outMPU6050_getlastMotion6(int16_t* ax, int16_t* ay, 
		int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	*ax  =outMPU6050_FIFO[0][10];
	*ay  =outMPU6050_FIFO[1][10];
	*az = outMPU6050_FIFO[2][10];
	*gx  =outMPU6050_FIFO[3][10];
	*gy = outMPU6050_FIFO[4][10];
	*gz = outMPU6050_FIFO[5][10];
}

uint8_t outHMC5883_Init(void)
{
    uint8_t tmp_ch = 0;
    if(outMPU6050_ReadData(HMC5883_ADDRESS, HMC58X3_R_IDA, &tmp_ch, 1) == 0)
    {
        if(tmp_ch != HMC5883_DEVICE_ID_A)
        {
            printf("error 2A\r\n");
            return 0xff;
        }
    }
    else
    {
        printf("error 2B\r\n");
        return 0xff;
    }

    IIC2_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,0x70);
    delay_ms(5);
    IIC2_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFB,0xA0);
    delay_ms(5);
    IIC2_WriteData(HMC5883_ADDRESS, HMC58X3_R_MODE,0x00);    //�����ʼ��Ϊ0x00 ����ģʽ
    //wait the response of the hmc5883 stabalizes, 6 milliseconds  
    delay_ms(6);
    //set DOR
    IIC2_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,6<<2);   //75HZ����
    delay_ms(6);
    return 0;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*��������:	   дHMC5883L�ļĴ�����ȡ5883�Ĵ�����ֵ
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
*******************************************************************************/
void outHMC58X3_ReadData(u8 *vbuff) {   
   outMPU6050_ReadData(HMC5883_ADDRESS,HMC58X3_R_XM,vbuff,6);   //��ȡ������������
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*��������:	   ����һ�����ݵ�FIFO����
���������  �������������Ӧ��ADCֵ
���������  ��
*******************************************************************************/
void  outHMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	uint8_t i = 0;
	int32_t sum=0;

	for(i=1;i<10;i++)
	{
		outHMC5883_FIFO[0][i-1]=outHMC5883_FIFO[0][i];
		outHMC5883_FIFO[1][i-1]=outHMC5883_FIFO[1][i];
		outHMC5883_FIFO[2][i-1]=outHMC5883_FIFO[2][i];
	}
	outHMC5883_FIFO[0][9]= x;//���µ����ݷ��õ� ���ݵ������
	outHMC5883_FIFO[1][9]= y;
	outHMC5883_FIFO[2][9]= z;
	
	for(i=0;i<10;i++)//��ǰ����ĺϣ���ȡƽ��ֵ
	{	
		 sum+=outHMC5883_FIFO[0][i];
	}
	outHMC5883_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=outHMC5883_FIFO[1][i];
	}
	outHMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=outHMC5883_FIFO[2][i];
	}
	outHMC5883_FIFO[2][10]=sum/10;
	//����ȫ��ΪδУ׼����
	if(outMagMaxMinData.MinMagX>outHMC5883_FIFO[0][10])
	{
		outMagMaxMinData.MinMagX=(int16_t)outHMC5883_FIFO[0][10];
	}
	if(outMagMaxMinData.MinMagY>outHMC5883_FIFO[1][10])
	{
		outMagMaxMinData.MinMagY=(int16_t)outHMC5883_FIFO[1][10];
	}
	if(outMagMaxMinData.MinMagZ>outHMC5883_FIFO[2][10])
	{
		outMagMaxMinData.MinMagZ=(int16_t)outHMC5883_FIFO[2][10];
	}

	if(outMagMaxMinData.MaxMagX<outHMC5883_FIFO[0][10])
	{
		outMagMaxMinData.MaxMagX=(int16_t)outHMC5883_FIFO[0][10];		
	}
	if(outMagMaxMinData.MaxMagY<outHMC5883_FIFO[1][10])
	{
		outMagMaxMinData.MaxMagY = outHMC5883_FIFO[1][10];
	}
	if(outMagMaxMinData.MaxMagZ<outHMC5883_FIFO[2][10])
	{
		outMagMaxMinData.MaxMagZ=(int16_t)outHMC5883_FIFO[2][10];
	}		
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*��������:	   дHMC5883L�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
*******************************************************************************/
void outHMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) 
{
    outHMC58X3_ReadData(&outmpu_buf[14]);
    outHMC58X3_newValues((((int16_t)outmpu_buf[18] << 8) | outmpu_buf[19]), -(((int16_t)outmpu_buf[14] << 8) | outmpu_buf[15]), ((int16_t)outmpu_buf[16] << 8) | outmpu_buf[17]);
    *x = outHMC5883_FIFO[0][10];
    *y = outHMC5883_FIFO[1][10];
    *z = outHMC5883_FIFO[2][10];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z)
*��������:	   ��ȡ �����Ƶĵ�ǰADCֵ
���������    �������Ӧ�����ָ��	
���������  ��
*******************************************************************************/
void outHMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z) 
{
    *x = outHMC5883_FIFO[0][10];
    *y = outHMC5883_FIFO[1][10]; 
    *z = outHMC5883_FIFO[2][10]; 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_mgetValues(volatile float *arry)
*��������:	   ��ȡ У����� ������ADCֵ
���������    �������ָ��	
���������  ��
*******************************************************************************/
void outHMC58X3_mgetValues(volatile float *arry) 
{
    int16_t xr,yr,zr;
    outHMC58X3_getRaw(&xr, &yr, &zr);
    arry[0]= outHMC5883_lastx=((float)(xr- outMagSavedCaliData.MagXOffset ))* outMagSavedCaliData.MagXScale;
    arry[1]= outHMC5883_lasty=((float)(yr - outMagSavedCaliData.MagYOffset ))* outMagSavedCaliData.MagYScale;
    arry[2]= outHMC5883_lastz=((float)(zr - outMagSavedCaliData.MagZOffset ))* outMagSavedCaliData.MagZScale;
}





