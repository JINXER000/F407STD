#include "out6050_i2c.h"


/*----I2C2----SCL----PB1---*/
/*----I2C2----SDA----PB0---*/
#define IIC2_SCL_H()      GPIO_SetBits(GPIOB,GPIO_Pin_1)
#define IIC2_SCL_L()      GPIO_ResetBits(GPIOB,GPIO_Pin_1)
#define IIC2_SDA_H()      GPIO_SetBits(GPIOB,GPIO_Pin_0)
#define IIC2_SDA_L()      GPIO_ResetBits(GPIOB,GPIO_Pin_0)
#define IIC2_SDA_Read()   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)

void IIC2_Delay(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a = 6;//6
		while(a--);
	}
}



void IIC2_GPIO_Init(void)
{
  GPIO_InitTypeDef   gpio;	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	gpio.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
//	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_OType =GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz; 
  GPIO_Init(GPIOB, &gpio);
}

void IIC2_SDA_Out(void)
{
  GPIO_InitTypeDef   gpio;    
	gpio.GPIO_Pin = GPIO_Pin_0;
  gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_Speed = GPIO_Speed_100MHz; 
  GPIO_Init(GPIOB, &gpio);
}

void IIC2_SDA_In(void)
{
    GPIO_InitTypeDef   gpio;    
	  gpio.GPIO_Pin = GPIO_Pin_0;    
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &gpio);
}

void IIC2_Start(void)								  
{
	IIC2_SDA_Out();
	IIC2_SDA_H();
	IIC2_SCL_H();
	IIC2_Delay(1);
	IIC2_SDA_L();
	IIC2_Delay(1);
	IIC2_SCL_L();
}

void IIC2_Stop(void)
{
	IIC2_SDA_Out();
	IIC2_SCL_L();
	IIC2_SDA_L();
	IIC2_Delay(1);
	IIC2_SCL_H();
	IIC2_SDA_H();
	IIC2_Delay(1);
}

void IIC2_Ack(u8 re)					     
{
	IIC2_SDA_Out();
	if(re)
	   IIC2_SDA_H();
	else
	   IIC2_SDA_L();
	IIC2_SCL_H();
	IIC2_Delay(1);
	IIC2_SCL_L();
	IIC2_Delay(1);
}

int IIC2_WaitAck(void)
{
	u16 Out_Time=1000;    
  IIC2_SDA_H();
	IIC2_SDA_In();
	IIC2_Delay(1);
	IIC2_SCL_H();
	IIC2_Delay(1);
	while(IIC2_SDA_Read())
	{
		if(--Out_Time)
		{
			IIC2_Stop();
            printf("error 2A\r\n");
            return 0xff;
		}
	}
	IIC2_SCL_L();
    return 0;
}

void IIC2_WriteBit(u8 Temp)
{
	u8 i;
	IIC2_SDA_Out();
	IIC2_SCL_L();
	for(i=0;i<8;i++)
	{
		if(Temp&0x80)
		{
			IIC2_SDA_H();
		}
		else
		{
			IIC2_SDA_L();
		}
		Temp<<=1;
		IIC2_Delay(1);
		IIC2_SCL_H();
		IIC2_Delay(1);
		IIC2_SCL_L();
	}
}

u8 IIC2_ReadBit(void)
{
	u8 i,Temp=0;
	IIC2_SDA_In();
	for(i=0;i<8;i++)
	{
		IIC2_SCL_L();
		IIC2_Delay(1);
		IIC2_SCL_H();
		Temp<<=1;
		if(IIC2_SDA_Read())
		   Temp++;
		IIC2_Delay(1);
	}
	IIC2_SCL_L();
	return Temp;
}

//写数据，成功返回0，失败返回0xff
int IIC2_WriteData(u8 dev_addr,u8 reg_addr,u8 data)
{
	IIC2_Start();
    
	IIC2_WriteBit(dev_addr);
	if(IIC2_WaitAck() == 0xff)
    {
        printf("error 2B\r\n");
        return 0xff;
    }
    
	IIC2_WriteBit(reg_addr);
	if(IIC2_WaitAck() == 0xff)
    {
        printf("error 2C\r\n");
        return 0xff;
    }

    IIC2_WriteBit(data);
    if(IIC2_WaitAck() == 0xff)
    {
        printf("error 2D\r\n");
        return 0xff;
    }

	 IIC2_Stop();
    return 0;
}

//读数据，成功返回0，失败返回0xff
int IIC2_ReadData(u8 dev_addr,u8 reg_addr,u8 *pdata,u8 count)
{
	  u8 i;

    IIC2_Start();
	
    IIC2_WriteBit(dev_addr);
	  if(IIC2_WaitAck() == 0xff)
    {
        printf("error 2F\r\n");
        return 0xff;
    }
    
    IIC2_WriteBit(reg_addr);
	  if(IIC2_WaitAck() == 0xff)
    {
        printf("error 2G\r\n");
        return 0xff;
    }
	
    IIC2_Start();
    
    IIC2_WriteBit(dev_addr+1);
	  if(IIC2_WaitAck() == 0xff)
    {
        printf("error 2H\r\n");
        return 0xff;
    }
    
    for(i=0;i<(count-1);i++)
    {
        *pdata=IIC2_ReadBit();
        IIC2_Ack(0);
        pdata++;
    }

    *pdata=IIC2_ReadBit();
    IIC2_Ack(1); 
    
    IIC2_Stop(); 
    
    return 0;    
}
