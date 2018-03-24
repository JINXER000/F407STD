#include "UART2.h"
//#include "JY61.h"

//extern void CopeSerial2Data(unsigned char ucData);


void Initial_UART2(unsigned long baudrate)
{
			USART_InitTypeDef usart2;
    GPIO_InitTypeDef  gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

    GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART3); 

    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD,&gpio);

    usart2.USART_BaudRate = baudrate;          // speed 10byte/ms
    usart2.USART_WordLength = USART_WordLength_8b;
    usart2.USART_StopBits = USART_StopBits_1;
    usart2.USART_Parity = USART_Parity_No;
    usart2.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2,&usart2);
  
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	
	USART_Cmd(USART2, ENABLE);
}

//void USART3_Configuration(void)
//{
//    USART_InitTypeDef usart3;
//    GPIO_InitTypeDef  gpio;
////    NVIC_InitTypeDef  nvic;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

//    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
//    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 

//    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
//    gpio.GPIO_Mode = GPIO_Mode_AF;
//    gpio.GPIO_OType = GPIO_OType_PP;
//    gpio.GPIO_Speed = GPIO_Speed_100MHz;
//    gpio.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_Init(GPIOB,&gpio);

//    usart3.USART_BaudRate = 115200;          // speed 10byte/ms
//    usart3.USART_WordLength = USART_WordLength_8b;
//    usart3.USART_StopBits = USART_StopBits_1;
//    usart3.USART_Parity = USART_Parity_No;
//    usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
//    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_Init(USART3,&usart3);

//    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
//    USART_Cmd(USART3,ENABLE);

//    nvic.NVIC_IRQChannel = USART3_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 3;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE; 
//    NVIC_Init(&nvic);

//}
void uart6_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
 
	//����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOG,GPIO_PinSource15,GPIO_AF_USART3);		//RX
    GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART3); 			//TX
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOG,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART6, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���1 
		
//	USART_ITConfig(USART6, USART_IT_TXE, ENABLE);    
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//��������ж�
}

void USART2_IRQHandler(void)
{
 if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
//		CopeSerial2Data((unsigned char)USART2->DR);//��������
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
	
	USART_ClearITPendingBit(USART2,USART_IT_ORE);
}

//void USART3_IRQHandler(void)
//{  
// if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//  {
//	
//	CopeSerial2Data((unsigned char)USART3->DR);//��������
//			USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//	}
//	USART_ClearITPendingBit(USART3,USART_IT_ORE);
//}
void USART6_IRQHandler(void)                
{
	u8 Res;
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  
	{
		Res =USART_ReceiveData(USART6);//(USART6->DR);	//��ȡ���յ�������
					USART_ClearITPendingBit(USART6, USART_IT_RXNE);

  } else  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  {   
    USART_ClearITPendingBit(USART2, USART_IT_TXE);
  }

} 

//void UART2_Put_Char(unsigned char DataToSend)
//{
//	TxBuffer[count++] = DataToSend;  
//  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  
//}

//void UART2_Put_String(unsigned char *Str)
//{
//	while(*Str)
//	{
//		if(*Str=='\r')UART2_Put_Char(0x0d);
//			else if(*Str=='\n')UART2_Put_Char(0x0a);
//				else UART2_Put_Char(*Str);
//		Str++;
//	}
//}
