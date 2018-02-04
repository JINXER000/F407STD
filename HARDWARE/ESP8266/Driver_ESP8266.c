#define __DRIVER_ESP8266_GLOBALS


#include "Driver_ESP8266.h"
#include "StringDecoding.h"
#include <String.h>
#include "timer.h"
#include	"delay.h"
#include "DMA.h"
extern uint32_t system_micrsecond;
/**
  * @brief  ESP8266��ʼ��
  * @param  void
  * @retval 1 ��ʼ���ɹ�         0 ��ʼ��ʧ��
  */
void ESP8266_InitConfig(void)
{
    LatestRespond = 0;
    
    ESP8266APPortLinkFlag = 0;
    
	    ESP8266_ExitTransparentMode();
//    while(!ESP8266_SetMode(1))
//    {
//        delay_ms(500);
//    }
//    
//    while(!ESP8266_JoinAP(TargetAPSSID, TargetAPPassword, 20000))
//    {
//        delay_ms(3000);
//    }
    
    while(!ESP8266_ConnectServerPort(1, "192.168.1.125", "6000"))
    {
        delay_ms(1000);
    }
    
    while(!ESP8266_StartTransparentMode())
    {
        delay_ms(1000);
    }
    
    ESP8266APPortLinkFlag = 1;

                
}


/**
  * @brief  ��ESP8266����һ������(��ESP8266ControlTXBuffer[ESP8266ControlTXBufferLenght])
  * @param  ������
  * @retval void
  */
void ESP8266_SendPack(uint16_t Num)
{
    if(DMA1_Stream3->NDTR)
    {
        return;
    }
    DMA_Cmd(DMA1_Stream3, DISABLE);                                     //�ر� DMA ����
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}                 //ȷ�� DMA ���Ա�����
    DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);       //��ձ�־λ
    DMA1_Stream3->M0AR = (uint32_t)ESP8266TXBuffer;              //��������
    DMA_SetCurrDataCounter(DMA1_Stream3, Num);                          //���ݴ�����
    DMA_Cmd(DMA1_Stream3, ENABLE);                                      //���� DMA ����
}


/**
  * @brief  �ȴ���Ӧ����
  * @param  ��ʱʱ��
  * @param  ���ʱ��
  * @retval responseOK  �յ�����        responseNO  ֱ����ʱ��δ�յ�����
  */
responseType ESP8266_WaitFordata(uint32_t Tick, uint32_t Crack)
{
   uint32_t  StartTick=system_micrsecond;
    
    LatestRespond = 0;
    
    while( Get_Time_Micros() - StartTick < Tick)
    {
        if(LatestRespond)
        {
            return responseOK;
        }
        else
        {
            delay_ms(Crack);
        }
    }
    return responseNO;
    
}



/**
  * @brief  ģ�鸴λ
  * @param  void
  * @retval 0 ��λʧ��       1 ��λ�ɹ�
  */
uint8_t ESP8266_Reset(void)
{
    strcpy(ESP8266TXBuffer, "AT+RST\n");
    ESP8266_SendPack(7);
    
    //δ�յ���Ӧ
    if(ESP8266_WaitFordata(3000, 50) == responseNO)
    {
        return 0;
    }
    
    //�յ���Ӧ��ȷ���Ƿ���OK
    if(SDEC_StringIsEqual(ESP8266RXBuffer, "OK") == 0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}


/**
  * @brief  ����Ϊָ��ģʽ
  * @param  1 Stationģʽ     2 APģʽ     3 AP+Stationģʽ
  * @retval 0 ����ʧ��      1 ���óɹ�
  */
uint8_t  ESP8266_SetMode(uint8_t mode)
{
    if(mode == 1)
    {
        strcpy(ESP8266TXBuffer, "AT+CWMODE=1\r\n");
    }
    else if(mode == 2)
    {
        strcpy(ESP8266TXBuffer, "AT+CWMODE=2\r\n");
    }
    else
    {
        strcpy(ESP8266TXBuffer, "AT+CWMODE=3\r\n");
    }
    
    ESP8266_SendPack(11);
    
    //δ�յ���Ӧ
    if(ESP8266_WaitFordata(1000, 50) == responseNO)
    {
        return 0;
    }
    
    //�յ���Ӧ��ȷ���Ƿ�ɹ�
    if(mode == 1)
    {
        if(*(ESP8266RXBuffer + 10) != '1')
        {
            return 0;
        }
    }
    else if(mode == 2)
    {
        if(*(ESP8266RXBuffer + 10) != '2')
        {
            return 0;
        }
    }
    else
    {
        if(*(ESP8266RXBuffer + 10) != '3')
        {
            return 0;
        }
    }
    
    return 1;
}


/**
  * @brief  ��Ѱ����AP�б�
  * @param  �洢APP�б�������׵�ַ
  * @param  �洢��WiFi�����б����ƫ�Ƶ������׵�ַ����һ������ΪAP������
  * @retval ��������AP����
  */
uint16_t ESP8266_SearchAP(char *List, uint16_t *Offset)
{
    strcpy(ESP8266TXBuffer, "AT+CWLAP\r\n");
    ESP8266_SendPack(10);
    
    //δ�յ���Ӧ
    if(ESP8266_WaitFordata(10000, 50) == responseNO)
    {
        return 0;
    }
    
    //δ�յ���Ӧ
    if(ESP8266_WaitFordata(20000, 50) == responseNO)
    {
        return 0;
    }
    
#warning decoding is not finished!!!
    
    return 1;
}


/**
  * @brief  ����ָ��AP
  * @param  ָ��AP��SSID
  * @param  ����
  * @param  ��ʱʱ��
  * @retval 0 ����ʧ��      1 ����ɹ�
  */
uint8_t ESP8266_JoinAP(char *SSID, char *Password, uint16_t Tick)
{
    char Str[128] = "AT+CWJAP=\"";
    uint16_t Lenght = 0;
    
    strcat(Str, SSID);
    Lenght += SDEC_Lenght(SSID);
    strcat(Str, "\",\"");
    strcat(Str, Password);
    Lenght += SDEC_Lenght(Password);
    strcat(Str, "\"\r\n");
    Lenght += 16;
    
    strcpy(ESP8266TXBuffer, Str);
    ESP8266_SendPack(Lenght);
    
    //δ�յ���Ӧ
    if(ESP8266_WaitFordata(5000, 100) == responseNO)
    {
        return 0;
    }
    
    if(SDEC_ChildStringIsEqual(ESP8266RXBuffer, 45, "ERROR", 0, 5))
    {
        return 0;
    }
    
    //δ�յ���Ӧ
    if(ESP8266_WaitFordata(20000, 100) == responseNO)
    {
        return 0;
    }
    
    //δ�յ���Ӧ
    if(ESP8266_WaitFordata(20000, 100) == responseNO)
    {
        return 0;
    }
    
    //δ�յ���Ӧ
    if(ESP8266_WaitFordata(20000, 100) == responseNO)
    {
        return 0;
    }
    
    if(SDEC_ChildStringIsEqual(ESP8266RXBuffer, 2, "OK", 0, 2))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief  ����ָ��IP
  * @param  Э��  1 ��TCP��     0��UDP��
  * @param  IP��ַ
  * @param  �˿�
  * @retval 0 ʧ��      1 �ɹ�
  */
uint8_t ESP8266_ConnectServerPort(uint8_t  protocol, char *IP, char *port)
{
    char Str[128] = "AT+CIPSTART=\"";
    uint16_t Lenght = 0;

//�����ַ�������
    if(protocol)
    {
        strcat(Str, "TCP");
    }
    else
    {
        strcat(Str, "UDP");
    }
    
    strcat(Str, "\",\"");
    
    strcat(Str, IP);
    Lenght += SDEC_Lenght(IP);
    
    strcat(Str, "\",");
    strcat(Str, port);
    
    Lenght += SDEC_Lenght(port);
    strcat(Str, "\r\n");
    
    Lenght += 25;
    
//����
    strcpy(ESP8266TXBuffer, Str);
    ESP8266_SendPack(Lenght);
    
    //δ�յ���Ӧ
    if(ESP8266_WaitFordata(20000, 50) == responseNO)
    {
        return 0;
    }
    
    //�ж����ӽ��
    if((SDEC_ChildStringIsEqual(ESP8266RXBuffer, 0, "CONNECT", 0, 7)) || (SDEC_ChildStringIsEqual(ESP8266RXBuffer, 41, "ALREADY CONNECTED", 0, 17)))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief  ���봮��͸��ģʽ,������͸��
  * @param  void
  * @retval 0 ����ʧ��      1 ����ɹ�
  */
uint8_t ESP8266_StartTransparentMode(void)
{
    strcpy(ESP8266TXBuffer, "AT+CIPMODE=1\r\n");
    ESP8266_SendPack(14);
    
    //��Ӧ1
    if(ESP8266_WaitFordata(2000, 50) == responseNO)
    {
        return 0;
    }
    
    //��Ӧ�ж�
    if(!SDEC_ChildStringIsEqual(ESP8266RXBuffer, 17, "OK", 0, 2))
    {
        return 0;
    }
    
    strcpy(ESP8266TXBuffer, "AT+CIPSEND\r\n");
    ESP8266_SendPack(12);
    
    //��Ӧ1
    if(ESP8266_WaitFordata(2000, 50) == responseNO)
    {
        return 0;
    }
    
    //������Ӧ
    if(!SDEC_ChildStringIsEqual(ESP8266RXBuffer, 15, "OK", 0, 2))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}


/**
  * @brief  ͸��ģʽ�·�������
  * @param  ����������
  * @param  ������ʼƫ��
  * @param  ��������-1Ϊ������'\0'��
  * @retval void
  */
void ESP8266_SendTransparentData(uint8_t *data, uint16_t Offset, int8_t Lenght)
{
    char *BufferPoint = ESP8266TXBuffer;
	  uint16_t index;
    data += Offset;
  
    
    //�������������ͻ���
    if(Lenght < 0)
    {
        Lenght = 0;
        while(*data)
        {
            *BufferPoint = *data;
            Lenght++;
            data++;
            BufferPoint++;
        }
    }
    else
    {
        for(index = 0; index < Lenght; index++)
        {
            *BufferPoint = *data;
            data++;
            BufferPoint++;
        }
    }
    
    ESP8266_SendPack(Lenght);
}


/**
  * @brief  �˳�͸��ģʽ
  * @param  void
  * @retval void
  */
void ESP8266_ExitTransparentMode(void)
{
    strcpy(ESP8266TXBuffer, "+++\r\n");
    ESP8266_SendPack(5);
}







