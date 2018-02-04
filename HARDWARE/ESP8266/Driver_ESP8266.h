#ifndef __DRIVER_ESP8266_H
#define __DRIVER_ESP8266_H


#include "sys.h"


#define TargetAPSSID                "TP-LINK_E98B7A"
#define TargetAPPassword            "kaixinjiuhao"


#define ESP8266RXBufferLenght       512                     //ESP8266���ͻ��泤��
#define ESP8266TXBufferLenght       512                     //ESP8266���ջ��泤��
#define ESP8266APListLenght         512                     //������AP�б�
#define ESP8266APListOffsetLenght   64                      //AP�б��SSIDƫ��



#ifdef  __DRIVER_ESP8266_GLOBALS
#define __DRIVER_ESP8266_EXT
#else
#define __DRIVER_ESP8266_EXT extern
#endif


//�յ�������Ӧö��
typedef enum
{
    responseNO,
    responseOK
}responseType;


__DRIVER_ESP8266_EXT uint8_t ESP8266APPortLinkFlag;                             //͸�����ӳɹ���־  0 ����δ�ɹ�         1 ���ӳɹ�
__DRIVER_ESP8266_EXT char ESP8266TXBuffer[ESP8266TXBufferLenght];               //ESP8266���ͻ���
__DRIVER_ESP8266_EXT char ESP8266RXBuffer[ESP8266RXBufferLenght];               //ESP8266���ջ���
__DRIVER_ESP8266_EXT uint8_t LatestRespond;                                     //ESP8266���������������1
__DRIVER_ESP8266_EXT char ESP8266APList[ESP8266APListLenght];                   //ESP8266������AP�б�
__DRIVER_ESP8266_EXT uint16_t ESP8266APListOffset[ESP8266APListOffsetLenght];   //������AP�б��и�SSIDƫ��λ�ã���һ���ֽ�Ϊ������AP���������һ���ֽ�Ϊ�б���ֹλ��

void ESP8266_InitConfig(void);
responseType ESP8266_WaitFordata(uint32_t Tick, uint32_t Crack);
uint8_t ESP8266_Reset(void);
uint8_t  ESP8266_SetMode(uint8_t mode);
void ESP8266_SendPack(uint16_t Num);
uint16_t ESP8266_SearchAP(char *List, uint16_t *Offset);
uint8_t ESP8266_JoinAP(char *SSID, char *Password, uint16_t Tick);
uint8_t ESP8266_ConnectServerPort(uint8_t  protocol, char *IP, char *port);
uint8_t ESP8266_StartTransparentMode(void);
void ESP8266_SendTransparentData(uint8_t *data, uint16_t Offset, int8_t Lenght);
void ESP8266_ExitTransparentMode(void);



#endif
