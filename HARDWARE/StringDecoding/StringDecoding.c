

#include "StringDecoding.h"


/**
  * @brief  �ַ������ȼ���
  * @brief  �ַ����׵�ַ
  * @retval �ַ�������
  */
int32_t SDEC_Lenght(char *Str)
{
    int32_t Lenght = 0;
    
    for(;;)
    {
        if(*Str)
        {
            Str++;
            Lenght++;
        }
        else
        {
            break;
        }
    }
    return Lenght;
}


/**
  * @brief  �����ַ���ָ��λ�õ��ַ�
  * @brief  �ַ����׵�ַ
  * @param  λ��
  * @retval �ַ�
  */
char SDEC_GetChar(char *Str, int32_t Location)
{
    return *(Str + Location);
}


/**
  * @brief  �����ַ���ָ��λ��ָ�������ַ���
  * @brief  �ַ����׵�ַ
  * @brief  Ŀ���ַ����洢�׵�ַ
  * @param  λ��
  * @retval void
  */
void SDEC_GetString(char *Str, char *Target, char Location, int32_t Lenght)
{
    int32_t index;
    
    for(Str += Location, index = 0; index < Lenght; index++, Str++, Target++)
    {
        *Target = *Str;
    }
    *Target = 0;
}



/**
  * @brief  ����ָ���ַ���һ�γ��ֵ�λ��
  * @param  ����Ѱ���ַ���
  * @param  Ŀ���ַ�
  * @retval λ�� (������ʾ������)
  */
int32_t SDEC_SearchChar(char *Str, char Target)
{
    int32_t Location;
    
    for(Location = 0; *Str != 0; Str++, Location++)
    {
        if(*Str == Target)
        {
            return Location;
        }
    }
    
    return -1;
}



/**
  * @brief  ����ָ���ַ�����һ�γ��ֵ�λ��
  * @param  ����Ѱ���ַ���
  * @param  Ŀ���ַ�
  * @retval λ�� (������ʾ������)
  */
int32_t SDEC_SearchString(char *Str, char *Target)
{
//    int32_t index = 0, Location;
//    
//    for(Location = 0;*Str != 0;Str++, Location++)
//    {
//        if(*Str == *Target)
//        {
//            return index;
//        }
//    }
    
    return -1;
}


/**
  * @brief  �ж��ַ����Ƿ����
  * @param  �ַ���1
  * @param  �ַ���2
  * @retval 1 ���     0 �����
  */
int8_t SDEC_StringIsEqual(char *Str1, char *Str2)
{
    while((*Str1 != 0) && (*Str2 != 0))
    {
        if(*Str1 != *Str2)
        {
            return 0;
        }
        Str1++; 
        Str2++;
    }
    
    if((*Str1 == 0) && (*Str2 == 0))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief  �Ƚ������ַ���ָ��λ��ָ���������ַ����Ƿ����
  * @param  �ַ���1��ַ
  * @param  �ַ���1ƫ��
  * @param  �ַ���2��ַ
  * @param  �ַ���2ƫ��
  * @param  ���Ƚϲ��ֳ���
  * @retval 0 �����           1 ���
  */
uint8_t SDEC_ChildStringIsEqual(char *Str1, uint16_t Offset1, char *Str2, uint16_t Offset2, uint16_t Lenght)
{
    uint16_t index;
    
    Str1 += Offset1;
    Str2 += Offset2;
    
    for(index = 0; index < Lenght; index++, Str1++, Str2++)
    {
        if(*Str1 != *Str2)
        {
            return 0;
        }
    }
    return 1;
}



/**
  * @brief  ָ���ַ���ʳ��ֵ�λ��
  * @param  �ַ����׵�ַ
  * @param  ���ڴ洢λ������ĵ�ַ
  * @param  Ŀ���ַ�
  * @retval ���ִ���
  */
int32_t SDEC_SearchMultiChar(char *Str, uint32_t *Loc, char Target)
{
    int32_t Num = 0, Location = 0;
    
    while(*Str)
    {
        if(*Str == Target)
        {
            Loc[Num] = Location;
            Num++;
        }
        Location++;
        Str++;
    }
    return Num;
}









