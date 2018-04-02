#include "sys.h"
#include "usart.h"
#include "cali.h"
#include "bsp_flash.h"
#include "mpu6050_driver.h"
#include "out6050_driver.h"
#include "ospid.h"

AppParam_t gAppParamStruct;	//配置信息,这里保存着最新的校准值，并且与Flash中的内容同步
static MagCaliStruct_t  MagCaliData;         //保存磁力计校准值
MagCaliStruct_t MagSavedCaliData;			    //Mag offset data
static MagCaliStruct_t  outMagCaliData;         //保存磁力计校准值
MagCaliStruct_t outMagSavedCaliData;			    //Mag offset data

uint8_t app_param_calied_flag = 0;

extern outMagMaxMinData_t outMagMaxMinData;
extern MagMaxMinData_t MagMaxMinData;

//用于保存数据到flash中
static uint8_t AppParamSave(void)
{
    uint8_t retval = 1;   
    retval = BSP_FLASH_Write(PARAM_SAVED_START_ADDRESS, (uint8_t *)&gAppParamStruct, sizeof(AppParam_t));    
    if(retval == 0)
    {
			
    }
    return retval;   
}

void AppParamInit(void)
{
		AppParam_t tmp_param;
    
    memcpy(&tmp_param, (void *)PARAM_SAVED_START_ADDRESS, sizeof(AppParam_t));	//read flash
	
//	if((PARAM_SAVED_FLAG == tmp_param.ParamSavedFlag) &&\
//		(PARAM_CALI_DONE == tmp_param.GimbalCaliData.GimbalCaliFlag) &&\
//		(PARAM_CALI_DONE == tmp_param.GyroCaliData.GyroCaliFlag))
		if(PARAM_SAVED_FLAG == tmp_param.ParamSavedFlag)
	{
		app_param_calied_flag =1;
        memcpy(&gAppParamStruct, &tmp_param, sizeof(AppParam_t));
    }
    else
    {
		app_param_calied_flag = 0;
        gAppParamStruct.FirmwareVersion = 0; //保留未使用
        gAppParamStruct.ParamSavedFlag = PARAM_SAVED_FLAG;
    }


	    if(gAppParamStruct.MagCaliData.MagCaliFlag != PARAM_CALI_DONE)
    {
        gAppParamStruct.MagCaliData.MagCaliFlag = PARAM_CALI_NONE;
        gAppParamStruct.MagCaliData.MagXOffset = 0;
        gAppParamStruct.MagCaliData.MagYOffset = 0;
        gAppParamStruct.MagCaliData.MagZOffset = 0;
        gAppParamStruct.MagCaliData.MagXScale = 1.0;
        gAppParamStruct.MagCaliData.MagYScale = 1.0;
        gAppParamStruct.MagCaliData.MagZScale = 1.0;
    }
			    if(gAppParamStruct.outMagCaliData.MagCaliFlag != PARAM_CALI_DONE)
    {
        gAppParamStruct.outMagCaliData.MagCaliFlag = PARAM_CALI_NONE;
        gAppParamStruct.outMagCaliData.MagXOffset = 0;
        gAppParamStruct.outMagCaliData.MagYOffset = 0;
        gAppParamStruct.outMagCaliData.MagZOffset = 0;
        gAppParamStruct.outMagCaliData.MagXScale = 1.0;
        gAppParamStruct.outMagCaliData.MagYScale = 1.0;
        gAppParamStruct.outMagCaliData.MagZScale = 1.0;
    }

}
void SetMagCaliData(MagCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
		memcpy(&gAppParamStruct.MagCaliData, cali_data, sizeof(*cali_data));   //step1: copy data to struct
		AppParamSave();	
    }
																														 //step2:write data to the flash
}



void outSetMagCaliData(MagCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
		memcpy(&gAppParamStruct.outMagCaliData, cali_data, sizeof(*cali_data));   //step1: copy data to struct
		AppParamSave();	
    }
																														 //step2:write data to the flash
}

void mysetpid(PID_Type *cali_data)//calidata
{
		if(cali_data != NULL)
    {
			if(cali_data->motortype==PITCHO)
			{
			memcpy(&gAppParamStruct.PitchPositionPID, cali_data, sizeof(*cali_data));
						AppParamSave();	

			}
			else if(cali_data->motortype==PITCHI)
			{
			memcpy(&gAppParamStruct.PitchSpeedPID, cali_data, sizeof(*cali_data));
					AppParamSave();	

			}
			else if(cali_data->motortype==YAWO)
			{
			memcpy(&gAppParamStruct.YawPositionPID, cali_data, sizeof(*cali_data));
					AppParamSave();	

			}
			else if(cali_data->motortype==YAWI)
			{
			memcpy(&gAppParamStruct.YawSpeedPID, cali_data, sizeof(*cali_data));
					AppParamSave();	

			}
			else if(cali_data->motortype==ROLLO)
			{
			memcpy(&gAppParamStruct.RollPositionPID, cali_data, sizeof(*cali_data));
					AppParamSave();	

			}
			
			else if(cali_data->motortype==ROLLI)
			{
			memcpy(&gAppParamStruct.RollSpeedPID, cali_data, sizeof(*cali_data));
					AppParamSave();	

			}

		}
}
	


void GetMagCaliData(MagCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParamStruct.MagCaliData, sizeof(MagCaliStruct_t));
    }
}

void outGetMagCaliData(MagCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParamStruct.outMagCaliData, sizeof(MagCaliStruct_t));
    }
}

/*********************************************************************
 * @fn      CalibrateLoop
 *
 * @brief   do the calibration according to the corresponding cali flag
 *
 * @param   *flag_grp - the pointer to the cali flag group
 *
 * @return  none
 */

uint8_t IsMagCalied(void)
{
    return (gAppParamStruct.MagCaliData.MagCaliFlag == PARAM_CALI_DONE);
}

uint8_t outIsMagCalied(void)
{
    return (gAppParamStruct.outMagCaliData.MagCaliFlag == PARAM_CALI_DONE);
}

static uint32_t CaliCmdFlagGrp = 0;     //cali cmd flag group every bit represents a cali cmd received from the PC

void SetCaliCmdFlag(uint32_t flag)  //设置校准标志位
{
	CaliCmdFlagGrp |= flag;
}

void ResetCaliCmdFlag(uint32_t flag)
{
	CaliCmdFlagGrp &= ~flag;
}

uint32_t GetCaliCmdFlagGrp()
{
	return CaliCmdFlagGrp;
}

//to check whether a specfic flag if set
uint8_t IsCaliCmdFlagSet(uint32_t flag)
{
	if(flag & CaliCmdFlagGrp)
	{
		return 1;
	}else
	{
		return 0;	
	}
}



CALI_STATE_e  MagStartCaliProcess()
{	
	MagMaxMinData.MaxMagX = -4096;	//将原来的标定值清除
	MagMaxMinData.MaxMagY = -4096;
	MagMaxMinData.MaxMagZ = -4096;
	MagMaxMinData.MinMagX = 4096;
	MagMaxMinData.MinMagY = 4096;
	MagMaxMinData.MinMagZ = 4096;
	printf("Mag-cali start");

	
	return CALI_STATE_DONE;	
}

CALI_STATE_e  outMagStartCaliProcess()
{	
	outMagMaxMinData.MaxMagX = -4096;	//将原来的标定值清除
	outMagMaxMinData.MaxMagY = -4096;
	outMagMaxMinData.MaxMagZ = -4096;
	outMagMaxMinData.MinMagX = 4096;
	outMagMaxMinData.MinMagY = 4096;
	outMagMaxMinData.MinMagZ = 4096;
	printf("Mag-cali start");

	
	return CALI_STATE_DONE;	
}

CALI_STATE_e  MagEndCaliProcess()
{
	
		{
		MagCaliData.MagXOffset = (float)(MagMaxMinData.MaxMagX + MagMaxMinData.MinMagX)/2;
		MagCaliData.MagYOffset = (float)(MagMaxMinData.MaxMagY + MagMaxMinData.MinMagY)/2;
		MagCaliData.MagZOffset = (float)(MagMaxMinData.MaxMagZ + MagMaxMinData.MinMagZ)/2;
		MagCaliData.MagXScale = 1.0;
		MagCaliData.MagYScale = (float)(MagMaxMinData.MaxMagX - MagMaxMinData.MinMagX)/(float)(MagMaxMinData.MaxMagY - MagMaxMinData.MinMagY);
		MagCaliData.MagZScale = (float)(MagMaxMinData.MaxMagX - MagMaxMinData.MinMagX)/(float)(MagMaxMinData.MaxMagZ - MagMaxMinData.MinMagZ);	
		MagCaliData.MagCaliFlag = PARAM_CALI_DONE;
		printf("Mag-cali end");

		return CALI_STATE_DONE;		
	}	
}

CALI_STATE_e  outMagEndCaliProcess()
{
	
		{
		outMagCaliData.MagXOffset = (float)(outMagMaxMinData.MaxMagX + outMagMaxMinData.MinMagX)/2;
		outMagCaliData.MagYOffset = (float)(outMagMaxMinData.MaxMagY + outMagMaxMinData.MinMagY)/2;
		outMagCaliData.MagZOffset = (float)(outMagMaxMinData.MaxMagZ + outMagMaxMinData.MinMagZ)/2;
		outMagCaliData.MagXScale = 1.0;
		outMagCaliData.MagYScale = (float)(outMagMaxMinData.MaxMagX - outMagMaxMinData.MinMagX)/(float)(outMagMaxMinData.MaxMagY - outMagMaxMinData.MinMagY);
		outMagCaliData.MagZScale = (float)(outMagMaxMinData.MaxMagX - outMagMaxMinData.MinMagX)/(float)(outMagMaxMinData.MaxMagZ - outMagMaxMinData.MinMagZ);	
		outMagCaliData.MagCaliFlag = PARAM_CALI_DONE;
		printf("outMag-cali end");

		return CALI_STATE_DONE;		
	}	
}

void Sensor_Offset_Param_Init(AppParam_t *appParam)
{
	memcpy(&MagSavedCaliData, &(appParam->MagCaliData), sizeof((appParam->MagCaliData)));
	memcpy(&outMagSavedCaliData, &(appParam->outMagCaliData), sizeof((appParam->outMagCaliData)));

}


void CalibrateLoop(void)
{
    CALI_STATE_e cali_result;  
  if(IsCaliCmdFlagSet(CALI_START_FLAG_MAG))
	{
		cali_result = MagStartCaliProcess();   //reset the max min data of the magenemter
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{			
			ResetCaliCmdFlag(CALI_START_FLAG_MAG);
		}	
	}
	else if(IsCaliCmdFlagSet(CALI_END_FLAG_MAG))
	{
		cali_result = MagEndCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			SetMagCaliData(&MagCaliData);                 //set the apparamStruct using the GyroCaliData, and save apparamStruct to the flash 
			Sensor_Offset_Param_Init(&gAppParamStruct);   //update the parameter
			ResetCaliCmdFlag(CALI_END_FLAG_MAG);
		}		
	}else if(IsCaliCmdFlagSet(CALI_START_FLAG_MAG_OUT))
	{
		cali_result = outMagStartCaliProcess();   //reset the max min data of the magenemter
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{			
			ResetCaliCmdFlag(CALI_START_FLAG_MAG_OUT);
		}	
	}
	else if(IsCaliCmdFlagSet(CALI_END_FLAG_MAG_OUT))
	{
		cali_result = outMagEndCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			outSetMagCaliData(&outMagCaliData);                 //set the apparamStruct using the GyroCaliData, and save apparamStruct to the flash 
			Sensor_Offset_Param_Init(&gAppParamStruct);   //update the parameter
			ResetCaliCmdFlag(CALI_END_FLAG_MAG_OUT);
		}		
	}

}

void excallparaminit()				//call this function only in the init process
{
				Sensor_Offset_Param_Init(&gAppParamStruct);   //update the parameter

}
void cali_switch_order(u8 udata)
{
			if (udata==0x28)
		{
				SetCaliCmdFlag(CALI_START_FLAG_MAG);
		}
		else if(udata==0x29)
		{
					SetCaliCmdFlag(CALI_END_FLAG_MAG);

		}
				else if(udata==0x30)
		{
					SetCaliCmdFlag(CALI_START_FLAG_MAG_OUT);

		}
						else if(udata==0x31)
		{
					SetCaliCmdFlag(CALI_END_FLAG_MAG_OUT);

		}


}