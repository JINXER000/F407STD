#ifndef  CALI_H
#define CALI_H

#include "sys.h"

#define VERSION_A								1u
#define VERSION_B								6u
#define VERSION_C								4u
#define VERSION_D								0u
#define VERSION									(VERSION_A<<24)|(VERSION_B<<16)|(VERSION_C<<8)|(VERSION_D)

#define PARAM_SAVED_START_ADDRESS 								ADDR_FLASH_SECTOR_11
//enum the cali result
typedef enum
{
    CALI_STATE_ERR,
    CALI_STATE_IN,
    CALI_STATE_DONE,
}CALI_STATE_e;

typedef struct Version
{
		uint8_t A;   //main version number
		uint8_t B;	//sub version number
		uint8_t C;	
		uint8_t D;	// test version number
}Version;

#define VERSION_DEFAULT	\
{\
	1,\
	6,\
	2,\
	0,\
}\

#define PARAM_SAVED_FLAG                            0x5A   //header of the structure
#define PARAM_CALI_DONE                             0x5A 		
#define PARAM_CALI_NONE                             0x00

#define CALI_START_FLAG_GYRO                  ((uint32_t)1<<1)
#define CALI_END_FLAG_GYRO                    ((uint32_t)1<<2)
#define CALI_START_FLAG_ACC                   ((uint32_t)1<<3)
#define CALI_START_FLAG_MAG                   ((uint32_t)1<<4)
#define CALI_END_FLAG_MAG                     ((uint32_t)1<<5)
#define CALI_START_FLAG_GIMBAL                ((uint32_t)1<<6)
#define CALI_END_FLAG_GIMBAL                  ((uint32_t)1<<7)
#define CALI_FLAG_PID         				  ((uint32_t)1<<8)
#define CALI_FLAG_PITCH_SPEED_PID             ((uint32_t)1<<9)
#define CALI_FLAG_YAW_POSITION_PID            ((uint32_t)1<<10)
#define CALI_FLAG_YAW_SPEED_PID               ((uint32_t)1<<11)
#define CALI_START_FLAG_MAG_OUT 							((uint32_t)1<<12)
#define CALI_END_FLAG_MAG_OUT									((uint32_t)1<<13)

typedef __packed struct
{
    int16_t     GimbalYawOffset;
    int16_t     GimbalPitchOffset;
    uint8_t     GimbalCaliFlag;
}GimbalCaliStruct_t;

typedef __packed struct
{
    int16_t     GyroXOffset;
    int16_t     GyroYOffset;
    int16_t     GyroZOffset;
    uint8_t     GyroCaliFlag;
}GyroCaliStruct_t;

typedef __packed struct
{
    int16_t     AccXOffset;
    int16_t     AccYOffset;
    int16_t     AccZOffset; 
    float       AccXScale;
    float       AccYScale;
    float       AccZScale;
    uint8_t     AccCaliFlag;
}AccCaliStruct_t;

typedef __packed struct
{
    int16_t     MagXOffset;
    int16_t     MagYOffset;
    int16_t     MagZOffset;
    float       MagXScale;
    float       MagYScale;
    float       MagZScale;    
    uint8_t     MagCaliFlag;
}MagCaliStruct_t;

typedef __packed struct
{
	int8_t pid_type;		// position PID
	int8_t motor_type;   //motor type ie: pitch yaw 201 202 203 204	
	int16_t kp_offset;
	int16_t ki_offset;
	int16_t kd_offset;
}PIDParamStruct_t;

typedef __packed struct 
{
    uint8_t     ParamSavedFlag;    				//header 
    uint32_t    FirmwareVersion;    			//version
    GimbalCaliStruct_t GimbalCaliData;    //gimbal pitch yaw encoder offset
    GyroCaliStruct_t   GyroCaliData;      //gyro offset data
    AccCaliStruct_t    AccCaliData;    		//ACC offset data
    MagCaliStruct_t    MagCaliData;				//Mag offset data
	MagCaliStruct_t outMagCaliData;					//2nd Mag 
	PIDParamStruct_t   PitchPositionPID;
	PIDParamStruct_t   PitchSpeedPID;
	PIDParamStruct_t   YawPositionPID;
	PIDParamStruct_t   YawSpeedPID;
}AppParam_t;
//上传数据的类型
typedef enum
{
	REIMU = 1,
	REMOV = 2,
	REHMC = 3,
	REOFFSET = 4,
	REVERSION = 5,
	REERROR =6,
	REPID =7,
}UploadParamType_e;

static uint8_t AppParamSave(void);
void AppParamInit(void);
void SetMagCaliData(MagCaliStruct_t *cali_data);
void GetMagCaliData(MagCaliStruct_t *cali_data);
uint8_t IsMagCalied(void);
void SetCaliCmdFlag(uint32_t flag);  //设置校准标志位
void ResetCaliCmdFlag(uint32_t flag);
uint32_t GetCaliCmdFlagGrp(void);
uint8_t IsCaliCmdFlagSet(uint32_t flag);
CALI_STATE_e  MagStartCaliProcess(void);
CALI_STATE_e  MagEndCaliProcess(void);
void Sensor_Offset_Param_Init(AppParam_t *appParam);
void CalibrateLoop(void);
void excallparaminit(void);				//call this function only in the init process


#endif
