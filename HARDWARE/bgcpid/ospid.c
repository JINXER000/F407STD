#include "ospid.h"
#include "math.h"
#include "cali.h"
#include "mpu6050_driver.h"

//#define SINGLELOOP
#define DUALLOOP
#define ANO_CALIPID
//#define FRESHINIT
 PID_Type PitchOPID,PitchIPID,RollIPID,RollOPID,AnglePID ,YawIPID,YawOPID ;
extern volatile MPU6050_REAL_DATA   MPU6050_Real_Data;
 float pitchgoal,rollgoal, pitchnow,rollnow;
// float setanglexy,anglenow;
// short gyroxgoal,gyroygoal,gyrozgoal;
float pitcherrbias=0,rollerrbias=0;

PID_Type PitchPositionSavedPID;        	//PID offset data
PID_Type PitchSpeedSavedPID;        	//PID offset data
PID_Type YawPositionSavedPID;        	//PID offset data
PID_Type YawSpeedSavedPID;        	    //PID offset data
PID_Type RollPositionSavedPID;        	//PID offset data
PID_Type RollSpeedSavedPID;        	    //PID offset data

extern AppParam_t gAppParamStruct;	//配置信息,这里保存着最新的校准值，并且与Flash中的内容同步

void PID_READFLASH(AppParam_t *appParam)
{
	memcpy(&PitchPositionSavedPID, &(appParam->PitchPositionPID), sizeof((appParam->PitchPositionPID)));
	memcpy(&PitchSpeedSavedPID, &(appParam->PitchSpeedPID), sizeof((appParam->PitchSpeedPID)));
	memcpy(&YawPositionSavedPID, &(appParam->YawPositionPID), sizeof((appParam->YawPositionPID)));
	memcpy(&YawSpeedSavedPID, &(appParam->YawSpeedPID), sizeof((appParam->YawSpeedPID)));
	memcpy(&RollPositionSavedPID, &(appParam->RollPositionPID), sizeof((appParam->RollPositionPID)));
	memcpy(&RollSpeedSavedPID, &(appParam->RollSpeedPID), sizeof((appParam->RollSpeedPID)));

}
void PIDinitconfig()
{
#if defined ANO_CALIPID
	
		PID_READFLASH(&gAppParamStruct);
	  PitchOPID.P = PitchPositionSavedPID.P;
    PitchOPID.I = PitchPositionSavedPID.I;
    PitchOPID.D = PitchPositionSavedPID.D;
    PitchOPID.CurrentError = 0;
    PitchOPID.LastError = 0;
//    PitchOPID.LastTick = 0;
    PitchOPID.IMax = 2300;
    PitchOPID.PIDMax = 2500;
		PitchOPID.motortype=PITCHO;
	
#if defined DUALLOOP	
    PitchIPID.P = PitchSpeedSavedPID.P;
    PitchIPID.I = PitchSpeedSavedPID.I;
    PitchIPID.D = PitchSpeedSavedPID.D;
    PitchIPID.CurrentError = 0;
    PitchIPID.LastError = 0;
//    PitchIPID.LastTick = 0;
    PitchIPID.IMax = 0;
    PitchIPID.PIDMax = 5000;
		PitchIPID.motortype=PITCHI;
#endif

	  YawOPID.P = YawPositionSavedPID.P;
    YawOPID.I = YawPositionSavedPID.I;
    YawOPID.D = YawPositionSavedPID.D;
    YawOPID.CurrentError = 0;
    YawOPID.LastError = 0;
//    RollOPID.LastTick = 0;
    YawOPID.IMax = 500;
    YawOPID.PIDMax = 1200;
		YawOPID.motortype=YAWO;
#if defined DUALLOOP	
    
    YawIPID.P = YawSpeedSavedPID.P;
    YawIPID.I = YawSpeedSavedPID.I;
    YawIPID.D = YawSpeedSavedPID.D;
    YawIPID.CurrentError = 0;
    YawIPID.LastError = 0;
//    RollIPID.LastTick = 0;
    YawIPID.IMax = 0;
    YawIPID.PIDMax = 5000;
		YawIPID.motortype=YAWI;
#endif
	
		RollOPID.deadbond=10;
	  RollOPID.P = RollPositionSavedPID.P;//20
    RollOPID.I = RollPositionSavedPID.I;
    RollOPID.D = RollPositionSavedPID.D;//20
    RollOPID.CurrentError = 0;
    RollOPID.LastError = 0;
//    RollOPID.LastTick = 0;
    RollOPID.IMax = 500;
    RollOPID.PIDMax = 1200;
		RollOPID.motortype=ROLLO;
#if defined DUALLOOP	
    
    RollIPID.P = RollSpeedSavedPID.P;
    RollIPID.I = RollSpeedSavedPID.I;
    RollIPID.D = RollSpeedSavedPID.D;
    RollIPID.CurrentError = 0;
    RollIPID.LastError = 0;
//    RollIPID.LastTick = 0;
    RollIPID.IMax = 0;
    RollIPID.PIDMax = 1200;
		RollIPID.motortype=ROLLI;
#endif
	
#elif defined FRESHINIT
	  PitchOPID.P = 0.1;
    PitchOPID.I = 0;
    PitchOPID.D = 0;
    PitchOPID.CurrentError = 0;
    PitchOPID.LastError = 0;
//    PitchOPID.LastTick = 0;
    PitchOPID.IMax = 2300;
    PitchOPID.PIDMax = 2500;
		PitchOPID.motortype=PITCHO;
	
#if defined DUALLOOP	
    PitchIPID.P = 80;
    PitchIPID.I = 0;
    PitchIPID.D = 20;
    PitchIPID.CurrentError = 0;
    PitchIPID.LastError = 0;
//    PitchIPID.LastTick = 0;
    PitchIPID.IMax = 0;
    PitchIPID.PIDMax = 5000;
		PitchIPID.motortype=PITCHI;
#endif

	  YawOPID.P = 70;
    YawOPID.I = 0;
    YawOPID.D = 100;
    YawOPID.CurrentError = 0;
    YawOPID.LastError = 0;
//    RollOPID.LastTick = 0;
    YawOPID.IMax = 500;
    YawOPID.PIDMax = 1200;
		YawOPID.motortype=YAWO;
#if defined DUALLOOP	
    
    YawIPID.P = 80;
    YawIPID.I = 0;
    YawIPID.D = 20;
    YawIPID.CurrentError = 0;
    YawIPID.LastError = 0;
//    RollIPID.LastTick = 0;
    YawIPID.IMax = 0;
    YawIPID.PIDMax = 5000;
		YawIPID.motortype=YAWI;
#endif

		RollOPID.deadbond=10;
	  RollOPID.P = 10;//20
    RollOPID.I = 0;
    RollOPID.D = 20;//20
    RollOPID.CurrentError = 0;
    RollOPID.LastError = 0;
//    RollOPID.LastTick = 0;
    RollOPID.IMax = 500;
    RollOPID.PIDMax = 1200;
		RollOPID.motortype=ROLLO;
#if defined DUALLOOP	
    
    RollIPID.P = 3;
    RollIPID.I = 0;
    RollIPID.D = 0;
    RollIPID.CurrentError = 0;
    RollIPID.LastError = 0;
//    RollIPID.LastTick = 0;
    RollIPID.IMax = 0;
    RollIPID.PIDMax = 1200;
		RollIPID.motortype=ROLLI;
#endif

#endif
	


}

int16_t Control_PitchPID(void)
{
	
	PitchOPID.CurrentError=pitchgoal-pitchnow-pitcherrbias;
	PitchOPID.Pout = PitchOPID.P * PitchOPID.CurrentError;
	
	if (PitchOPID.CurrentError>10)
	{
		PitchOPID.index=0;
	}
	else
	{
		PitchOPID.index=1;
		PitchOPID.Iout += PitchOPID.I * PitchOPID.CurrentError;
	PitchOPID.Iout = PitchOPID.Iout > PitchOPID.IMax ? PitchOPID.IMax : PitchOPID.Iout;
	PitchOPID.Iout = PitchOPID.Iout < -PitchOPID.IMax ? -PitchOPID.IMax : PitchOPID.Iout;
	}
	
		PitchOPID.Dout = PitchOPID.D *(PitchOPID.CurrentError-PitchOPID.LastError);//+-?
	
	PitchOPID.PIDout = PitchOPID.Pout +PitchOPID.index* PitchOPID.Iout + PitchOPID.Dout;
	PitchOPID.PIDout = PitchOPID.PIDout > PitchOPID.PIDMax ? PitchOPID.PIDMax : PitchOPID.PIDout;
	PitchOPID.PIDout = PitchOPID.PIDout < -PitchOPID.PIDMax ? -PitchOPID.PIDMax : PitchOPID.PIDout;
	
		PitchOPID.LastError = PitchOPID.CurrentError;
#if defined SINGLELOOP
return (short)PitchOPID.PIDout;
#elif defined DUALLOOP
	/***************************************	内环	******************************************/

//	PitchIPID.CurrentError = PitchOPID.PIDout - Position.Real.OX;	
////	PitchIPID.CurrentError = DBUS_ReceiveData.ch4 - Position.Real.OX;
//	
//	PitchIPID.Pout = PitchIPID.P * PitchIPID.CurrentError;
//	
//	PitchIPID.Iout += PitchIPID.I * PitchIPID.CurrentError;
//	PitchIPID.Iout = PitchIPID.Iout > PitchIPID.IMax ? PitchIPID.IMax : PitchIPID.Iout;
//	PitchIPID.Iout = PitchIPID.Iout < -PitchIPID.IMax ? -PitchIPID.IMax : PitchIPID.Iout;
//	
//	if(PitchIPID.LastTick != CurrentTick)
//	{
//        PitchIPID.Dout = PitchIPID.D * (PitchIPID.LastError - PitchIPID.CurrentError) * 5 / (CurrentTick - PitchIPID.LastTick);
//    }
//    else
//    {
//        PitchIPID.Dout = PitchIPID.D * (PitchIPID.LastError - PitchIPID.CurrentError);
//    }
//	
//	PitchIPID.PIDout = (PitchIPID.Pout + PitchIPID.Iout + PitchIPID.Dout);
//	PitchIPID.PIDout = PitchIPID.PIDout > PitchIPID.PIDMax ? PitchIPID.PIDMax : PitchIPID.PIDout;
//	PitchIPID.PIDout = PitchIPID.PIDout < -PitchIPID.PIDMax ? -PitchIPID.PIDMax : PitchIPID.PIDout;
//	
//	PitchIPID.LastError = PitchIPID.CurrentError;
//	PitchIPID.LastTick = CurrentTick;

	

return (short)PitchIPID.PIDout;
#endif
}

int16_t Control_RollPID(void)
{
	RollOPID.CurrentError=rollgoal-rollnow-rollerrbias;
	RollOPID.Pout = RollOPID.P * RollOPID.CurrentError;
	
		if (RollOPID.CurrentError>10)
	{
		RollOPID.index=0;
	}
	else
	{
		RollOPID.index=1;
		RollOPID.Iout += RollOPID.I * RollOPID.CurrentError;
	RollOPID.Iout = RollOPID.Iout > RollOPID.IMax ? RollOPID.IMax : RollOPID.Iout;
	RollOPID.Iout = RollOPID.Iout < -RollOPID.IMax ? -RollOPID.IMax : RollOPID.Iout;
	}


		RollOPID.Dout = RollOPID.D *(RollOPID.CurrentError-RollOPID.LastError);
	
	RollOPID.PIDout = RollOPID.Pout + RollOPID.index*RollOPID.Iout + RollOPID.Dout;
	RollOPID.PIDout = RollOPID.PIDout > RollOPID.PIDMax ? RollOPID.PIDMax : RollOPID.PIDout;
	RollOPID.PIDout = RollOPID.PIDout < -RollOPID.PIDMax ? -RollOPID.PIDMax : RollOPID.PIDout;
	
		RollOPID.LastError = RollOPID.CurrentError;
		
if(fabs(RollOPID.CurrentError)<RollOPID.deadbond)	RollOPID.PIDout=0;
	
#if defined SINGLELOOP

	return (short)RollOPID.PIDout;
#elif defined DUALLOOP
	/***************************************	内环	******************************************/

	RollIPID.CurrentError = RollOPID.PIDout- MPU6050_Real_Data.Gyro_X;	
//	PitchIPID.CurrentError = DBUS_ReceiveData.ch4 - Position.Real.OX;
	
	RollIPID.Pout = RollIPID.P * RollIPID.CurrentError;
	
	RollIPID.Iout += RollIPID.I * RollIPID.CurrentError;
	RollIPID.Iout = RollIPID.Iout > RollIPID.IMax ? RollIPID.IMax : RollIPID.Iout;
	RollIPID.Iout = RollIPID.Iout < -RollIPID.IMax ? -RollIPID.IMax : RollIPID.Iout;
	
//	if(RollIPID.LastTick != CurrentTick)
//	{
//        PitchIPID.Dout = PitchIPID.D * (PitchIPID.LastError - PitchIPID.CurrentError) * 5 / (CurrentTick - PitchIPID.LastTick);
//    }
//    else
//    {
        RollIPID.Dout = RollIPID.D * (RollIPID.LastError - RollIPID.CurrentError);
//    }
	
	RollIPID.PIDout = (RollIPID.Pout + RollIPID.Iout + RollIPID.Dout);
	RollIPID.PIDout = RollIPID.PIDout > RollIPID.PIDMax ? RollIPID.PIDMax : RollIPID.PIDout;
	RollIPID.PIDout = RollIPID.PIDout < -RollIPID.PIDMax ? -RollIPID.PIDMax : RollIPID.PIDout;
	
	RollIPID.LastError = RollIPID.CurrentError;
//	RollIPID.LastTick = CurrentTick;

	

return (short)RollIPID.PIDout;
#endif

}

int16_t keepangle()
{
//	AnglePID.CurrentError=setanglexy-anglenow;		
//	AnglePID.Pout=AnglePID.P*AnglePID.CurrentError;
//	
//		if (AnglePID.CurrentError>5)
//	{
//		AnglePID.index=0;
//	}
//	else
//	{
//		AnglePID.index=1;
//		AnglePID.Iout += AnglePID.I * AnglePID.CurrentError;
//	AnglePID.Iout = AnglePID.Iout > AnglePID.IMax ? AnglePID.IMax : AnglePID.Iout;
//	AnglePID.Iout = AnglePID.Iout < -AnglePID.IMax ? -AnglePID.IMax : AnglePID.Iout;
//	}


//		AnglePID.Dout = AnglePID.D *(AnglePID.CurrentError-AnglePID.LastError);
//	
//	AnglePID.PIDout = AnglePID.Pout + AnglePID.index*AnglePID.Iout + AnglePID.Dout;
//	AnglePID.PIDout = AnglePID.PIDout > AnglePID.PIDMax ? AnglePID.PIDMax : AnglePID.PIDout;
//	AnglePID.PIDout = AnglePID.PIDout < -AnglePID.PIDMax ? -AnglePID.PIDMax : AnglePID.PIDout;
//	
//		AnglePID.LastError = AnglePID.CurrentError;

//	return (short)RollOPID.PIDout;

	
}

//int16_t Increment_PitchPID(int pwmx)
//{
//		RollOPID.CurrentError=rollgoal-roll-rollerrbias;
//		RollOPID.Pout=RollOPID.P*(RollOPID.CurrentError-RollOPID.LastError);
//			RollOPID.Iout= RollOPID.I*RollOPID.CurrentError;
//	RollOPID.Dout=RollOPID.D*(RollOPID.CurrentError-2*RollOPID.LastError+RollOPID.NextError);
//	RollOPID.PIDout = RollOPID.Pout + RollOPID.Iout + RollOPID.Dout;
//	pwmx+=RollOPID.PIDout;
//	
//	RollOPID.LastError=RollOPID.CurrentError;
//	RollOPID.NextError=RollOPID.LastError;
//	return pwmx;

//}

