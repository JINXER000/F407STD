#ifndef PID_H
#define PID_H

/*
  Sept 2013

  bgc32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Brushless Gimbal Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the EvvGC Brushless Gimbal Controller Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

//#pragma once
#include "sys.h"
#include "bgc32.h"
///////////////////////////////////////////////////////////////////////////////

#define OTHER   false
#define ANGULAR true

#define D_ERROR true
#define D_STATE false
#define NUMBER_OF_PIDS 3

// PID Variables
typedef struct PIDdata
{
    float   B, P, I, D;
    float   iTerm;
    float   windupGuard;
    float   lastDcalcValue;
    float   lastDterm;
    float   lastLastDterm;
    uint8_t dErrorCalc;
    uint8_t type;
} PIDdata_t;

typedef struct eepromConfig_t
{
    uint8_t version;

    float accelTCBiasSlope[3];
    float accelTCBiasIntercept[3];

    float gyroTCBiasSlope[3];
    float gyroTCBiasIntercept[3];

    float magBias[3];

    float accelCutoff;

    float KpAcc;

    float KiAcc;

    float KpMag;

    float KiMag;

    uint8_t dlpfSetting;

    float midCommand;

    PIDdata_t PID[NUMBER_OF_PIDS];

    float rollPower;
    float pitchPower;
    float yawPower;

    uint8_t rollEnabled;
    uint8_t pitchEnabled;
    uint8_t yawEnabled;

    uint8_t rollAutoPanEnabled;
    uint8_t pitchAutoPanEnabled;
    uint8_t yawAutoPanEnabled;

    uint8_t imuOrientation;

    float   rollMotorPoles;
    float   pitchMotorPoles;
    float   yawMotorPoles;

    float   rateLimit;

    uint8_t rollRateCmdInput;
    uint8_t pitchRateCmdInput;
    uint8_t yawRateCmdInput;

    float   gimbalRollRate;
    float   gimbalPitchRate;
    float   gimbalYawRate;

    float   gimbalRollLeftLimit;
    float   gimbalRollRightLimit;
    float   gimbalPitchDownLimit;
    float   gimbalPitchUpLimit;
    float   gimbalYawLeftLimit;
    float   gimbalYawRightLimit;

    float   accelX500HzLowPassTau;
    float   accelY500HzLowPassTau;
    float   accelZ500HzLowPassTau;

    float   rollRatePointingCmd50HzLowPassTau;
    float   pitchRatePointingCmd50HzLowPassTau;
    float   yawRatePointingCmd50HzLowPassTau;

    float   rollAttPointingCmd50HzLowPassTau;
    float   pitchAttPointingCmd50HzLowPassTau;
    float   yawAttPointingCmd50HzLowPassTau;

} eepromConfig_t;
extern eepromConfig_t eepromConfig;

extern uint8_t holdIntegrators;

///////////////////////////////////////////////////////////////////////////////

void initPID(void);

///////////////////////////////////////////////////////////////////////////////


float updatePID(float command, float state, float deltaT, uint8_t iHold, struct PIDdata *PIDparameters);

///////////////////////////////////////////////////////////////////////////////

void setPIDintegralError(uint8_t IDPid, float value);

///////////////////////////////////////////////////////////////////////////////

void zeroPIDintegralError(void);

///////////////////////////////////////////////////////////////////////////////

void setPIDstates(uint8_t IDPid, float value);

///////////////////////////////////////////////////////////////////////////////

void zeroPIDstates(void);

///////////////////////////////////////////////////////////////////////////////


#endif
