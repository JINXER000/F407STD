#ifndef BGC32_H
#define BGC32_H
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

///////////////////////////////////////////////////////////////////////////////
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

#include "pid.h"
#include "utilities.h"
#define __BGC32_VERSION "1.0"

///////////////////////////////////////////////////////////////////////////////

#define     PI 3.14159265f
#define TWO_PI 6.28318531f

#define D2R  (PI / 180.0f)

#define R2D  (180.0f / PI)

#define SQR(x)  ((x) * (x))

#define M_TWOPI 6.28318531f

extern float   testPhase;
extern float   testPhaseDelta;

///////////////////////////////////////////////////////////////////////////////

#define ROLL     0
#define PITCH    1
#define YAW      2

#define XAXIS    0
#define YAXIS    1
#define ZAXIS    2

#define NUMAXIS  3

#define MINCOMMAND  2000
#define MIDCOMMAND  3000
#define MAXCOMMAND  4000

///////////////////////////////////////////////////////////////////////////////
// Misc Type Definitions
///////////////////////////////////////////////////////////////////////////////
typedef union
{
    int16_t value;
    uint8_t bytes[2];
} int16andUint8_t;

typedef union
{
    int32_t value;
    uint8_t bytes[4];
} int32andUint8_t;

typedef union
{
    uint16_t value;
    uint8_t bytes[2];
} uint16andUint8_t;

///////////////////////////////////////

typedef volatile uint8_t semaphore_t;

///////////////////////////////////////////////////////////////////////////////
// Sensor Variables
///////////////////////////////////////////////////////////////////////////////

typedef struct sensors_t
{
    float accel500Hz[3];
    float evvgcCFAttitude500Hz[3];
    float margAttitude500Hz[3];
    float gyro500Hz[3];
    float mag10Hz[3];

} sensors_t;

extern sensors_t sensors;

///////////////////////////////////////////////////////////////////////////////
// PID Definitions
///////////////////////////////////////////////////////////////////////////////


#define ROLL_PID  0
#define PITCH_PID 1
#define YAW_PID   2

///////////////////////////////////////////////////////////////////////////////
// MPU6000 DLPF Configurations
///////////////////////////////////////////////////////////////////////////////

enum { DLPF_256HZ, DLPF_188HZ, DLPF_98HZ, DLPF_42HZ };

///////////////////////////////////////////////////////////////////////////////
// EEPROM
///////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
#endif
