/******************** (C) COPYRIGHT 2015 DUT ********************************

**********************************************************************************/


#ifndef __common_H
#define __common_H
#include "IOI2C.h"

#include "delay.h"
//#include "MPU6050.h"
//#include "HMC5883L.h"
//#include "ms5611.h"
//#include "IMU.h"
#include "Kalman.h"
#include "copter_config.h"
//#include "motor.h"
//#include "pid.h"
#include "stm32f4xx_it.h"
//#include "adns3080.h"
//浮点 联合体
typedef union
{
	float  value;
	unsigned char byte[4];
} f_bytes;

//整数 联合体
typedef union
{
	int16_t  value;
	unsigned char byte[2];
} i_bytes;

typedef struct {
    float p,i,d;
} pid_para_t;

#endif

//------------------End of File----------------------------



