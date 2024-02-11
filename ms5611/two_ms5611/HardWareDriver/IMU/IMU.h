#ifndef __IMU_H
#define __IMU_H

#include <math.h>
#include "common.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define M_PI  (float)3.1415926535
#define micros() TIM5->CNT
extern float IMU_values[9];
extern int16_t MPU6050_raw[6];
extern volatile float IMU_Pitch, IMU_Roll, IMU_Yaw, ACC_Pitch, ACC_Roll;	 //��λ ��

extern volatile float IMU_GYROx, IMU_GYROy, IMU_GYROz;	
extern volatile float acc_vector;  
//Mini IMU AHRS 
void IMU_getValues(float *values);
void IMU_init(void);
//void Initialize_Q(void);
//void IMU_getRollPitchYaw(float *angles);
float LPF(float x,float pre_value, float CUTOFF,float dt);
void IMU_getAttitude(float *RPY,float *rate_RPY, float *cf_accZ);

#endif

