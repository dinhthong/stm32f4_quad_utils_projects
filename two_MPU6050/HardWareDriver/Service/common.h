/******************** (C) COPYRIGHT 2015 DUT ********************************
 * ����    �����Ĳ�
 * �ļ���  ��common.h
 * ����    ������������������ͷ�ļ�
 * ����    ��2015/11/30 12:43:38
 * ��ϵ��ʽ��1461318172��qq��
**********************************************************************************/


#ifndef __common_H
#define __common_H
#include "IOI2C.h"

#include "delay.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "ms5611.h"
#include "report.h"
#include "IMU.h"
//#include "AT45DB.h"


//���� ������
typedef union
{
	float  value;
	unsigned char byte[4];
} f_bytes;

//���� ������
typedef union
{
	int16_t  value;
	unsigned char byte[2];
} i_bytes;


#endif

//------------------End of File----------------------------



