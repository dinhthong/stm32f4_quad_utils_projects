/******************** (C) COPYRIGHT 2015 DUT ********************************

**********************************************************************************/

#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "stm32f4xx.h"
#include "IOI2C.h"
#include "delay.h"

#define HMC58X3_ADDR 0x3C // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#define HMC58X3_R_YM (7)  //!< Register address for YM.
#define HMC58X3_R_YL (8)  //!< Register address for YL.
#define HMC58X3_R_ZM (5)  //!< Register address for ZM.
#define HMC58X3_R_ZL (6)  //!< Register address for ZL.

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12

extern unsigned char HMC5883_calib;	//���ڱ궨�У�
extern int16_t  HMC5883_maxx, HMC5883_maxy, HMC5883_maxz,
       HMC5883_minx, HMC5883_miny, HMC5883_minz;

void HMC5883L_SetUp(void);	
void HMC5883L_SetUp_2(void);
void HMC58X3_init_2(u8 setmode);
void HMC58X3_setMode_2(unsigned char mode);
void HMC58X3_writeReg_2(unsigned char reg, unsigned char val);
void HMC58X3_setDOR_2(unsigned char DOR);
void HMC58X3_FIFO_init_2(void);
void HMC58X3_getID(char id[3]);	
void HMC58X3_getValues(int16_t *x, int16_t *y, int16_t *z); 
void HMC58X3_mgetValues(float *arry); 
void HMC58X3_getlastValues(int16_t *x, int16_t *y, int16_t *z);
void HMC5883L_Save_Calib(void);	 
void HMC5883L_Start_Calib(void);  
void HMC58X3_getRaw_2(int16_t *x, int16_t *y, int16_t *z);
void HMC58X3_mgetValues_2(float *arry);
void  HMC58X3_newValues_2(int16_t x, int16_t y, int16_t z);
#endif
