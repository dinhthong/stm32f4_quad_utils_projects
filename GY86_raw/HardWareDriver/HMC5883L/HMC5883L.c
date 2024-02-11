/******************** (C) COPYRIGHT 2015 DUT ********************************

**********************************************************************************/
#include "HMC5883L.h"
#include "common.h"

#define  HMC5883_Buf_Size 10

float HMC5883_lastx, HMC5883_lasty, HMC5883_lastz;

//�����Ʊ궨ֵ
int16_t HMC5883_Offset_X = 0,
	HMC5883_Offset_Y = 0,
	HMC5883_Offset_Z = 0;

float  HMC5883_Scale_X = 1.0f,
       HMC5883_Scale_Y = 1.0f,
       HMC5883_Scale_Z = 1.0f;
//��ǰ�ų������ֵ����Сֵ
int16_t  HMC5883_maxx = 0, HMC5883_maxy = 0, HMC5883_maxz = 0,
	 HMC5883_minx = -0, HMC5883_miny = -0, HMC5883_minz = -0;
unsigned char HMC5883_calib = 0; //��ʼ����ɱ�־

int8_t  HMC5883_Buf_index = 0;
int8_t  HMC5883_Buf_index_2 = 0;
int16_t  HMC5883_FIFO[3][11]; //�������˲�
int16_t  HMC5883_FIFO_2[3][11]; //�������˲�
void HMC58X3_getRaw(int16_t *x, int16_t *y, int16_t *z);

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
unsigned char HMC5883_IS_newdata(void)
{
	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) == Bit_SET)
	{
		return 1;
	}
	else return 0;
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void HMC58X3_FIFO_init(void)
{
	int16_t temp[3];
	unsigned char i;
	for(i = 0; i < 50; i++)
	{
		HMC58X3_getRaw(&temp[0], &temp[1], &temp[2]);
		delay_us(200);  //��ʱ�ٶ�ȡ����

	}
}

void HMC58X3_FIFO_init_2(void)
{
	int16_t temp[3];
	unsigned char i;
	for(i = 0; i < 50; i++)
	{
		HMC58X3_getRaw_2(&temp[0], &temp[1], &temp[2]);
		delay_us(200);  //��ʱ�ٶ�ȡ����

	}
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void  HMC58X3_newValues(int16_t x, int16_t y, int16_t z)
{
	unsigned char i ;
	int32_t sum = 0;

	HMC5883_FIFO[0][HMC5883_Buf_index] = x;
	HMC5883_FIFO[1][HMC5883_Buf_index] = y;
	HMC5883_FIFO[2][HMC5883_Buf_index] = z;
	HMC5883_Buf_index = (HMC5883_Buf_index + 1) % HMC5883_Buf_Size;//ѭ��

	sum = 0;
	for(i = 0; i < 10; i++) 	//ȡ�����ڵ�ֵ���������ȡƽ��
	{
		sum += HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10] = sum / 10;	//��ƽ��ֵ����

	sum = 0;
	for(i = 0; i < 10; i++)
	{
		sum += HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10] = sum / 10;

	sum = 0;
	for(i = 0; i < 10; i++)
	{
		sum += HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10] = sum / 10;

	if(HMC5883_calib) //У����Ч�Ļ� �ɼ��궨ֵ
	{
		if(HMC5883_minx > HMC5883_FIFO[0][10])HMC5883_minx = (int16_t)HMC5883_FIFO[0][10];
		if(HMC5883_miny > HMC5883_FIFO[1][10])HMC5883_miny = (int16_t)HMC5883_FIFO[1][10];
		if(HMC5883_minz > HMC5883_FIFO[2][10])HMC5883_minz = (int16_t)HMC5883_FIFO[2][10];

		if(HMC5883_maxx < HMC5883_FIFO[0][10])HMC5883_maxx = (int16_t)HMC5883_FIFO[0][10];
		if(HMC5883_maxy < HMC5883_FIFO[1][10])HMC5883_maxy = (int16_t)HMC5883_FIFO[1][10];
		if(HMC5883_maxz < HMC5883_FIFO[2][10])HMC5883_maxz = (int16_t)HMC5883_FIFO[2][10];

	}

} //HMC58X3_newValues
//HMC5883_Buf_index_2;
void  HMC58X3_newValues_2(int16_t x, int16_t y, int16_t z)
{
	unsigned char i ;
	int32_t sum = 0;

	HMC5883_FIFO_2[0][HMC5883_Buf_index_2] = x;
	HMC5883_FIFO_2[1][HMC5883_Buf_index_2] = y;
	HMC5883_FIFO_2[2][HMC5883_Buf_index_2] = z;
	HMC5883_Buf_index_2 = (HMC5883_Buf_index_2 + 1) % HMC5883_Buf_Size;//ѭ��

	sum = 0;
	for(i = 0; i < 10; i++) 	//ȡ�����ڵ�ֵ���������ȡƽ��
	{
		sum += HMC5883_FIFO_2[0][i];
	}
	HMC5883_FIFO_2[0][10] = sum / 10;	//��ƽ��ֵ����

	sum = 0;
	for(i = 0; i < 10; i++)
	{
		sum += HMC5883_FIFO_2[1][i];
	}
	HMC5883_FIFO_2[1][10] = sum / 10;

	sum = 0;
	for(i = 0; i < 10; i++)
	{
		sum += HMC5883_FIFO_2[2][i];
	}
	HMC5883_FIFO_2[2][10] = sum / 10;

//	if(HMC5883_calib) //У����Ч�Ļ� �ɼ��궨ֵ
//	{
//		if(HMC5883_minx > HMC5883_FIFO[0][10])HMC5883_minx = (int16_t)HMC5883_FIFO[0][10];
//		if(HMC5883_miny > HMC5883_FIFO[1][10])HMC5883_miny = (int16_t)HMC5883_FIFO[1][10];
//		if(HMC5883_minz > HMC5883_FIFO[2][10])HMC5883_minz = (int16_t)HMC5883_FIFO[2][10];

//		if(HMC5883_maxx < HMC5883_FIFO[0][10])HMC5883_maxx = (int16_t)HMC5883_FIFO[0][10];
//		if(HMC5883_maxy < HMC5883_FIFO[1][10])HMC5883_maxy = (int16_t)HMC5883_FIFO[1][10];
//		if(HMC5883_maxz < HMC5883_FIFO[2][10])HMC5883_maxz = (int16_t)HMC5883_FIFO[2][10];

//	}

} //HMC58X3_newValues

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void HMC58X3_writeReg(unsigned char reg, unsigned char val)
{
	IICwriteByte(HMC58X3_ADDR, reg, val);
}

void HMC58X3_writeReg_2(unsigned char reg, unsigned char val)
{
	IIC3writeByte(HMC58X3_ADDR, reg, val);
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void HMC58X3_getRaw(int16_t *x, int16_t *y, int16_t *z)
{
	unsigned char vbuff[6];
	vbuff[0] = vbuff[1] = vbuff[2] = vbuff[3] = vbuff[4] = vbuff[5] = 0;
	IICreadBytes(HMC58X3_ADDR, HMC58X3_R_XM, 6, vbuff);
	HMC58X3_newValues(((int16_t)vbuff[0] << 8) | vbuff[1], ((int16_t)vbuff[4] << 8) | vbuff[5], ((int16_t)vbuff[2] << 8) | vbuff[3]);
	*x = HMC5883_FIFO[0][10];
	*y = HMC5883_FIFO[1][10];
	*z = HMC5883_FIFO[2][10];
}

void HMC58X3_getRaw_2(int16_t *x, int16_t *y, int16_t *z)
{
	unsigned char vbuff[6];
//	vbuff[0] = vbuff[1] = vbuff[2] = vbuff[3] = vbuff[4] = vbuff[5] = 0;
	IIC3readBytes(HMC58X3_ADDR, HMC58X3_R_XM, 6, vbuff);
	HMC58X3_newValues_2(((int16_t)vbuff[0] << 8) | vbuff[1], ((int16_t)vbuff[4] << 8) | vbuff[5], ((int16_t)vbuff[2] << 8) | vbuff[3]);
	*x = HMC5883_FIFO_2[0][10];
	*y = HMC5883_FIFO_2[1][10];
	*z = HMC5883_FIFO_2[2][10];
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void HMC58X3_getlastValues(int16_t *x, int16_t *y, int16_t *z)
{
	*x = HMC5883_FIFO[0][10];
	*y = HMC5883_FIFO[1][10];
	*z = HMC5883_FIFO[2][10];
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void HMC58X3_mgetValues(float *arry)
{
	int16_t xr, yr, zr;
	HMC58X3_getRaw(&xr, &yr, &zr);
	arry[0] = HMC5883_lastx = (float)(((float)(xr - HMC5883_Offset_X)) * HMC5883_Scale_X);
	arry[1] = HMC5883_lasty = (float)(((float)(yr - HMC5883_Offset_Y)) * HMC5883_Scale_Y);
	arry[2] = HMC5883_lastz = (float)(((float)(zr - HMC5883_Offset_Z)) * HMC5883_Scale_Z);
}

void HMC58X3_mgetValues_2(float *arry)
{
	int16_t xr, yr, zr;
	HMC58X3_getRaw_2(&xr, &yr, &zr);
	arry[0] =  (float)(((float)(xr - HMC5883_Offset_X)) * HMC5883_Scale_X);
	arry[1] = (float)(((float)(yr - HMC5883_Offset_Y)) * HMC5883_Scale_Y);
	arry[2] = (float)(((float)(zr - HMC5883_Offset_Z)) * HMC5883_Scale_Z);
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void HMC58X3_setGain(unsigned char gain)
{
	// 0-7, 1 default
	if (gain > 7) return;
	HMC58X3_writeReg(HMC58X3_R_CONFB, gain << 5);
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void HMC58X3_setMode(unsigned char mode)
{
	if (mode > 2)
	{
		return;
	}
	HMC58X3_writeReg(HMC58X3_R_MODE, mode);
	delay_us(100);
}

void HMC58X3_setMode_2(unsigned char mode)
{
	if (mode > 2)
	{
		return;
	}
	HMC58X3_writeReg_2(HMC58X3_R_MODE, mode);
	delay_us(100);
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void HMC58X3_init(u8 setmode)
{

//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	//Ӧ�����á���GPIOC��
//	GPIO_Init(GPIOC, &GPIO_InitStructure);

	if (setmode)
	{
		HMC58X3_setMode(0);
	}

	HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
	HMC58X3_writeReg(HMC58X3_R_CONFB, 0xA0);
	HMC58X3_writeReg(HMC58X3_R_MODE, 0x00);

}

void HMC58X3_init_2(u8 setmode)
{

//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	//Ӧ�����á���GPIOC��
//	GPIO_Init(GPIOC, &GPIO_InitStructure);

	if (setmode)
	{
		HMC58X3_setMode_2(0);
	}

	HMC58X3_writeReg_2(HMC58X3_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
	HMC58X3_writeReg_2(HMC58X3_R_CONFB, 0xA0);
	HMC58X3_writeReg_2(HMC58X3_R_MODE, 0x00);

}

/**************************ʵ�ֺ���********************************************

0 -> 0.75Hz  |   1 -> 1.5Hz
2 -> 3Hz     |   3 -> 7.5Hz
4 -> 15Hz    |   5 -> 30Hz
6 -> 75Hz

*******************************************************************************/
void HMC58X3_setDOR(unsigned char DOR)
{
	if (DOR > 6) return;
	HMC58X3_writeReg(HMC58X3_R_CONFA, DOR << 2);
}

void HMC58X3_setDOR_2(unsigned char DOR)
{
	if (DOR > 6) return;
	HMC58X3_writeReg_2(HMC58X3_R_CONFA, DOR << 2);
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void HMC58X3_getID(char id[3])
{
	id[0] = I2C_ReadOneByte(HMC58X3_ADDR, HMC58X3_R_IDA);
	id[1] = I2C_ReadOneByte(HMC58X3_ADDR, HMC58X3_R_IDB);
	id[2] = I2C_ReadOneByte(HMC58X3_ADDR, HMC58X3_R_IDC);
}   // getID().

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void HMC5883L_SetUp(void)
{
	HMC58X3_init(0); // Don't set mode yet, we'll do that later on.
	HMC58X3_setMode(0);
	HMC58X3_setDOR(6);  //75hz ������
	HMC58X3_FIFO_init();
}

void HMC5883L_SetUp_2(void)
{
	HMC58X3_init_2(0); // Don't set mode yet, we'll do that later on.
	HMC58X3_setMode_2(0);
	HMC58X3_setDOR_2(6);  //75hz ������
	HMC58X3_FIFO_init_2();
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void HMC5883L_Start_Calib(void)
{
	HMC5883_calib = 1; //��ʼ�궨
	HMC5883_maxx = 0;	//��ԭ���ı궨ֵ���
	HMC5883_maxy = 0;
	HMC5883_maxz = 0;
	HMC5883_minx = -0;
	HMC5883_miny = -0;
	HMC5883_minz = -0;
}


//------------------End of File----------------------------
