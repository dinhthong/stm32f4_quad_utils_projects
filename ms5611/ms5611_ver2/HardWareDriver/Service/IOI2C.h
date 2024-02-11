/******************** (C) COPYRIGHT 2015 DUT ********************************
 * ����    �����Ĳ�
 * �ļ���  ��IOI2C.h
 * ����    ��ģ��IIC����ͷ�ļ�
 * ����    ��2015/11/30 12:43:38
 * ��ϵ��ʽ��1461318172��qq��
**********************************************************************************/

#ifndef __IOI2C_H
#define __IOI2C_H
#include "stm32f4xx.h"
#include "sys.h"

//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

//IO�ڵ�ַӳ��
//#define GPIOB_ODR_Addr    (GPIOB_BASE+0x14) //0x40010C14 
//#define GPIOC_ODR_Addr    (GPIOC_BASE+0x14) //0x40011014 

//#define GPIOB_IDR_Addr    (GPIOB_BASE+0x10) //0x40010C10 
//#define GPIOC_IDR_Addr    (GPIOC_BASE+0x10) //0x40011010 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

//IO��������
#define SDA_IN()  {GPIOB->MODER&=0xFFFF3FFF;GPIOB->MODER|=0x00000000;}
#define SDA_OUT() {GPIOB->MODER&=0xFFFF3FFF;GPIOB->MODER|=0x00004000;}


//IO��������
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //����SDA 

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�


extern int16_t  I2C_Erorr_Count;

void IIC_Write_One_Byte(u8 daddr, u8 addr, u8 data);
u8 IIC_Read_One_Byte(u8 daddr, u8 addr);
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8 *data);
u8 IICwriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data);
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

#endif

//------------------End of File----------------------------
