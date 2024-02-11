/******************** (C) COPYRIGHT 2015 DUT ********************************
  *1. I2C �ӿ� PB6  PB7 ����
********************************************************************************
 */
#include "IOI2C.h"
#include "delay.h"


int16_t  I2C_Erorr_Count;
int16_t  I2C2_Erorr_Count;
int16_t  I2C3_Erorr_Count;
/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	//����PB6 PB7 Ϊ��©���  ˢ��Ƶ��Ϊ50Mhz
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//Ӧ�����õ�GPIOB
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA = 1;
	IIC_SCL = 1;
	delay_us(2);
	IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
	delay_us(2);
	IIC_SCL = 0; //ǯסI2C���ߣ�׼�����ͻ��������
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL = 0;
	IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
	delay_us(2);
	IIC_SCL = 1;
	IIC_SDA = 1; //����I2C���߽����ź�
	delay_us(2);
}


u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	SDA_IN();      //SDA����Ϊ����
	IIC_SDA = 1;
	delay_us(1);
	IIC_SCL = 1;
	delay_us(1);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime > 50)
		{
			IIC_Stop();
			I2C_Erorr_Count ++;	 //I2C ����
			return 1;
		}
		delay_us(1);
	}
	IIC_SCL = 0; //ʱ�����0
	return 0;
}


void IIC_Ack(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 0;
	delay_us(1);
	IIC_SCL = 1;
	delay_us(1);
	IIC_SCL = 0;
}


void IIC_NAck(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 1;
	delay_us(1);
	IIC_SCL = 1;
	delay_us(1);
	IIC_SCL = 0;
}


void IIC_Send_Byte(u8 txd)
{
	u8 t;
	SDA_OUT();
	IIC_SCL = 0; //����ʱ�ӿ�ʼ���ݴ���
	for(t = 0; t < 8; t++)
	{
		IIC_SDA = (txd & 0x80) >> 7;
		txd <<= 1;
		delay_us(1);
		IIC_SCL = 1;
		delay_us(1);
		IIC_SCL = 0;
		delay_us(1);
	}
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA_IN();//SDA����Ϊ����
	for(i = 0; i < 8; i++ )
	{
		IIC_SCL = 0;
		delay_us(1);
		IIC_SCL = 1;
		receive <<= 1;
		if(READ_SDA)receive++;
		delay_us(1);
	}
	if (ack)
		IIC_Ack(); //����ACK
	else
		IIC_NAck();//����nACK
	return receive;
}


/**************************ʵ�ֺ���********************************************

*******************************************************************************/
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
	unsigned char res = 0;

	IIC_Start();
	IIC_Send_Byte(I2C_Addr);	   //����д����
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	res++;  //���͵�ַ
	IIC_Wait_Ack();
	//IIC_Stop();//����һ��ֹͣ����
	IIC_Start();
	IIC_Send_Byte(I2C_Addr + 1);
	res++;          //�������ģʽ
	IIC_Wait_Ack();
	res = IIC_Read_Byte(0);
	IIC_Stop();//����һ��ֹͣ����

	return res;
}


/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
	u8 count = 0;

	IIC_Start();
	IIC_Send_Byte(dev);	   // Send address with Transmitter mode
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(dev + 1); //�Send address with Receiver mode
	IIC_Wait_Ack();

	for(count = 0; count < length; count++)
	{

		if(count != length - 1)data[count] = IIC_Read_Byte(1); //�read byte with ACK
		else  data[count] = IIC_Read_Byte(0);	 //���һ���ֽ�NACK
	}
	IIC_Stop();//����һ��ֹͣ����
	return count;
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8 *data)
{

	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	//for MS5611
	if (reg!=0)
	{
	IIC_Send_Byte(reg);   //���͵�ַ
	IIC_Wait_Ack();
	}
	for(count = 0; count < length; count++)
	{
		IIC_Send_Byte(data[count]);
		IIC_Wait_Ack();
	}
	IIC_Stop();//����һ��ֹͣ����

	return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IICreadByte(u8 dev, u8 reg, u8 *data)
{
	*data = I2C_ReadOneByte(dev, reg);
	return 1;
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
	return IICwriteBytes(dev, reg, 1, &data);
}

u8 IICwriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data)
{

	u8 b;
	if (IICreadByte(dev, reg, &b) != 0)
	{
		u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
		data <<= (8 - length);
		data >>= (7 - bitStart);
		b &= mask;
		b |= data;
		return IICwriteByte(dev, reg, b);
	}
	else
	{
		return 0;
	}
}

/********************************************************************

*******************************************************************************/
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
	u8 b;
	IICreadByte(dev, reg, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------



void IIC2_Start(void)
{
	SDA2_OUT();     //sda�����
	IIC2_SDA = 1;
	IIC2_SCL = 1;
	delay_us(1);
	IIC2_SDA = 0; //START:when CLK is high,DATA change form high to low
	delay_us(1);
	IIC2_SCL = 0; //ǯסI2C���ߣ�׼�����ͻ��������
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void IIC2_Stop(void)
{
	SDA2_OUT();//sda�����
	IIC2_SCL = 0;
	IIC2_SDA = 0; //STOP:when CLK is high DATA change form low to high
	delay_us(1);
	IIC2_SCL = 1;
	delay_us(1);
	IIC2_SDA = 1; //����I2C���߽����ź�
	delay_us(1);
}


u8 IIC2_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	SDA2_IN();      //SDA����Ϊ����
	IIC2_SDA = 1;
	delay_us(1);
	IIC2_SCL = 1;
	delay_us(1);
	while(READ2_SDA)
	{
		ucErrTime++;
		if(ucErrTime > 50)
		{
			IIC2_Stop();
			I2C2_Erorr_Count ++;	 //I2C ����
			return 1;
		}
		delay_us(2);
	}
	IIC2_SCL = 0; //ʱ�����0
	return 0;
}


void IIC2_Ack(void)
{
	IIC2_SCL = 0;
	SDA2_OUT();
	IIC2_SDA = 0;
	delay_us(1);
	IIC2_SCL = 1;
	delay_us(1);
	IIC2_SCL = 0;
}


void IIC2_NAck(void)
{
	IIC2_SCL = 0;
	SDA2_OUT();
	IIC2_SDA = 1;
	delay_us(1);
	IIC2_SCL = 1;
	delay_us(1);
	IIC2_SCL = 0;
}


void IIC2_Send_Byte(u8 txd)
{
	u8 t;
	SDA2_OUT();
	IIC2_SCL = 0; //����ʱ�ӿ�ʼ���ݴ���
	for(t = 0; t < 8; t++)
	{
		IIC2_SDA = (txd & 0x80) >> 7;
		txd <<= 1;
		delay_us(1);
		IIC2_SCL = 1;
		delay_us(1);
		IIC2_SCL = 0;
		delay_us(1);
	}
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IIC2_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA2_IN();//SDA����Ϊ����
	for(i = 0; i < 8; i++ )
	{
		IIC2_SCL = 0;
		delay_us(2);
		IIC2_SCL = 1;
		receive <<= 1;
		if(READ2_SDA)receive++;
		delay_us(1);
	}
	if (ack)
		IIC2_Ack(); //����ACK
	else
		IIC2_NAck();//����nACK
	return receive;
}


/**************************ʵ�ֺ���********************************************

*******************************************************************************/
unsigned char I2C2_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
	unsigned char res = 0;

	IIC2_Start();
	IIC2_Send_Byte(I2C_Addr);	   //����д����
	res++;
	IIC2_Wait_Ack();
	IIC2_Send_Byte(addr);
	res++;  //���͵�ַ
	IIC2_Wait_Ack();
	//IIC_Stop();//����һ��ֹͣ����
	IIC2_Start();
	IIC2_Send_Byte(I2C_Addr + 1);
	res++;          //�������ģʽ
	IIC2_Wait_Ack();
	res = IIC_Read_Byte(0);
	IIC2_Stop();//����һ��ֹͣ����

	return res;
}


/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IIC2readBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
	u8 count = 0;

	IIC2_Start();
	IIC2_Send_Byte(dev);	   // Send address with Transmitter mode
	IIC2_Wait_Ack();
//	delay_us(5);
	IIC2_Send_Byte(reg); 
  IIC2_Wait_Ack();
	IIC2_Stop();
	delay_us(5);  
	IIC2_Start();
	IIC2_Send_Byte(dev + 1); //�Send address with Receiver mode
	IIC2_Wait_Ack();
  delay_us(6);
	for(count = 0; count < length; count++)
	{

		if(count != length - 1) {
			data[count] = IIC2_Read_Byte(1); //�read byte with ACK
			delay_us(3);
		}
		else  data[count] = IIC2_Read_Byte(0);	 //���һ���ֽ�NACK
	}
	IIC2_Stop();//����һ��ֹͣ����
	return count;
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IIC2writeBytes(u8 dev, u8 reg, u8 length, u8 *data)
{

	u8 count = 0;
	IIC2_Start();
	IIC2_Send_Byte(dev);	   //����д����
	IIC2_Wait_Ack();
	//for MS5611
	if (reg!=0)
	{
	IIC2_Send_Byte(reg);   //���͵�ַ
	IIC2_Wait_Ack();
	}
	for(count = 0; count < length; count++)
	{
		IIC2_Send_Byte(data[count]);
		IIC2_Wait_Ack();
	}
	IIC2_Stop();//����һ��ֹͣ����

	return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IIC2readByte(u8 dev, u8 reg, u8 *data)
{
	*data = I2C2_ReadOneByte(dev, reg);
	return 1;
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
unsigned char IIC2writeByte(unsigned char dev, unsigned char reg, unsigned char data)
{
	return IIC2writeBytes(dev, reg, 1, &data);
}

u8 IIC2writeBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data)
{

	u8 b;
	if (IIC2readByte(dev, reg, &b) != 0)
	{
		u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
		data <<= (8 - length);
		data >>= (7 - bitStart);
		b &= mask;
		b |= data;
		return IIC2writeByte(dev, reg, b);
	}
	else
	{
		return 0;
	}
}

/********************************************************************

*******************************************************************************/
u8 IIC2writeBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
	u8 b;
	IIC2readByte(dev, reg, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return IIC2writeByte(dev, reg, b);
}

/*
Third I2C
*/

void IIC3_Start(void)
{
	SDA3_OUT();     //sda�����
	IIC3_SDA = 1;
	IIC3_SCL = 1;
	delay_us(2);
	IIC3_SDA = 0; //START:when CLK is high,DATA change form high to low
	delay_us(2);
	IIC3_SCL = 0; //ǯסI2C���ߣ�׼�����ͻ��������
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
void IIC3_Stop(void)
{
	SDA3_OUT();//sda�����
	IIC3_SCL = 0;
	IIC3_SDA = 0; //STOP:when CLK is high DATA change form low to high
	delay_us(2);
	IIC3_SCL = 1;
//	delay_us(1);
	IIC3_SDA = 1; //����I2C���߽����ź�
	delay_us(2);
}


u8 IIC3_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	SDA3_IN();      //SDA����Ϊ����
	IIC3_SDA = 1;
	delay_us(1);
	IIC3_SCL = 1;
	delay_us(1);
	while(READ3_SDA)
	{
		ucErrTime++;
		if(ucErrTime > 50)
		{
			IIC3_Stop();
			I2C3_Erorr_Count ++;	 //I2C ����
			return 1;
		}
		delay_us(1);
	}
	IIC3_SCL = 0; //ʱ�����0
	return 0;
}


void IIC3_Ack(void)
{
	IIC3_SCL = 0;
	SDA3_OUT();
	IIC3_SDA = 0;
	delay_us(1);
	IIC3_SCL = 1;
	delay_us(1);
	IIC3_SCL = 0;
}


void IIC3_NAck(void)
{
	IIC3_SCL = 0;
	SDA3_OUT();
	IIC3_SDA = 1;
	delay_us(1);
	IIC3_SCL = 1;
	delay_us(1);
	IIC3_SCL = 0;
}


void IIC3_Send_Byte(u8 txd)
{
	u8 t;
	SDA3_OUT();
	IIC3_SCL = 0; //����ʱ�ӿ�ʼ���ݴ���
	for(t = 0; t < 8; t++)
	{
		IIC3_SDA = (txd & 0x80) >> 7;
		txd <<= 1;
		delay_us(1);
		IIC3_SCL = 1;
		delay_us(1);
		IIC3_SCL = 0;
		delay_us(1);
	}
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IIC3_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA3_IN();//SDA����Ϊ����
	for(i = 0; i < 8; i++ )
	{
		IIC3_SCL = 0;
		delay_us(1);
		IIC3_SCL = 1;
		receive <<= 1;
		if(READ3_SDA)receive++;
		delay_us(1);
	}
	if (ack)
		IIC3_Ack(); //����ACK
	else
		IIC3_NAck();//����nACK
	return receive;
}


/**************************ʵ�ֺ���********************************************

*******************************************************************************/
unsigned char I2C3_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
	unsigned char res = 0;

	IIC3_Start();
	IIC3_Send_Byte(I2C_Addr);	   //����д����
	res++;
	IIC3_Wait_Ack();
	IIC3_Send_Byte(addr);
	res++;  //���͵�ַ
	IIC3_Wait_Ack();
	//IIC_Stop();//����һ��ֹͣ����
	IIC3_Start();
	IIC3_Send_Byte(I2C_Addr + 1);
	res++;          //�������ģʽ
	IIC3_Wait_Ack();
	res = IIC_Read_Byte(0);
	IIC3_Stop();//����һ��ֹͣ����

	return res;
}


/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IIC3readBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
	u8 count = 0;

	IIC3_Start();
	IIC3_Send_Byte(dev);	   // Send address with Transmitter mode
	IIC3_Wait_Ack();
//	delay_us(5);
	IIC3_Send_Byte(reg); 
  IIC3_Wait_Ack();
	//IIC3_Stop();
	//delay_us(5);  
	IIC3_Start();
	IIC3_Send_Byte(dev + 1); //�Send address with Receiver mode
	IIC3_Wait_Ack();
 // delay_us(6);
	for(count = 0; count < length; count++)
	{

		if(count != length - 1) {
			data[count] = IIC3_Read_Byte(1); //�read byte with ACK
		//	delay_us(3);
		}
		else  data[count] = IIC3_Read_Byte(0);	 //���һ���ֽ�NACK
	}
	IIC3_Stop();//����һ��ֹͣ����
	return count;
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IIC3writeBytes(u8 dev, u8 reg, u8 length, u8 *data)
{

	u8 count = 0;
	IIC3_Start();
	IIC3_Send_Byte(dev);	   //����д����
	IIC3_Wait_Ack();
	//for MS5611
	if (reg!=0)
	{
	IIC3_Send_Byte(reg);   //���͵�ַ
	IIC3_Wait_Ack();
	}
	for(count = 0; count < length; count++)
	{
		IIC3_Send_Byte(data[count]);
		IIC3_Wait_Ack();
	}
	IIC3_Stop();//����һ��ֹͣ����

	return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
u8 IIC3readByte(u8 dev, u8 reg, u8 *data)
{
	*data = I2C3_ReadOneByte(dev, reg);
	return 1;
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
unsigned char IIC3writeByte(unsigned char dev, unsigned char reg, unsigned char data)
{
	return IIC3writeBytes(dev, reg, 1, &data);
}

u8 IIC3writeBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data)
{

	u8 b;
	if (IIC3readByte(dev, reg, &b) != 0)
	{
		u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
		data <<= (8 - length);
		data >>= (7 - bitStart);
		b &= mask;
		b |= data;
		return IIC3writeByte(dev, reg, b);
	}
	else
	{
		return 0;
	}
}

/********************************************************************

*******************************************************************************/
u8 IIC3writeBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
	u8 b;
	IIC3readByte(dev, reg, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return IIC3writeByte(dev, reg, b);
}
