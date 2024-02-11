/******************** (C) COPYRIGHT 2015 DUT ********************************
 * ����    �����Ĳ�
 * �ļ���  ��report.c
 * ����    ����λ���ϴ�����
 * ����    ��2015/11/30 12:43:38
 * ��ϵ��ʽ��1461318172��qq��
**********************************************************************************/


#include "common.h"
#include "usart.h"


//����1����1���ַ�
//c:Ҫ���͵��ַ�
void usart1_send_char(u8 c)
{

	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	USART_SendData(USART1, c);

}
//���ܳ���λ�������ϴ�����
void Report_imu( unsigned short int roll,  unsigned short int pitch,  unsigned short int yaw)
{
	int i, j;
	static unsigned short int send_data[3][8] = { { 0 }, { 0 }, { 0 } };

	send_data[0][0] = ( unsigned short int)(roll);
	send_data[0][1] = ( unsigned short int)(pitch);
	send_data[0][2] = ( unsigned short int)(yaw);
	send_data[0][3] = (unsigned short int)(0);
	send_data[0][4] = (unsigned short int)(0);
	send_data[0][5] = (unsigned short int)(0);
	send_data[0][6] = (unsigned short int)(0);
	send_data[0][7] = (unsigned short int)(0);

	send_data[1][0] = (unsigned short int)(0);
	send_data[1][1] = (unsigned short int)(0);
	send_data[1][2] = (unsigned short int)(0);
	send_data[1][3] = (unsigned short int)(0);
	send_data[1][4] = (unsigned short int)(0);
	send_data[1][5] = (unsigned short int)(0);
	send_data[1][6] = (unsigned short int)(0);
	send_data[1][7] = (unsigned short int)(0);

	send_data[2][0] = (unsigned short int)(0);
	send_data[2][1] = (unsigned short int)(0);
	send_data[2][2] = (unsigned short int)(0);
	send_data[2][3] = (unsigned short int)(0);
	send_data[2][4] = (unsigned short int)(0);
	send_data[2][5] = (unsigned short int)(0);
	send_data[2][6] = (unsigned short int)(0);
	send_data[2][7] = (unsigned short int)(0);

	printf("ST");
	for ( i = 0; i < 3; i++)
		for ( j = 0; j < 8; j++)
		{
			usart1_send_char((unsigned char)(send_data[i][j] & 0x00ff));
			usart1_send_char((unsigned char)(send_data[i][j] >> 8u));
		}
}

//void usart1_send_char(u8 c)
//{

//	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//	USART_SendData(USART1, c);

//}
//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
//void usart1_niming_report(u8 fun, u8 *data, u8 len)
//{
//	u8 send_buf[32];
//	u8 i;
//	if(len > 28)return;	//���28�ֽ�����
//	send_buf[len + 3] = 0;	//У��������
//	send_buf[0] = 0X88;	//֡ͷ
//	send_buf[1] = fun;	//������
//	send_buf[2] = len;	//���ݳ���
//	for(i = 0; i < len; i++)send_buf[3 + i] = data[i];			//��������
//	for(i = 0; i < len + 3; i++)send_buf[len + 3] += send_buf[i];	//����У���
//	for(i = 0; i < len + 4; i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1
//}
//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//void mpu6050_send_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
//{
//	u8 tbuf[12];
//	tbuf[0] = (aacx >> 8) & 0XFF;
//	tbuf[1] = aacx & 0XFF;
//	tbuf[2] = (aacy >> 8) & 0XFF;
//	tbuf[3] = aacy & 0XFF;
//	tbuf[4] = (aacz >> 8) & 0XFF;
//	tbuf[5] = aacz & 0XFF;
//	tbuf[6] = (gyrox >> 8) & 0XFF;
//	tbuf[7] = gyrox & 0XFF;
//	tbuf[8] = (gyroy >> 8) & 0XFF;
//	tbuf[9] = gyroy & 0XFF;
//	tbuf[10] = (gyroz >> 8) & 0XFF;
//	tbuf[11] = gyroz & 0XFF;
//	usart1_niming_report(0XA1, tbuf, 12); //�Զ���֡,0XA1
//}
//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
//void usart1_report_imu(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
//{
//	u8 tbuf[28];
//	u8 i;
//	for(i = 0; i < 28; i++)tbuf[i] = 0; //��0
//	tbuf[0] = (aacx >> 8) & 0XFF;
//	tbuf[1] = aacx & 0XFF;
//	tbuf[2] = (aacy >> 8) & 0XFF;
//	tbuf[3] = aacy & 0XFF;
//	tbuf[4] = (aacz >> 8) & 0XFF;
//	tbuf[5] = aacz & 0XFF;
//	tbuf[6] = (gyrox >> 8) & 0XFF;
//	tbuf[7] = gyrox & 0XFF;
//	tbuf[8] = (gyroy >> 8) & 0XFF;
//	tbuf[9] = gyroy & 0XFF;
//	tbuf[10] = (gyroz >> 8) & 0XFF;
//	tbuf[11] = gyroz & 0XFF;
//	tbuf[18] = (roll >> 8) & 0XFF;
//	tbuf[19] = roll & 0XFF;
//	tbuf[20] = (pitch >> 8) & 0XFF;
//	tbuf[21] = pitch & 0XFF;
//	tbuf[22] = (yaw >> 8) & 0XFF;
//	tbuf[23] = yaw & 0XFF;
//	usart1_niming_report(0XAF, tbuf, 28); //�ɿ���ʾ֡,0XAF
//}
