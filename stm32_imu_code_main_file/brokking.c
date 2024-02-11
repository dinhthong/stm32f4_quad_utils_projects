#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "MPU6050.h"
#include <Math.h>
#include <stdio.h>
#include <stdlib.h>
#include "ms5611.h"
#define UPDATE_INTERVAL 25000

#ifdef FAST_I2C_MODE
 #define I2C_SPEED 400000
 #define I2C_DUTYCYCLE I2C_DutyCycle_16_9  
#else /* STANDARD_I2C_MODE*/
 #define I2C_SPEED 100000
 #define I2C_DUTYCYCLE I2C_DutyCycle_2
#endif /* FAST_I2C_MODE*/
RCC_ClocksTypeDef RCC_ClocksStruct;
GPIO_InitTypeDef  GPIO_InitStructure;
NVIC_InitTypeDef  NVIC_InitStructure;
EXTI_InitTypeDef  EXTI_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef          TIM_OCInitStructure;

void USART_Configuration(unsigned int BaudRate);
void GPIO_Configuration(void);
void Delay(__IO uint32_t nCount);
void I2C1_mpu6050(void);
void I2C2_ms5611(void);
void I2C3_hmc5883l(void);

void gyro_signalen();
void calculate_angle(void);
void uart_pc_connection(void);
void _delay_us(uint32_t us);
void _delay_ms(uint32_t ms);
void send_USART_PC(void);
void rcv_USART_handler(void);
void hardware_init(void);
float LPF(float x,float pre_value, float CUTOFF);
float Kalman_getAngle_roll(float newAngle, float newRate, float dt);
float Kalman_getAngle_pitch(float newAngle, float newRate, float dt);

int16_t MPU6050data[7], loop=0;


int8_t start=0;
uint32_t mili(void);
uint32_t micros(void);
float Kp_roll=0.54, Ki_roll=0.004, Kd_roll=17.6, Kp_pitch=0.54, Ki_pitch=0.004, Kd_pitch=17.6,
	Kp_yaw=3.3, 				Ki_yaw=0.02, 			Kd_yaw=0;

/*PID values from brokking project*/
//float Kp_roll=1.3, Ki_roll=.04, Kd_roll=18.0, Kp_pitch=1.3, Ki_pitch=.04, Kd_pitch=18.0,
//	Kp_yaw=3.3, 				Ki_yaw=0.02, 			Kd_yaw=0;
float Kp_alt=20.2,         Ki_alt= 0.002 ,       Kd_alt=0.127;//0.11 0.05 20.5  0.058 0.13 20.2
float gyro_pitch_input,gyro_roll_input,gyro_yaw_input;

int auto_control,ha_canh;
int32_t a0,c0,a1,c1,a2,c2,a3,c3,a4,c4,c5,a5,c6,a6;
int8_t cnt=0;
int32_t gyro_X_cal,gyro_Y_cal,gyro_Z_cal;
float angle_roll, angle_yaw,angle_pitch;
float angle_roll_acc,angle_pitch_acc;
float tong;
int16_t calib_cnt;
float gyro_pitch,gyro_roll,gyro_yaw,acc_x,acc_y,acc_z;
float pid_i_mem_alt, pid_alt_setpoint, gyro_alt_input, pid_output_alt, pid_last_alt_d_error;
float cut_off=10;
int dem_timer,dem_time;
uint32_t loop_4ms;
double dt;
int offset_Z;
uint8_t usart_cnt;
#define BUFFERSIZE 18
char RxBuffer[BUFFERSIZE];
__IO uint8_t RxIndex = 0x00;
uint16_t imu_loop,elapsed_imu_loop,pre_imu_loop;
int main(void)
{
		hardware_init();
 
    _delay_us(500000);
	
    for (calib_cnt = 0; calib_cnt < 1000; calib_cnt++)
    {
        gyro_signalen();
        gyro_X_cal += MPU6050data[4];
        gyro_Y_cal += MPU6050data[5];
        gyro_Z_cal += MPU6050data[6];
        offset_Z += MPU6050data[2];
        _delay_us(3000);
    }
    gyro_X_cal /= 1000;
    gyro_Y_cal /= 1000;
    gyro_Z_cal /= 1000;
    offset_Z /=1000;


    dem_timer = micros();
    dem_time = micros();

    while (1)
    {
        gyro_signalen();
        calculate_angle();

//          if (usart_cnt>30){
//            send_USART_PC();
//						usart_cnt=0;
//					}
//					else 
//					{
//						usart_cnt++;
//					}
        while((tick_count - loop_4ms )< 4000) {};
        dt=(tick_count-loop_4ms)*0.000001;
        loop_4ms= tick_count;
    }
}

void gyro_signalen() {
    MPU6050_GetRawAccelTempGyro(MPU6050data);
		
    acc_x = MPU6050data[0];
    acc_y =  MPU6050data[1];
    acc_z =  MPU6050data[2];

    if( calib_cnt  == 1000) {
        MPU6050data[4] -= gyro_X_cal;
        MPU6050data[5] -= gyro_Y_cal;
        MPU6050data[6] -= gyro_Z_cal;
    }

    gyro_roll =LPF(MPU6050data[4],gyro_roll,cut_off);
    gyro_pitch =LPF(MPU6050data[5],gyro_pitch,cut_off);
    gyro_yaw =LPF(MPU6050data[6],gyro_yaw,cut_off);

}
float tmp;
void calculate_angle(void)
{
    gyro_roll_input  = (gyro_roll_input * 0.7)	+ ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
    gyro_yaw_input	 = (gyro_yaw_input * 0.7) 	+ ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

    //  1/250/65.5 =0.000061068
    angle_roll+= gyro_roll * 0.000061068;// 0.004*500/32767 = 0.000061037
    angle_pitch+= gyro_pitch * 0.000061068;
    angle_yaw+= gyro_yaw * 0.000061068;
	
    //(1/250/65.5)*pi/180=0.000001066
    angle_pitch-= angle_roll*sin(gyro_yaw*0.000001066);
    angle_roll+= angle_pitch*sin(gyro_yaw*0.000001066);

    tong = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
	  if(abs(acc_y) < tong) {
        angle_roll_acc=asin((float)acc_y/tong)*57.296;
    }
		
    if(abs(acc_x) < tong) {
        angle_pitch_acc=-asin((float)acc_x/tong)*57.296;
    }


//			acc_pitch += 0;                                                   //Accelerometer calibration value for pitch.
//		  acc_roll -= 0;
    angle_pitch = angle_pitch * 0.9996 +  angle_pitch_acc * 0.0004;
    angle_roll =  angle_roll * 0.9996  +   angle_roll_acc * 0.0004 ;
	}

void hardware_init(void)
{
	  SysTick_Config(168);
//    GPIO_Configuration();
//    USART_Configuration(115200);

//    I2C2_ms5611();
//    begin();
//    referencePressure = readPressure(0);
    I2C1_mpu6050();
    MPU6050_Initialize();
//		I2C3_hmc5883l();
//		HMC5883L_Initialize();

  
}

void send_USART_PC(void)
{
//printf("*%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,\r\n",
//	(float)tick_count*0.000001,(float)esc_1,(float)esc_2,(float)esc_3,(float)esc_4,angle_roll,
//		angle_pitch,angle_yaw,gyro_roll_input,gyro_pitch_input,gyro_yaw_input);
	
//	printf("*%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,\r\n",
//	(float)tick_count*0.000001,(float)CH_AILE,(float)CH_ELEV,(float)CH_GEAR,(float)CH_DR,
//		(float)CH_THRO,(float)CH_RUDD,angle_yaw,gyro_roll_input,gyro_pitch_input,gyro_yaw_input);
}


void USART_Configuration(unsigned int BaudRate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);//UART Tx pin
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);//UART Rx pin

    USART_InitStruct.USART_BaudRate=BaudRate;
    USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
    USART_InitStruct.USART_Parity=USART_Parity_No;
    USART_InitStruct.USART_StopBits=USART_StopBits_1;
    USART_InitStruct.USART_WordLength=USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStruct);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

void rcv_USART_handler(void)
{
    Kp_roll=(RxBuffer[0]-48)*100+(RxBuffer[1]-48)*10+(RxBuffer[2]-48)+(RxBuffer[4]-48)/10.0+(RxBuffer[5]-48)/100.0;
    Ki_roll=(RxBuffer[6]-48)*100+(RxBuffer[7]-48)*10+(RxBuffer[8]-48)+(RxBuffer[10]-48)/10.0+(RxBuffer[11]-48)/100.0;
    Kd_roll=(RxBuffer[12]-48)*100+(RxBuffer[13]-48)*10+(RxBuffer[14]-48)+(RxBuffer[16]-48)/10.0+(RxBuffer[17]-48)/100.0;
	  Kp_pitch=Kp_roll;
   	Ki_pitch=Ki_roll;
   	Kd_pitch=Kd_roll;
}
void I2C2_ms5611(void)
{
    I2C_InitTypeDef   I2C_InitStructure;

		GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_I2C2);   
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_I2C2);  

    I2C_DeInit(I2C2);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
    I2C_InitStructure.I2C_OwnAddress1 = 0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_ClockSpeed = 100000;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2, &I2C_InitStructure);
    /* I2C ENABLE */
    I2C_Cmd(I2C2, ENABLE);
}

void I2C3_hmc5883l(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  I2C_InitTypeDef   I2C_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_I2C3);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_I2C3);	

  /* I2C De-initialize */
  I2C_DeInit(I2C3);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_ClockSpeed = 100000;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C3, &I2C_InitStructure);
 /* I2C ENABLE */
  I2C_Cmd(I2C3, ENABLE); 
  /* Enable Interrupt */
}

void GPIO_Configuration(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Configure PB1-Trigger in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}



void I2C1_mpu6050(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef   I2C_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);

    /* I2C De-initialize */
    I2C_DeInit(I2C1);//reset thanh ghi i2c dua ve mac dinh
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
    I2C_InitStructure.I2C_OwnAddress1 = 0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_ClockSpeed = 100000;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);

}

void _delay_us(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}

uint32_t mili(void)
{
    float ms;
    ms = tick_count*0.001;
    return ms;
}

void _delay_ms(uint32_t ms)
{
    uint32_t now = mili();
    while (mili() - now < ms);
}
uint32_t micros(void)
{
    float us;
    us = tick_count;
    return us;
}
void Delay(__IO uint32_t nCount)
{
    while(nCount--)
    {
    }
}

float LPF(float x,float pre_value, float CUTOFF)
{
    float RC, alpha, y;
    RC = 1.0/(CUTOFF*2*3.1416);
    //dt =SAMPLE_RATE;
    alpha = dt/(RC+dt);
    y = pre_value + alpha * ( x - pre_value );
    return y;
}
void USART2_IRQHandler(void)
{

    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {

    }

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        //read from USART_DR will automatically clear the RNXE bit.
        RxBuffer[RxIndex++] = USART_ReceiveData(USART2);
        if (RxIndex >= BUFFERSIZE)
        {
            RxIndex=0;
            /* Receive Transaction data */
            rcv_USART_handler();
        }
    }
}
