/*
PC9 <-> GY-86 DRY
PB6 <-> SCL
PB7 <-> SDA
This program read the RAW values from GY-86.
Using IOI2C driver ( I2C protocol running using GPIO ).
*/
#include "common.h"

#define reportHz (500)
#define uploadTime 1000000/reportHz
float LPF(float x,float pre_value, float CUTOFF);

void Initial_System_Timer(void)
{
	RCC->APB1ENR |= 0x0008;	
	TIM5->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
	TIM5->CR2 = 0x0000;
	TIM5->CNT = 0x0000;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->PSC = 84 - 1;	//1us
	TIM5->EGR = 0x0001;
	TIM5->CR1 |= 0x0001; // Enable
}

float IMU_values[9];
//float cut_off=10;
//double referencePressure;
//float ms5611_temperature;
//long realPressure;
//float ms5611_alt_temp,fil_comple_alt,fil_lpf_alt ,EstAlt;
//float AltHold,ms5611_alt_filter,abs_Alt_baro,Alt_baro_cal,ms5611_absolute_alt;
double dt;
float HMC5883_data_2[3];
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	delay_init(168);		
	delay_ms(500);
	GPIO_InitTypeDef GPIO_InitStructure;
	//I2C1 init
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	delay_ms(50);
	// GPIO IIC3 init
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// GPIO IIC4 init
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	Initial_System_Timer();  
	
	MPU6050_initialize();
	HMC5883L_SetUp();
	HMC5883L_SetUp_2();
//	delay_ms(50);

//	begin();
//	referencePressure = readPressure(0);
	uint32_t loop_4ms;

	while(1) {
		HMC58X3_mgetValues_2(HMC5883_data_2);	//
		IMU_getValues(&IMU_values[0]);
	//	HMC58X3_mgetValues_2(HMC5883_data_2);	//
		delay_us(500);
//		update_baro(&ms5611_temperature,&realPressure,&ms5611_alt_temp);
//    fil_lpf_alt =LPF(ms5611_alt_temp,fil_lpf_alt,cut_off);
//    fil_comple_alt =fil_comple_alt*(0.977)+fil_lpf_alt*(0.023);
//		EstAlt =fil_comple_alt*10;
		while((micros() - loop_4ms )< 4000) {};
        dt=(micros()-loop_4ms)*0.000001;
        loop_4ms= micros();
	}
}
float LPF(float x,float pre_value, float CUTOFF)
{
    float RC, alpha, y;
    RC = 1.0f/(CUTOFF*2*3.1416f);
    alpha = dt/(RC+dt);
    y = pre_value + alpha * ( x - pre_value );
    return y;
}
