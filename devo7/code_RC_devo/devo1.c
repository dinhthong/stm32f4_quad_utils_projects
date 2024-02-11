#include "stm32f4xx.h"
#include "system_timetick.h"
#include <Math.h>
#include <stdio.h>
#include <stdlib.h>

RCC_ClocksTypeDef RCC_ClocksStruct;
GPIO_InitTypeDef  GPIO_InitStructure;
NVIC_InitTypeDef  NVIC_InitStructure;
EXTI_InitTypeDef  EXTI_InitStructure;
#define GPIO_TEMP (uint32_t *)0x40021000

void EXTILine_all_Config(void);
void Delay(__IO uint32_t nCount);

void _delay_us(uint32_t us);
void _delay_ms(uint32_t ms);
uint32_t mili(void);
uint32_t micros(void);
uint16_t CH_ELEV, CH_AILE, CH_RUDD,CH_GEAR,CH_DR,CH_THRO,CH_AUX2;// ELEV=pitch
int32_t a0,a1,a2,a3,a4,a5,a6,c10,a10;
uint16_t cnt_1900_thro,cnt_1900_rud,cnt_1900_a,cnt_1900_e,cnt_1900_g,cnt_1900_d;
uint32_t CH_temp;
int main(void)
{
	//uint32_t *p;

  SysTick_Config(168);
	EXTILine_all_Config();
  while (1)
  {
	}	
}


void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}
void EXTILine_all_Config(void)
{
    /* Enable GPIOE clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Configure PB0 pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_10 ;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect EXTI Line0, to PE[6:0]pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource4);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource5);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource6);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource10);
    /* Configure EXTI Line5, Line6 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line =EXTI_Line1;
    EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line =EXTI_Line2;
    EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line =EXTI_Line3;
    EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line =EXTI_Line4;
    EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line =EXTI_Line5;
    EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line =EXTI_Line6;
    EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line =EXTI_Line10;
    EXTI_Init(&EXTI_InitStructure);
    
    /* Enable and set EXTI Line5, Line6 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
		NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
		NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
		NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
		NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
		NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_Init(&NVIC_InitStructure);
}

void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0))
{
		a0=tick_count;
}
else
{
	CH_temp=tick_count-a0;
	if(CH_temp <= 1105) {CH_temp = 1100;}
		if(CH_temp >= 1895) {
		CH_temp = 1900;
		cnt_1900_e++;
	}
	if(CH_temp>=1495 & CH_temp<=1505) {CH_temp =1500;}
	CH_ELEV=CH_temp;
}
    
    EXTI->PR = EXTI_Line0;
  }
}

void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1))
{
			a1=tick_count;
}
else
{
	CH_temp=tick_count-a1;
	if(CH_temp <= 1105) {CH_temp = 1100;}
		if(CH_temp >= 1895) {
	CH_temp = 1900;
		cnt_1900_a++;
	}
	if(CH_temp>=1495 & CH_temp<=1505) {CH_temp =1500;}
	CH_AILE=CH_temp;
}
    
    EXTI->PR = EXTI_Line1;
  }
}
void EXTI2_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2))
{
		a2=tick_count;
}
else
{
	CH_temp=tick_count-a2;
	if(CH_temp <= 1105) {CH_temp = 1100;}
		if(CH_temp >= 1895) {
		CH_temp = 1900;
		cnt_1900_thro++;
	}
	CH_THRO=CH_temp;
}
    EXTI->PR = EXTI_Line2;
  }
}
void EXTI3_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3))
{
		a3=tick_count;
}
else
{
CH_temp=tick_count-a3;
	if(CH_temp <= 1105) {CH_temp = 1100;}
		if(CH_temp >= 1895) {
		CH_temp = 1900;
		cnt_1900_rud++;
	}
	if(CH_temp>=1495 & CH_temp<=1505) {CH_temp =1500;}
	CH_RUDD=CH_temp;
}
    EXTI->PR = EXTI_Line3;
  }
}
void EXTI4_IRQHandler(void)
{

  if(EXTI_GetITStatus(EXTI_Line4) != RESET)
  {
if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
{
		
		a4=tick_count;
}
else
{
	CH_temp=tick_count-a4;
	if(CH_temp <= 1105) {CH_temp = 1100;}
		if(CH_temp >= 1895) {CH_temp = 1900;
	cnt_1900_g++;}
	if(CH_temp>=1495 & CH_temp<=1505) {CH_temp =1500;}
	CH_GEAR=CH_temp;
	
}
    EXTI->PR = EXTI_Line4;
  }
}
void EXTI9_5_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6))
{
		a6=tick_count;
}
else
{
	CH_temp=tick_count-a6;
	if(CH_temp <= 1105) {CH_temp = 1100;}
		if(CH_temp >= 1895) {CH_temp = 1900;
	cnt_1900_d++;}
	CH_DR=CH_temp;
}
    EXTI->PR = EXTI_Line6;
  }
}
void EXTI15_10_IRQHandler(void)
{

  if(EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10))
{
		a10=tick_count;
}
else
{
	CH_AUX2=tick_count-a10;
	if(CH_AUX2 <= 1105) {CH_AUX2 = 1100;}
	if(CH_AUX2 >= 1895) {CH_AUX2 = 1900;}
	
}
    EXTI->PR = EXTI_Line10;
  }
}
uint32_t micros(void)
{
		float us;
		us = tick_count;
		return us;
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