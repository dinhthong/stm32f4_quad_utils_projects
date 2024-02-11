/*
Read RC using GPIO rising edge Interrupt and tick.
Doesn't work properly, I don't know why.
https://os.mbed.com/forum/mbed/topic/466/?page=1#comment-2558
*/

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#define micros() TIM5->CNT
void EXTILine_all_Config(void);
void Initialise_Timer5(void);
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;
RCC_ClocksTypeDef RCC_ClocksStruct;

NVIC_InitTypeDef  NVIC_InitStructure;
EXTI_InitTypeDef  EXTI_InitStructure;
uint16_t channels[10],channels_buffer[10];
uint8_t channels_cnt;
uint32_t lastRC_us,now;


int main(void)
{
	
	
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */
       	SysTick_Config(168);
 //Initialise_Timer5();
		lastRC_us=tick_count;
		EXTILine_all_Config();
  while (1)
		{
			}
}
/*
1 microsecond resolution
*/
void Initialise_Timer5(void)
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

void EXTILine_all_Config(void)
{
    /* Enable GPIOE clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Configure PB0 pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect EXTI Line0, to PE[6:0]pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);

    /* RC PPM input Interrupt Line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}
uint16_t PPMval;

void EXTI0_IRQHandler(void)
{
/* Make sure that interrupt flag is set */
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
	// microsecond
		now=tick_count;

		PPMval=now-lastRC_us;
		lastRC_us=now;
			if (PPMval>10000)
				{
					if (channels_cnt==8)
					{
						RC_update();
					}
					channels_cnt=0;
				}
			else
				{
					if (PPMval>1050 && PPMval<1950){
		channels_buffer[channels_cnt]=PPMval;}
					
		channels_cnt++;
				}
		}
	 /* Clear interrupt flag */
			EXTI->PR = EXTI_Line0;
}
