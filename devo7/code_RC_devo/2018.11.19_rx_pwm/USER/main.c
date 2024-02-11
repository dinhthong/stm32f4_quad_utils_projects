#include "main.h"

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

// LPF
//float cut_off=10;
//float baro_altitude;
//double dt, dt_alt;
//float rpy[3], rate_rpy[3]; //roll pitch yaw 
//float rate_rpy_setpoint[3];
//float rate_alt;
//uint16_t Motors[4];
uint16_t temp_throttle;
int16_t Motor_throttle, Motor_roll, Motor_pitch, Motor_yaw;
uint8_t flight_mode;
uint8_t arm, alt_hold_status;
//int8_t calib_count = 0;
uint32_t loop_cnt;
//float althold_pid;
//int16_t add_throttle;
//uint32_t sonar_trigger_dt;
//float alt_sonar;

uint16_t channels[10];
int main(void) {
//	  uint16_t j;
	  EXTILine_Config();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
    delay_init(168);
    delay_ms(200);
    board_leds_config();
    Initial_System_Timer();
    delay_ms(100);
    while(1) {
    }
}




GPIO_InitTypeDef GPIO_InitStructure;
EXTI_InitTypeDef  EXTI_InitStructure;
NVIC_InitTypeDef  NVIC_InitStructure;
void EXTILine_Config(void)
{
	// PE5, PC9,10,13
    /* Enable GPIOE clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* Enable SYSCFG clock */

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 |GPIO_Pin_5 |GPIO_Pin_7 ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//	
//	    /* Configure PB0 pin as input floating */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_13 ;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//	
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
//	    /* Configure PB0 pin as input floating */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
		
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//	RCC_APB2PeriphClockCmd(RCC_APB1Periph_SYSCFG, ENABLE);
	/* Connect EXTI Line0, to PE[6:0]pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource7);
		
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line =EXTI_Line5;
    EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line =EXTI_Line7;
    EXTI_Init(&EXTI_InitStructure);
   

		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
				
		 NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
}

void EXTI9_5_IRQHandler(void)
{
  static uint32_t a1;
	static uint32_t a2;
    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5))
        {   
             // c1 = TIM5->CNT - a1;
					    a1 = TIM5->CNT;
        }
        else {
					 // c1=tick_count-a1;
            channels[2] = TIM5->CNT - a1;
            if(channels[2] <= 1000) {
                channels[2] = 1000;
            }
            if(channels[2] >= 2000) {
                channels[2] = 2000;
            }
            if(channels[2]>1497 & channels[2]<1503) {
                channels[2] =1500;
            }
				}
        EXTI->PR = EXTI_Line5;
    }

    if(EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
              if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7))
        {   
         //     c2 = TIM5->CNT - a2;
					    a2 = TIM5->CNT;
        }
        else {
					 // c1=tick_count-a1;
            channels[0] = TIM5->CNT - a2;
            if(channels[0] <= 1000) {
                channels[0] = 1000;
            }
            if(channels[0] >= 2000) {
                channels[0] = 2000;
            }
            if(channels[0]>1497 & channels[0]< 1503) {
                channels[0] =1500;
            }
				}
        EXTI->PR = EXTI_Line7;
    }
}

void EXTI1_IRQHandler(void)
{
static uint32_t a3;
//static uint32_t a4,c4;
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
        {   
           //   c3 = TIM5->CNT - a3;
					    a3 = TIM5->CNT;
        }
        else {
					 // c1=tick_count-a1;
            channels[1] = TIM5->CNT - a3;
            if(channels[1] <= 1000) {
                channels[1] = 1000;
            }
            if(channels[1] >= 2000) {
                channels[1] = 2000;
            }
            if(channels[1]>1497 & channels[1]<1503) {
                channels[1] =1500;
            }
				}
        EXTI->PR = EXTI_Line1;
    }
}

void EXTI3_IRQHandler(void)
{
static uint32_t a4;
//static uint32_t a4,c4;
    if(EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3))
        {   
           //   c3 = TIM5->CNT - a3;
					    a4 = TIM5->CNT;
        }
        else {
					 // c1=tick_count-a1;
            channels[3] = TIM5->CNT - a4;
            if(channels[3] <= 1000) {
                channels[3] = 1000;
            }
            if(channels[3] >= 2000) {
                channels[3] = 2000;
            }
            if(channels[3]>1497 & channels[3]<1503) {
                channels[3] = 1500;
            }
				}
        EXTI->PR = EXTI_Line3;
    }

}
