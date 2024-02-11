/*
PWM with comments.
Most of ESCs have nominal refesh rate in range of: 50Hz to 400Hz. It means that the PWM frequency should be
about 200Hz
*/

#include "main.h"
#define rc_elev ch[2]
#define rc_aile ch[1]
#define rc_thro ch[0]
#define rc_rudd ch[3] // CH4
#define PWM_period 20000 //micro seconds
void PWM_Config(void);
void EXTILine_Config(void);

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

void delay_us(uint32_t period){

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  	TIM2->PSC = 83;		// clk = SystemCoreClock /2/(PSC+1) = 1MHz
  	TIM2->ARR = period-1;
  	TIM2->CNT = 0;
  	TIM2->EGR = 1;		// update registers;
  	TIM2->SR  = 0;		// clear overflow flag
  	TIM2->CR1 = 1;		// enable Timer6
  	while (!TIM2->SR);
  	TIM2->CR2 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
}
//delay 0.1ms
void delay_01ms(uint16_t period){
	
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 8399;		// clk = SystemCoreClock /2 /(PSC+1) = 10KHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;
  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6
  	while (!TIM6->SR);
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
uint16_t S = 1000;
uint16_t ch[5];
int main(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC->AHB1ENR |= RCC_AHB1Periph_GPIOD;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	Initial_System_Timer();
	PWM_Config();
	
	EXTILine_Config();
	
	//"on the fly" update CCR registers. -> use for quadcopter
	//This is sent to initialize the motors
//	printf("Hello this is the start of trace\r\n");
//	TIM1->CCR1 = S;
//	TIM1->CCR2 = S;
//	TIM1->CCR3 = S;
//  TIM1->CCR4 = S;
//	delay_01ms(50000);
	while(1)  
  {
//		if (S<1400)
//		{
//		S+=50;
//		delay_01ms(50000);
//		//	printf("Hello this is tracing %d\r\n",S);
//		GPIO_ToggleBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
//		}
	if (rc_thro>=1900) {
		rc_thro = 1900;
	}
	TIM1->CCR1 = rc_thro;
	TIM1->CCR2 = rc_thro;
	TIM1->CCR3 = rc_thro;
  TIM1->CCR4 = rc_thro;
		
	}
}

 void PWM_Config(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_9 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
 
    TIM_TimeBaseStructure.TIM_Prescaler = 168-1; // 1 tick = 1us
    TIM_TimeBaseStructure.TIM_Period = PWM_period-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    //TIM_OCStructInit(&TIM_OCInitStructure);

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
GPIO_InitTypeDef GPIO_InitStructure;
EXTI_InitTypeDef  EXTI_InitStructure;
NVIC_InitTypeDef  NVIC_InitStructure;
//void EXTILine_Config(void)
//{
//	// PE5, PC9,10,13
//    /* Enable GPIOE clock */
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//    /* Enable SYSCFG clock */

//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 |GPIO_Pin_5 |GPIO_Pin_7 ;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
////	
////	    /* Configure PB0 pin as input floating */
////    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
////    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
////    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_13 ;
////    GPIO_Init(GPIOC, &GPIO_InitStructure);
////	
////		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	
////	    /* Configure PB0 pin as input floating */
////    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
////    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
////    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
////    GPIO_Init(GPIOB, &GPIO_InitStructure);
//		
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//	//	RCC_APB2PeriphClockCmd(RCC_APB1Periph_SYSCFG, ENABLE);
//	/* Connect EXTI Line0, to PE[6:0]pin */
//    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);
//	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);
//	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);
//	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource7);
//		
//    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//		EXTI_Init(&EXTI_InitStructure);
//		
//		EXTI_InitStructure.EXTI_Line = EXTI_Line3;
//    EXTI_Init(&EXTI_InitStructure);
//		
//		EXTI_InitStructure.EXTI_Line =EXTI_Line5;
//    EXTI_Init(&EXTI_InitStructure);
//		
//		EXTI_InitStructure.EXTI_Line =EXTI_Line7;
//    EXTI_Init(&EXTI_InitStructure);
//   

//		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//		
//				
//		 NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//		
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//		
//}



//void EXTI9_5_IRQHandler(void)
//{
//  static uint32_t a1;
//	static uint32_t a2;
//    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
//    {
//        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5))
//        {   
//             // c1 = TIM5->CNT - a1;
//					    a1 = TIM5->CNT;
//        }
//        else {
//					 // c1=tick_count-a1;
//            rc_thro = TIM5->CNT - a1;
//            if(rc_thro <= 1000) {
//                rc_thro = 1000;
//            }
//            if(rc_thro >= 2000) {
//                rc_thro = 2000;
//            }
//            if(rc_thro>1497 & rc_thro<1503) {
//                rc_thro =1500;
//            }
//				}
//        EXTI->PR = EXTI_Line5;
//    }

//    if(EXTI_GetITStatus(EXTI_Line7) != RESET)
//    {
//              if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7))
//        {   
//         //     c2 = TIM5->CNT - a2;
//					    a2 = TIM5->CNT;
//        }
//        else {
//					 // c1=tick_count-a1;
//            rc_elev = TIM5->CNT - a2;
//            if(rc_elev <= 1000) {
//                rc_elev = 1000;
//            }
//            if(rc_elev >= 2000) {
//                rc_elev = 2000;
//            }
//            if(rc_elev>1497 & rc_elev< 1503) {
//                rc_elev =1500;
//            }
//				}
//        EXTI->PR = EXTI_Line7;
//    }
//}

//void EXTI1_IRQHandler(void)
//{
//static uint32_t a3;
////static uint32_t a4,c4;
//    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
//    {
//        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
//        {   
//           //   c3 = TIM5->CNT - a3;
//					    a3 = TIM5->CNT;
//        }
//        else {
//					 // c1=tick_count-a1;
//            rc_aile = TIM5->CNT - a3;
//            if(rc_aile <= 1000) {
//                rc_aile = 1000;
//            }
//            if(rc_aile >= 2000) {
//                rc_aile = 2000;
//            }
//            if(rc_aile>1497 & rc_aile<1503) {
//                rc_aile =1500;
//            }
//				}
//        EXTI->PR = EXTI_Line1;
//    }
//}

//void EXTI3_IRQHandler(void)
//{
//static uint32_t a4;
////static uint32_t a4,c4;
//    if(EXTI_GetITStatus(EXTI_Line3) != RESET)
//    {
//        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3))
//        {   
//           //   c3 = TIM5->CNT - a3;
//					    a4 = TIM5->CNT;
//        }
//        else {
//					 // c1=tick_count-a1;
//            rc_rudd = TIM5->CNT - a4;
//            if(rc_rudd <= 1000) {
//                rc_rudd = 1000;
//            }
//            if(rc_rudd >= 2000) {
//                rc_rudd = 2000;
//            }
//            if(rc_rudd>1497 & rc_rudd<1503) {
//                rc_rudd = 1500;
//            }
//				}
//        EXTI->PR = EXTI_Line3;
//    }

//}


GPIO_InitTypeDef GPIO_InitStructure;
EXTI_InitTypeDef  EXTI_InitStructure;
NVIC_InitTypeDef  NVIC_InitStructure;
void EXTILine_Config(void)
{
	// PE5, PC9,10,13
    /* Enable GPIOE clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    /* Enable SYSCFG clock */

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	    /* Configure PB0 pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_13 ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	    /* Configure PB0 pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	/* Connect EXTI Line0, to PE[6:0]pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource10);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource5);
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line = EXTI_Line9;
    EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line =EXTI_Line10;
    EXTI_Init(&EXTI_InitStructure);
		
		EXTI_InitStructure.EXTI_Line =EXTI_Line13;
    EXTI_Init(&EXTI_InitStructure);
   
    /* Enable and set EXTI Line5, Line6 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_Init(&NVIC_InitStructure);
}


void EXTI9_5_IRQHandler(void)
{
static uint32_t a1,c1;
	static uint32_t a2,c2;
    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5))
        {   
              c1 = TIM5->CNT - a1;
					    a1 = TIM5->CNT;
        }
        else {
					 // c1=tick_count-a1;
            ch[0] = TIM5->CNT - a1;
            if(ch[0] <= 1000) {
                ch[0] = 1000;
            }
            if(ch[0] >= 2000) {
                ch[0] = 2000;
            }
            if(ch[0]>1495 & ch[0]<1505) {
                ch[0] =1500;
            }
				}
        EXTI->PR = EXTI_Line5;
    }

    if(EXTI_GetITStatus(EXTI_Line9) != RESET)
    {
              if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9))
        {   
              c2 = TIM5->CNT - a2;
					    a2 = TIM5->CNT;
        }
        else {
					 // c1=tick_count-a1;
            ch[1] = TIM5->CNT - a2;
            if(ch[1] <= 1000) {
                ch[1] = 1000;
            }
            if(ch[1] >= 2000) {
                ch[1] = 2000;
            }
            if(ch[1]>1495 & ch[1]<1505) {
                ch[1] =1500;
            }
				}
        EXTI->PR = EXTI_Line9;
    }
}

void EXTI15_10_IRQHandler(void)
{
static uint32_t a3,c3;
static uint32_t a4,c4;
    if(EXTI_GetITStatus(EXTI_Line10) != RESET)
    {
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10))
        {   
              c3 = TIM5->CNT - a3;
					    a3 = TIM5->CNT;
        }
        else {
					 // c1=tick_count-a1;
            ch[2] = TIM5->CNT - a3;
            if(ch[2] <= 1000) {
                ch[2] = 1000;
            }
            if(ch[2] >= 2000) {
                ch[2] = 2000;
            }
            if(ch[2]>1495 & ch[2]<1505) {
                ch[2] =1500;
            }
				}
        EXTI->PR = EXTI_Line10;
    }

    if(EXTI_GetITStatus(EXTI_Line13) != RESET)
    {
              if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
        {   
              c4 = TIM5->CNT - a4;
					    a4 = TIM5->CNT;
        }
        else {
					 // c1=tick_count-a1;
            ch[3] = TIM5->CNT - a4;
            if(ch[3] <= 1000) {
                ch[3] = 1000;
            }
            if(ch[3] >= 2000) {
                ch[3] = 2000;
            }
            if(ch[3]>1495 & ch[3]<1505) {
                ch[3] =1500;
            }
				}
        EXTI->PR = EXTI_Line13;
    }
}


