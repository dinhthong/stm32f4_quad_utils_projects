#include "copter_config.h"

RCC_ClocksTypeDef RCC_ClocksStruct;
GPIO_InitTypeDef GPIO_InitStruct;
I2C_InitTypeDef   I2C_InitStruct;
USART_InitTypeDef USART_InitStruct;
NVIC_InitTypeDef  NVIC_InitStruct;
EXTI_InitTypeDef  EXTI_InitStruct;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef          TIM_OCInitStruct;

void board_leds_config(void)
{
    // GPIO LED
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Configure PD12, PD13 in output pushpull mode */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15 ;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
//		// Trigger
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//    /* Configure PB1-Trigger in output pushpull mode */
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOC, &GPIO_InitStruct);
	
//			// Lora M0,M1 pin
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//    /* Configure PB1-Trigger in output pushpull mode */
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOC, &GPIO_InitStruct);
		
}
/*
Basic Timer 1ms interrupt
timer 4 is 16 bits.
*/
void tim4_config(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock/2)/1000000)-1;     // frequency = 1000000Hz
	
	// if usart protocol is reliable, like direct hardware connect. it should about 150 ms
  TIM_TimeBaseStructure.TIM_Period = 30000 - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM4, ENABLE);
  
  NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}
/*
timer for main loop
*/
void tim3_config(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock/2)/1000000)-1;     // frequency = 1000000Hz
  TIM_TimeBaseStructure.TIM_Period = 4000 - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  
  NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);   
}

/*
Timer 1 PWM mode.
For quadcopter's motor
*/
void TIM_PWM_Configuration(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 168-1;
    TIM_TimeBaseStructure.TIM_Period = 20000-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStruct.TIM_Pulse = 0;
 
    TIM_OC1Init(TIM1, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM1, &TIM_OCInitStruct);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OC3Init(TIM1, &TIM_OCInitStruct);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OC4Init(TIM1, &TIM_OCInitStruct);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void exti_gpio_config(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_5;
    GPIO_Init(GPIOE, &GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
	
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource5);
    /* RC PPM input Interrupt Line */
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);
	
//	/* HC-SR04 Echo pin */
//	
//    EXTI_InitStruct.EXTI_Line = EXTI_Line5;
//    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//    EXTI_Init(&EXTI_InitStruct);
//		
//    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
//    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStruct);

//    NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
//    NVIC_Init(&NVIC_InitStruct);
}
/*
Used for printf function 
*/
void usart_printf_config(uint32_t BaudRate)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);//UART Tx pin
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);//UART Rx pin

    USART_InitStruct.USART_BaudRate= BaudRate;
    USART_InitStruct.USART_HardwareFlowControl= USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode= USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_Parity= USART_Parity_No;
    USART_InitStruct.USART_StopBits= USART_StopBits_1;
    USART_InitStruct.USART_WordLength= USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

//DMA_InitTypeDef DMA_InitStruct;
//uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
//void gps_init(void) {
//	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
//	
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_Init(GPIOB, &GPIO_InitStruct);
//	
//	  /*PD5 to PL2303 RX*/
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);//UART Tx pins 
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);//UART Rx pins
//	
//    //USART2 configuration
//    USART_InitStruct.USART_BaudRate = 115200;
//    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
//    USART_InitStruct.USART_Parity = USART_Parity_No;
//    USART_InitStruct.USART_StopBits = USART_StopBits_1;
//    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
//    USART_Init(USART3, &USART_InitStruct);
//		
//		NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
//		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStruct);
////		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
//		USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
//		USART_Cmd(USART3, ENABLE);
//	  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
//	
//	  /* Configure DMA for USART RX, DMA1, Stream1, Channel4 */
//    DMA_StructInit(&DMA_InitStruct);
//    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
//    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)DMA_RX_Buffer;
//    DMA_InitStruct.DMA_BufferSize = DMA_RX_BUFFER_SIZE;
//    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
//    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
//    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//		DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
//		DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    DMA_Init(DMA1_Stream1, &DMA_InitStruct);
//		DMA_Cmd(DMA1_Stream1, ENABLE);
////		    /* Enable global interrupts for DMA stream */
////    NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream1_IRQn;
////    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
////    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
////    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
////    NVIC_Init(&NVIC_InitStruct);
//////    
//////		    /* Enable transfer complete interrupt */
////    DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
//}
