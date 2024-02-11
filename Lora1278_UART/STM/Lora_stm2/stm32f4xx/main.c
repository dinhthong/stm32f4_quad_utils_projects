#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
void rcv_USART_handler(void);
/*
Sample delay functions.
*/
void Pin_Init(uint32_t BaudRate);
//void Pin_Init(void);
int i;
void Delay(int t){
	int i;
	for (i=0; i<t; i++);
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
  	TIM6->CR2 = 0;		// stop Timer6
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
NVIC_InitTypeDef  NVIC_InitStruct;
GPIO_InitTypeDef GPIO_InitStruct;
USART_InitTypeDef USART_InitStruct;
GPIO_InitTypeDef  GPIO_InitStruct;
int gyro[3]={2,-1,-200};
uint32_t count;
int main(void)
{
	Pin_Init(9600);

	delay_01ms(5000);
//	printf("--New Reset--\r\n");
	
  delay_01ms(10000);
  while (1)
  {
		count+=5;
		printf("Hello world %d \n\r",count);
		delay_01ms(10000);
//		printf("Hello worlddd %d \r\n", count);
//		count+=5;
//		delay_01ms(5000);
//    GPIO_SetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
//		delay_01ms(5000);
//		GPIO_ResetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
  }
}
void Pin_Init(uint32_t BaudRate){
/* GPIOD Peripheral clock enable */
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

    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 10;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
	}
#define BUFFERSIZE 11
char RxBuffer[BUFFERSIZE];
__IO uint8_t RxIndex = 0x00;
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
void rcv_USART_handler(void) {
//    Kp_roll=(RxBuffer[0]-48)*100+(RxBuffer[1]-48)*10+(RxBuffer[2]-48)+(RxBuffer[4]-48)/10.0+(RxBuffer[5]-48)/100.0;
//    Ki_roll=(RxBuffer[6]-48)*100+(RxBuffer[7]-48)*10+(RxBuffer[8]-48)+(RxBuffer[10]-48)/10.0+(RxBuffer[11]-48)/100.0;
//    Kd_roll=(RxBuffer[12]-48)*100+(RxBuffer[13]-48)*10+(RxBuffer[14]-48)+(RxBuffer[16]-48)/10.0+(RxBuffer[17]-48)/100.0;
//	  Kp_pitch=Kp_roll;
//   	Ki_pitch=Ki_roll;
//   	Kd_pitch=Kd_roll;
}