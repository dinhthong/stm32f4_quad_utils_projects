#include <stdio.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

static void TaskA(void *pvParameters);
static void TaskB(void *pvParameters);
static void TaskC(void *pvParameters);
static void TaskD(void *pvParameters);
void GPIO_Configuration(void);
void USART_Configuration(unsigned int BaudRate);
void Delay(__IO uint32_t nCount);
void TIM_IC_Config(void);
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
TIM_ICInitTypeDef  TIM_ICInitStructure;
__IO uint32_t IC2Value = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;
uint16_t RCvalue,channel_count,cnt,channels[10];
int main(void)
{
    GPIO_Configuration();
    USART_Configuration(115200);
    TIM_IC_Config();
    printf("-------FreeRTOS------\r\n");
    /*Doi so thu 2, gia tri gi cung duoc, 
doi so 3: 128: so stack. stack: nhung cac bien thuoc function.
vi du neu trong ham taskA khai bao int a,b,c -> tinh toan duoc so stack can dung.
nhung thuc ra chua du, vi con lien quan den cac ham con ( nhu GPIO_WritePin ), ... ->
cach nay rat lau. 
NULL: lien quan den doi so trong dinh nghiax static ..
    */
    xTaskCreate(TaskA, "TaskA", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskB, "TaskB", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskC, "TaskC", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskD, "TaskD", 128, NULL, tskIDLE_PRIORITY, NULL);
    vTaskStartScheduler();
}

static void TaskA(void *pvParameters) //NULL
{  
    for(;;) 
    {
	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
	vTaskDelay(1500); //block state -> cac task khac co the xu ly tiep
    }
}


static void TaskB(void *pvParameters)
{ 
    for(;;) 
    {
	GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
	vTaskDelay(250);
    }
}

static void TaskC(void *pvParameters)
{   
    for(;;) 
    {
	printf("Channels %d\n\r",channels[3]);
	vTaskDelay(100);
    }
}

static void TaskD(void *pvParameters)
{   
    for(;;) 
    {
	GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
	vTaskDelay(1500);
    }
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOA, ENABLE);
    
    /* Configure PB0 PB1 in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    /* Configure PA0 in input mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void USART_Configuration(unsigned int BaudRate)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = BaudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); 
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); 
    
    USART_Cmd(USART1, ENABLE);  
}

void TIM_IC_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* TIM4 configuration: PWM Input mode ------------------------
     The external signal is connected to TIM4 CH2 pin (PB.07), 
     The Rising edge is used as active edge,
     The TIM4 CCR2 is used to compute the frequency value 
     The TIM4 CCR1 is used to compute the duty cycle value
  ------------------------------------------------------------ */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

  /* Select the TIM3 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);

  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

  /* Connect TIM pin to AF2 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

  /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

void TIM2_IRQHandler(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);

  /* Clear TIM4 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC2Value = TIM_GetCapture2(TIM2);

  if (IC2Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle = (TIM_GetCapture1(TIM2) * 100) / IC2Value;

    /* Frequency computation 
       TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */

    Frequency = (RCC_Clocks.HCLK_Frequency)/2 / IC2Value;
		
  }
  else
  {
    DutyCycle = 0;
    Frequency = 0;
  }
	//number of pulses with sample rate = 1 us
			RCvalue=IC2Value/84;
			if (RCvalue>=3000) // seperator for each RC read loop
			{
				//channel_count=cnt-1;
				cnt=0;
			}
			else
			{
				  if (RCvalue>900){
					channels[cnt]=RCvalue;}
					cnt++;
			}
//				for (int i;i<=channel_count;i++)
//				{
//					if (channels[i]>=900||channels[i]<=2000)
//					{
//						
//					}
//				}
//				update_channels();
//				channels[channel_count]=0;
				
}
void Delay(__IO uint32_t nCount)
{
    while(nCount--)
    {
    }
}

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(USART1, (uint8_t) ch);
    
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}
    
    return ch;
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    
    /* Infinite loop */
    while (1)
    {
    }
}
#endif

