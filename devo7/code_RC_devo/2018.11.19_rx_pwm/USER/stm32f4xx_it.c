#include "stm32f4xx_it.h"
#include <math.h>

void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

}
/*
Input Capture Timer2 for RC channels readings.
In this case number of channels is predetermined by me.
7 channels, PPM.
More works will be made for universal cases.
*/
//__IO uint32_t IC2Value = 0;
//uint16_t RCvalue;
//uint16_t channels_buffer[10],channels[10];
//uint8_t channel_read_fail;
//uint16_t PPMval;
//uint8_t channels_cnt;
//uint32_t lastRC_us;
//void EXTI0_IRQHandler(void)
//{
///* Make sure that interrupt flag is set */
//	  if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
//	  	  PPMval = TIM5->CNT - lastRC_us;
//	  	  lastRC_us = TIM5->CNT;
//	  		if (PPMval > 10000) {
//						if (channels_cnt == 8 && channel_read_fail == 0) {
//						    RC_update();
//					  }
//					  channels_cnt=0;
//					  channel_read_fail=0;
//				}
//			  else {
//					  if (PPMval > 1050 && PPMval < 1950) {
//						    channels_buffer[channels_cnt++] = PPMval;
//					  }
//					  else {
//						channel_read_fail = 1;
//					  }
//				}
//    /* Clear interrupt flag */
//		EXTI->PR = EXTI_Line0;
//		}

//}
//void RC_update(void) {
//		for (uint8_t i=0; i<8; i++) {
//				channels[i] = channels_buffer[i];
//	  } 
//}
#define BUFFERSIZE 18
char RxBuffer[BUFFERSIZE];
char pidtuneBuffer[16];
float pidtune_data;
uint8_t tune_idx, dataidx;
__IO uint8_t RxIndex = 0x00;
//extern float Kp_Alt_PID, Ki_Alt_PID , Kd_Alt_PID;
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {

    }

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
//        //read from USART_DR will automatically clear the RNXE bit.
//        RxBuffer[RxIndex] = USART_ReceiveData(USART2);
//						// start of data frame character.
//			  if (RxBuffer[RxIndex] == '!') {
//					tune_idx = 0;
//				}
//						// end of data frame character.
//				else if (RxBuffer[RxIndex] == '@') {
//					// first two number is decimal value.
//					pidtune_data = (float)(pidtuneBuffer[1]-48)*10.0f + (float)(pidtuneBuffer[2]-48);
//					// pidtuneBuffer[3] == '.'
//					// then < 1 values
//					for (dataidx = 4; dataidx <= (tune_idx-1); dataidx++) {
//						pidtune_data += (float)(pidtuneBuffer[dataidx]-48)/(pow(10.0f,dataidx-3.0f));
//					}
//					if (pidtuneBuffer[0] == 'p') {
//						Kp_Alt_PID = pidtune_data;
//		//				printf("Kp is: %f\n\r", Kp_Alt_PID);
//					}
//					if (pidtuneBuffer[0] == 'i') {
//						Ki_Alt_PID = pidtune_data;
//			//			printf("Ki is: %f\n\r", Ki_Alt_PID);
//					}
//					if (pidtuneBuffer[0] == 'd') {
//						Kd_Alt_PID = pidtune_data;
//				//		printf("Kd is: %f\n\r", Ki_Alt_PID);
//					}
//				}
//				else {
//					pidtuneBuffer[tune_idx] = RxBuffer[RxIndex];
//					tune_idx++;
//				}
//			  RxIndex++;
//        if (RxIndex >= BUFFERSIZE)
//        {
//            RxIndex=0;
//            /* Receive Transaction data 
//					  This will be called every time buffer is full*/
//            rcv_USART_handler();
//        }
    }
}

void rcv_USART_handler(void)
{
//    Kp_roll=(RxBuffer[0]-48)*100+(RxBuffer[1]-48)*10+(RxBuffer[2]-48)+(RxBuffer[4]-48)/10.0+(RxBuffer[5]-48)/100.0;
//    Ki_roll=(RxBuffer[6]-48)*100+(RxBuffer[7]-48)*10+(RxBuffer[8]-48)+(RxBuffer[10]-48)/10.0+(RxBuffer[11]-48)/100.0;
//    Kd_roll=(RxBuffer[12]-48)*100+(RxBuffer[13]-48)*10+(RxBuffer[14]-48)+(RxBuffer[16]-48)/10.0+(RxBuffer[17]-48)/100.0;
//	  Kp_pitch=Kp_roll;
//   	Ki_pitch=Ki_roll;
//   	Kd_pitch=Kd_roll;
}

float getAvg(float *buff, int size)
{
	float sum = 0;
	int i;
	for(i = 0; i < size; i++)
	{
		sum += buff[i];
	}
	return sum / size;
}