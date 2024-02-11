/**
  ******************************************************************************
  * @file    TIM_PWM_Input/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup TIM_PWM_Input
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t IC2Value = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
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
  {}
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
  {}
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
  {}
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
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
uint16_t RCvalue,channel_count,cnt,channels[10],channels_buffer[10];
volatile	uint32_t cnt_not8=0;
void TIM2_IRQHandler(void)
{
  /* Get the Input Capture value */
  IC2Value = TIM_GetCapture2(TIM2);

	//number of pulses with sample rate = 1 us
			RCvalue=IC2Value/84;
				if (RCvalue>=2500) // seperator for each RC read loop
			{
				//channel_count=cnt-1;
				/*
				if cnt!= number of channels. it means that the readings is wrong somehow. 
				Unless that case, we update the RC readings.
				*/
				if (cnt==8)
				{
					RC_update();
				}
				else {
					cnt_not8++;
				}
				cnt=0;
			}
			else
			{
				// if the readings < 1050 -> wrong -> cnt won't increase -> won't update RC
					if (RCvalue>=1050)
					{
					channels_buffer[cnt]=RCvalue;
					cnt++;
					}
			}
				 TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
}
void RC_update(void)
{
	uint8_t i;
	for (i=0;i<=8;i++)
	{
		channels[i]=channels_buffer[i];
	}
}
//uint16_t channel_thro,channel_elev,channel_aile,channel_rudd,channel_gear,channel_aux1,channel_aux2;
//void update_channels(uint16_t CH_THRO,uint16_t CH_ELEV,uint16_t CH_AILE,uint16_t CH_RUDD,uint16_t CH_GEAR,uint16_t CH_AUX1,uint16_t CH_AUX2)
//{
//	channel_thro=CH_THRO;
//	channel_elev=CH_ELEV;
//	channel_aile=CH_AILE;
//	channel_rudd=CH_RUDD;
//	channel_gear=CH_GEAR;
//	channel_aux1=CH_AUX1;
//	channel_aux2=CH_AUX2;
//}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
