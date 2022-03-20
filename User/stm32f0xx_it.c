/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    05-December-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include "ToolsFun.h"
#include "DMA.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
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
	TimingDelay_Decrement();
}
void EXTI4_15_IRQHandler(void)
{
	//extern uint8_t RFPower;//读写器功率衰减，0-31
    //extern uint8_t RFPowerMin;
    //extern uint8_t RFPowerMax;					   
    //extern uint8_t RFPowerStep;//步进
	uint16_t delay = 0;
	uint16_t i = 0;
	
//	if(EXTI_GetITStatus(EXTI_Line5) != RESET)//模式设定
//	{
//			for(delay = 0 ; delay < 30000 ; delay++)
//			{
//				;
//			}
//			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 1)//去抖
//			{
//				if(runstate == RUNSTATEREAD)
//				{
//				   runstate = RUNSTATESETTING;
//				}
//				else
//				{
//				   //runstate = RUNSTATEREAD;
//					runstate = RUNSTATEWAIT;//如果从设定模式切换回读取模式也清空读写器
//				}

//			}
//		
//		/* Clear the EXTI line 5 pending bit */
//		EXTI_ClearITPendingBit(EXTI_Line5);
//	}
	
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)//减小衰减,放大功率
	{
		if((runstate == RUNSTATESETTING)  && (RFPowerSettingButton == 0))
		{
			for(delay = 0 ; delay < 40000 ; delay++)
			{
				;
			}
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == 1)//去抖
			{
				if((RFPower - RFPowerStep) >= RFPowerMax)
				{
					RFPower -= RFPowerStep;
				}
				else
				{
					RFPower = RFPowerMax;
				}
				
				if(DMAbusy == 0)
				{
					for(delay = 0 ; delay < 10000 ; delay++)
					{
						for(i = 0 ; i <300 ; i++)
						{						
							;
						}
					}
					SetRFPower(RFPower);
					RFPowerSettingButton = 1;
				}
			}
		}	
		/* Clear the EXTI line 7 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
 
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)//放大衰减，减小功率
	{
        if((runstate == RUNSTATESETTING) && (RFPowerSettingButton == 0))
		{
			for(delay = 0 ; delay < 40000 ; delay++)
			{
				;
			}
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 1)//去抖
			{	
				if((RFPower + RFPowerStep) <= RFPowerMin)
				{
					RFPower += RFPowerStep;
				}
				else
				{
					RFPower = RFPowerMin;
				}
				
				if(DMAbusy == 0)
				{
					for(delay = 0 ; delay < 10000 ; delay++)
					{
						for(i = 0 ; i <300 ; i++)
						{						
							;
						}
					}
					SetRFPower(RFPower);
					RFPowerSettingButton = 1;
				}
			}
		}
		/* Clear the EXTI line 6 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line6);
	}

}


/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/

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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
