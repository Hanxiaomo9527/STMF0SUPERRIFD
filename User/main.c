
#include "ToolsFun.h"
#include "USART1.h"
#include "DMA.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void)
{
		uint16_t i = 0;
   uint8_t WorkMode = 0;//工作模式
   RFPowerSettingButton = 0;//功率调整按钮为非按下状态
   Systick_Init();
   Alert_Init();
   USART1_Init(38400);
   DMA_Config();
   LED_Init();
   EXTI13_14_Config();
   while(runstate == RUNSTATEGETRF)//获取读写器功率值
   {
	    Delay_ms(300);
		GetReadRF();
   }
	
   //Card_Init();
  while (1)
  {
	  if(runstate == RUNSTATEREAD)
	  {
			  Delay_ms(60);
			  LED_ON();
			  if(DMAbusy == 0)//如果DMA空闲
			  {
				  GetTags();
			  }

			  if(alert > 0)//报警		  
			  {
				  GPIO_ResetBits(GPIOA, GPIO_Pin_3);
				  GPIO_ResetBits(GPIOA, GPIO_Pin_2);
				  
				  //GPIO_SetBits(GPIOA, GPIO_Pin_3);
				  //GPIO_SetBits(GPIOA, GPIO_Pin_2);
				  alert--;
			  }
			  else
			  {
				  alert = 0;
				  GPIO_SetBits(GPIOA, GPIO_Pin_3);
				  GPIO_SetBits(GPIOA, GPIO_Pin_2);
				  //GPIO_ResetBits(GPIOA, GPIO_Pin_3);
				  //GPIO_ResetBits(GPIOA, GPIO_Pin_2);
			  }
			  Delay_ms(60);
			  LED_OFF();
	   }
	   else if(runstate == RUNSTATESETTING)
	   {
		   alert = 0;
	   }
	   else if(runstate == RUNSTATEWAIT)
	   {
		    if(DMAbusy == 0)//如果DMA空闲
			{
				  ClearReadTag();
			}
			Delay_ms(100);
	   }
	  
	   if((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 1) && (WorkMode == 0))//加入workmode是防止一直进入此判断，即只有在首次按下该按钮的时候才进入该判断
	   {
	        for(i = 0 ; i < 30000 ; i++)
			{
				;
			}
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 1)//去抖
			{
	
				//runstate = RUNSTATEREAD;
				runstate = RUNSTATEWAIT;//如果从设定模式切换回读取模式也清空读写器
				WorkMode = 1;

			}
	    }
	   else if((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0) && (WorkMode == 1))//进入设定模式
	   {
		    for(i = 0 ; i < 30000 ; i++)
			{
				;
			}
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0)//去抖
			{
				//GPIO_ResetBits(GPIOA, GPIO_Pin_3);//清除报警信息
				//GPIO_ResetBits(GPIOA, GPIO_Pin_2);//清除报警信息
				
				GPIO_SetBits(GPIOA, GPIO_Pin_3);
				GPIO_SetBits(GPIOA, GPIO_Pin_2);
				
				runstate = RUNSTATESETTING;
				WorkMode = 0;
				RFPowerSettingButton = 0;//此处意为在设定模式下，按下功率调整按钮后，如果该按钮一直被按下，则不会一直发送功率调整信息
				                         //即只有在每次调整后并且读写器发送回信息才会进入下一次触发
			}
	   }
		
  }
}

