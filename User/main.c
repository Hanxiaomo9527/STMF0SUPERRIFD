
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
   uint8_t WorkMode = 0;//����ģʽ
   RFPowerSettingButton = 0;//���ʵ�����ťΪ�ǰ���״̬
   Systick_Init();
   Alert_Init();
   USART1_Init(38400);
   DMA_Config();
   LED_Init();
   EXTI13_14_Config();
   while(runstate == RUNSTATEGETRF)//��ȡ��д������ֵ
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
			  if(DMAbusy == 0)//���DMA����
			  {
				  GetTags();
			  }

			  if(alert > 0)//����		  
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
		    if(DMAbusy == 0)//���DMA����
			{
				  ClearReadTag();
			}
			Delay_ms(100);
	   }
	  
	   if((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 1) && (WorkMode == 0))//����workmode�Ƿ�ֹһֱ������жϣ���ֻ�����״ΰ��¸ð�ť��ʱ��Ž�����ж�
	   {
	        for(i = 0 ; i < 30000 ; i++)
			{
				;
			}
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 1)//ȥ��
			{
	
				//runstate = RUNSTATEREAD;
				runstate = RUNSTATEWAIT;//������趨ģʽ�л��ض�ȡģʽҲ��ն�д��
				WorkMode = 1;

			}
	    }
	   else if((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0) && (WorkMode == 1))//�����趨ģʽ
	   {
		    for(i = 0 ; i < 30000 ; i++)
			{
				;
			}
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0)//ȥ��
			{
				//GPIO_ResetBits(GPIOA, GPIO_Pin_3);//���������Ϣ
				//GPIO_ResetBits(GPIOA, GPIO_Pin_2);//���������Ϣ
				
				GPIO_SetBits(GPIOA, GPIO_Pin_3);
				GPIO_SetBits(GPIOA, GPIO_Pin_2);
				
				runstate = RUNSTATESETTING;
				WorkMode = 0;
				RFPowerSettingButton = 0;//�˴���Ϊ���趨ģʽ�£����¹��ʵ�����ť������ð�ťһֱ�����£��򲻻�һֱ���͹��ʵ�����Ϣ
				                         //��ֻ����ÿ�ε������Ҷ�д�����ͻ���Ϣ�Ż������һ�δ���
			}
	   }
		
  }
}

