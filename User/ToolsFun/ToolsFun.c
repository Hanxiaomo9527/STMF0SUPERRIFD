
#include "ToolsFun.h"


//��д������
uint8_t Cmd_DateCheck[] = {0xFF , 0xFF , 0xFF , 0xFF , 0x0A , 0x3A , 0xC1 , 0x02 , 0x00 , 0x0A ,
                           0x3B , 0xE3 , 0x40 , 0x06 , 0x00 , 0x38 , 0x56 , 0xDD , 0x09 , 0x6C};
//---------------------------------------------
//��д��������ѯ��ǩ����
uint8_t Cmd_GetTagId[] = {0xFF , 0xFF , 0xFF , 0xFF , 0x6A , 0xFC , 0xC1 , 0x02 , 0x00 , 0x0C , 0xD3 , 0x74 ,
                          0x40 , 0x07 , 0x00 , 0x37 , 0x00 , 0x01 , 0x56 , 0xFF , 0x26 , 0x59};
						   

//�趨��д������
uint8_t Cmd_SetRF[] = {0xFF , 0xFF , 0xFF , 0xFF , 0xDB , 0x97 , 0xC1 , 0x02 , 0x00 , 0x07 ,
                       0xFF/*CRC*/ , 0xFF/*CRC*/ , 0x40 , 0x52 , 0x00 , 0x37 , 0xFF/*˥��ֵ0-31*/};

//��ѯ��д������
uint8_t Cmd_GetRF[] = {0xFF , 0xFF , 0xFF , 0xFF , 0xCB , 0xB6 , 0xC1 , 0x02 , 0x00 , 0x06 , 0x84 , 0x73 ,
                       0x40 , 0x61 , 0x00 , 0x37};
						  
uint8_t Cmd_ClearTag[] = {0xFF , 0xFF , 0xFF , 0xFF , 0xCB , 0xB6 , 0xC1 , 0x02 , 0x00 , 0x06 , 0x71 , 0x78 ,
                          0x40 , 0x03 , 0x00 , 0x37};
					   
uint16_t fac_ms;//ȫ�ֱ���
uint8_t fac_us;//ȫ�ֱ���
						   
uint16_t warning = 0;//Ԥ��
uint16_t alert = 0;//����

uint8_t RFPower = 0;//��д������˥����0-31
uint8_t RFPowerMin = 31;
uint8_t RFPowerMax = 0;					   
uint8_t RFPowerStep = 1;//����					   
uint8_t RFPowerSettingButton = 0;//�������ʰ�ť����״̬����ֹ��������ʱ�������Ƶ�ʹ���
						  
uint8_t runstate = RUNSTATEGETRF;//������Ϊ��ȡ����״̬�����ʻ�ȡ�ɹ������Ϊ��ȡ��ǩ״̬

uint8_t IsActive = 0;//�ж��Ƿ�Ϊ����ģʽ,��ģʽ�����ڽ��յ���д�����͵ı�ǩ���ݺ��жϸñ�ǩ�ǲ��Ǳ�������������
						  
Card_t Card_Ids[Card_Num];
const unsigned short crc_ta[16]={0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,0x8108,0x9129,
0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef};

/****************************************************
�������ܣ�ms����ʱ
���������nms : ����
�����������
��    ע�����ô˺���ǰ����Ҫ��ʼ��Delay_Init()����
*****************************************************/	
uint32_t a = 0;
void delay_ms(uint16_t nms)
{
   	  SysTick->LOAD = (uint32_t)fac_ms*nms-1;//����ʱ��ֵ
	  SysTick->VAL = 1;//���д��ֵ��������ؼĴ�����ֵ
	  SysTick->CTRL |= BIT(0);//SysTickʹ��
	  while(!(SysTick->CTRL&(1<<16))) a = SysTick->CTRL ;//�ж��Ƿ����0
	  SysTick->CTRL &=~BIT(0);//�ر�SysTick
}

/****************************************************
�������ܣ���ʱ��ʼ��
���������SYSCLK : ϵͳʱ��(72)MHZ
�����������
��    ע����
*****************************************************/
void Delay_Init(uint8_t SYSCLK)
{
     SysTick->CTRL &=~BIT(2);//ѡ���ⲿʱ��
	 SysTick->CTRL &=~BIT(1);//�رն�ʱ������0����ж�����
	 fac_us = SYSCLK/8;//�����SysTick����ֵ
	 fac_ms = (uint16_t)fac_us*1000;	 
}

/****************************************************
�������ܣ�us����ʱ
���������nus : ΢��
�����������
��    ע�����ô˺���ǰ����Ҫ��ʼ��Delay_Init()����
*****************************************************/		    								   
void delay_us(uint32_t nus)
{		
	  SysTick->LOAD = (uint32_t)fac_us*nus-1;//����ʱ��ֵ
	  SysTick->VAL = 1;//���д��ֵ��������ؼĴ�����ֵ
	  SysTick->CTRL |= BIT(0);//SysTickʹ��
	  while(!(SysTick->CTRL&(1<<16)));//�ж��Ƿ����0
	  SysTick->CTRL &=~BIT(0);//�ر�SysTick
}

/****************************************************
�������ܣ�LED��ʼ��
�����������
�����������
��    ע�����ô˺���ǰ����Ҫ��LED.h�޸ĺ궨��LED����
****************************************************/
void LED_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;

	  RCC_AHBPeriphClockCmd(LED_GPIO_CLK, ENABLE);
   
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, LED_PIN);
}

/****************************************************
�������ܣ�LED��
�����������
�����������
��    ע�����ô˺���ǰ����Ҫ��LED.h�޸ĺ궨��LED����
****************************************************/
void LED_ON(void)
{
		GPIO_ResetBits(LED_PORT, LED_PIN);
}

/****************************************************
�������ܣ�LED��
�����������
�����������
��    ע�����ô˺���ǰ����Ҫ��LED.h�޸ĺ궨��LED����
****************************************************/
void LED_OFF(void)
{
		GPIO_SetBits(LED_PORT, LED_PIN);
}


/****************************************************
�������ܣ��ܽŽų�ʼ��
�����������
�����������
��    ע���ùܽ�Ϊ���������ùܽ�
****************************************************/
void Alert_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_3);
		
	/*
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    //GPIO_SetBits(GPIOA, GPIO_Pin_3);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);

}


/****************************************************
�������ܣ���ƬCard�ṹ���ʼ��
�����������
�����������
��    ע��
****************************************************/
void Card_Init(void)
{
	uint32_t i = 0;
	for(;i< Card_Num ; i++)
	{
		Card_Ids[i].ID[0] = 0x00;
		Card_Ids[i].ID[1] = 0x00;
		Card_Ids[i].ID[2] = 0x00;
		Card_Ids[i].ID[3] = 0x00;
		Card_Ids[i].States = Inactive_States;
		if(i != (Card_Num - 1))
		{
			Card_Ids[i].Next = &Card_Ids[i+1];
		}
		else
		{
			Card_Ids[i].Next = 0x00;
		}
	}
}

//------------------------------------------------------------------------
uint32_t TimingDelay;
void Systick_Init(void)
{
        if (SysTick_Config(SystemCoreClock / 1000))//����Ϊ 1 ����
        {
                while (1);
        }
}
void TimingDelay_Decrement(void)
{
        if (TimingDelay != 0x00)
        {
                TimingDelay--;
        }
}
void Delay_ms(__IO uint32_t nTime)//�ӳٺ���������Ϊ MS
{
        TimingDelay = nTime;//ʱ�ӵδ���
        while(TimingDelay != 0);
}


/****************************************************
�������ܣ���ȡ��д���ڵı�ǩ����
���������
�����������
��    ע��
*****************************************************/	
void GetTags(void)
{
	Send_Data_DMA(Cmd_GetTagId, 22);
}

/****************************************************
�������ܣ���ѯ��д������
���������
�����������
��    ע��
*****************************************************/	
void GetReadRF(void)
{
	Send_Data_DMA(Cmd_GetRF, 16);
}

/****************************************************
�������ܣ����õ�����д�����ʴ�С��ť�ж�
���������
�����������
��    ע��PA13 PA14
*****************************************************/	
void EXTI13_14_Config(void)
{
  
  GPIO_InitTypeDef   GPIO_InitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	
  /* Configure PA7 pin as input floating *///Pin5ֻ�������ж�����main��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//  /* Connect EXTI7 Line to PA13 pin */
//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);
// 
//  
//  /* Configure EXTI7 line */
//  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//������
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);
 
	
  /* Configure PA7 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI7 Line to PA13 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource7);
 
  
  /* Configure EXTI7 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//������
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
 
  
    /* Configure PA8 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI8 Line to PA14 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);
 
  /* Configure EXTI8 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//������
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
 
   /* Enable and set EXTI4_15_IRQHandler Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
 }


/****************************************************
�������ܣ����ö�д�����ʴ�С
���������Power ˥������ 0-31 ����Ϊ5
�����������
��    ע��
*****************************************************/	
void SetRFPower(uint8_t Power)
{
	//uint8_t RFPower = 0;//��д������˥����0-31
    //uint8_t RFPowerMin = 31;
    //uint8_t RFPowerMax = 0;					   
    //uint8_t RFPowerStep = 5;//����	
	//uint8_t Cmd_SetRF[] = {0xFF , 0xFF , 0xFF , 0xFF , 0xDB , 0x97 , 0xC1 , 0x02 , 0x00 , 0x07 ,
                      // 0xFF/*CRC*/ , 0xFF/*CRC*/ , 0x40 , 0x52 , 0x00 , 0x37 , 0xFF/*˥��ֵ0-31*/};
	uint16_t check = 0;
	if(Power < RFPowerMax || Power > RFPowerMin)
	{
		return ;
	}
	
	Cmd_SetRF[16] = Power;
	check = Crc16_DATAs(&Cmd_SetRF[12] ,5);
	Cmd_SetRF[10] = (check >> 8);
	Cmd_SetRF[11] = (check & 0x00FF);
	
	Send_Data_DMA(Cmd_SetRF, 17);
	
}

/****************************************************
�������ܣ���ն�д����ǩ����
���������
�����������
��    ע��
*****************************************************/	
void ClearReadTag(void)
{
	Send_Data_DMA(Cmd_ClearTag, 16);
}



//�������ϴ�����������
//uint8_t ab = 0;
uint8_t Tags = 0;
//uint8_t abc[50];
void Process_Data(uint8_t * Data , uint8_t length)
{
	uint16_t Check = 0;
	//uint8_t Tags = 0;//�����д���ϴ���ǩ�������ж��м�����ǩ
	uint16_t Tags_ChufaqiNum = 0;//��ǩ�����д�������ŵ�λ��
	//uint16_t Tags_ID_Postion = 0;//��ǩIDλ��
	uint16_t i = 0;
	uint16_t j = 0;
	    //ab = 0;
	if(length < 16)//���ݳ��ȴ��� FF FF FF FF CRC16 CRC16 Э���� Э���� ���ݳ��� ���ݳ���  
		            //CRC16 CRC16 ֡ѡ�� ������ �豸��ַ �豸��ַ ��������Ӧ���ǳ���Ϊ16
	{return ;}
	
	Check = Crc16_DATAs(&Data[6] ,4);
	
	if( (Check >> 8) == Data[4] &&  (Check & 0x00FF) == Data[5])//�׸�У��
	{
		
		if(Data[13] == 0x06)//13Ϊ������λ��,06Ϊʱ��У������
		{
			if(length != 16){return ;}//��ָ���ϴ����ݳ���Ϊ16
			
			Check = Crc16_DATAs(&Data[12] ,4);
			if( (Check >> 8) == Data[10] &&  (Check & 0x00FF) == Data[11])//������У��
			{
				//ע�˴�Ӧ�ø��ݶ�д����ַ���Լ�ϵͳ��ǰʱ�����д������ʱ��У��������˴�ֱ�ӻظ�һ���̶�ֵ��������֮��
				Send_Data_DMA(Cmd_DateCheck , 20);
			}
			
		}
		
		if(Data[13] == 0x08)//���Ϊ��д�������ϴ���ǩ����
			                //�˴��и�ϸ����Ҫע�⣬��ÿ�δ��ڷ������ݽ�������λ�������յ��˶�����Ϣ
		                    //�������һ��ֻ����һ�����ݾͻ����©���ݣ�����ZigbeeЭ��ջ��Ϣ������ƣ�û������һ������
		                    //��Ҫ���أ����ٴ�ִ�������ж��Ƿ����ݶ�������ϣ������ڱ�����ܹ������⣬���Դ˴���δ����
		                    //�÷������
		{
		
			if( (length > 19/*��֤N���ݴ��ڣ�ָ���ֵ�����*/) && (length >= (9+20*Data[18])) )//�ٴν��г����ж�
			{
				//Data[20]-Data[23]Ϊ��ǩID
				Tags_ChufaqiNum = 25;//��26���ֽ�
				//Tags_ID_Postion = 20;//ID��ʼ��λ��
				//i = 0;
				for(Tags = 0 ; Tags < Data[18] ; Tags++)//���������ж����ǩ��Ϣ
				{
				
					    //abc[i++] = Data[Tags_ID_Postion];
						//abc[i++] = Data[Tags_ID_Postion+1];
						//abc[i++] = Data[Tags_ID_Postion+2];
						//abc[i++] = Data[Tags_ID_Postion+3];
					    warning = 35;
						if(Data[Tags_ChufaqiNum] != 0x00 || Data[Tags_ChufaqiNum+1] != 0x00)//�б�ǩ���봥����
						{
							//����
							//alert = 35;
						}
						Tags_ChufaqiNum += 20;//ÿһ����ǩ��Ϣ���ݳ���Ϊ20
					    //Tags_ID_Postion += 20;
				}
				
				/*
				if((length -10-9-20*Data[18]) > 0)//�ж�������
				{
					Process_Data(&Data[10+9+20*Data[18] - 1] , (length -10-9-20*Data[18]));
				}
				*/
				
			}
			
		
		}//end if �������Ϊ08
		//��������д��
		if(Data[13] == 0x07)//485��ȡ��ǩ����---�˴�û��ǩʱҲ������
		{
			/*
			if(Data[18] > 0)//���صı�ǩ��������0ʱ
			{
				if(IsActive == 0)//�Ǵ���ģʽ
				{
					alert = 20;//��������������ֵ��Ҫ����
				}
				else
				{
					//����ģʽ
					
				}
			}
			*/
			
			if( (length > 19/*��֤N���ݴ��ڣ�ָ���ֵ�����*/) && (length >= (9+20*Data[18])) )//�ٴν��г����ж�
			{
				//Data[20]-Data[23]Ϊ��ǩID
				Tags_ChufaqiNum = 25;//��26���ֽ�

				for(Tags = 0 ; Tags < Data[18] ; Tags++)//���������ж����ǩ��Ϣ
				{
						
					    if(IsActive == 0)//�Ǵ���ģʽ
						{							
					 
							if(Data[Tags_ChufaqiNum] == 0x00 || Data[Tags_ChufaqiNum+1] == 0x00)//������ǩ����
							{
								//����///////////�޸ģ�2016/4/27)
								alert = 26;//20
							}

						}
						else//����ģʽ
						{
							if(Data[Tags_ChufaqiNum] != 0x00 || Data[Tags_ChufaqiNum+1] != 0x00)//������ǩ����
							{
								//����///////////�޸ģ�2016/4/18��
								alert = 39;//20
								//ab++;
							}
						}
						Tags_ChufaqiNum += 20;//ÿһ����ǩ��Ϣ���ݳ���Ϊ20
						
					    
				}
				
				
			}
			
		}
		//���ù��ʷ�������
		if(Data[13] == 0x52)
		{
			if(Data[16] == 0x00)//�ɹ�
			{
				//ab++;
				RFPowerSettingButton = 0;
				LED_ON();
				for(i = 0 ; i < 1500 ; i++)
				{
					for(j = 0 ; j < 200 ; j++)
					{
						;
					}
				}
				LED_OFF();
			}
		}
		//��ȡ��д���������������,��ȡ���ʺ������豸�ȴ���ť����
		if(Data[13] == 0x61)
		{
			RFPower = Data[16];//���ع���ֵ
			//runstate = RUNSTATEREAD;
			runstate = RUNSTATESETTING;//�ȴ���ն�д���ڴ�
		}
		
		//��ն�д���ڴ�,�ɹ�
		if((Data[13] == 0x03) && (Data[16] == 0x00))
		{
			runstate = RUNSTATEREAD;
		}
		
	}

}

unsigned short Crc16_DATAs(unsigned char *ptr, unsigned short len) 
{
	unsigned char da;   
	unsigned short  CRC_16_Data = 0;
	while(len-- != 0)
	{
		da = CRC_16_Data >> 12; 
		CRC_16_Data <<= 4; 
		CRC_16_Data ^= crc_ta[da^(*ptr/16)];                                   
		da = CRC_16_Data >> 12; 
		CRC_16_Data <<= 4; 
		CRC_16_Data ^= crc_ta[da^(*ptr&0x0f)]; 
		ptr++;
	}
	return  CRC_16_Data;
}


