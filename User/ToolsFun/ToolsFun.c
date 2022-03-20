
#include "ToolsFun.h"


//读写器命令
uint8_t Cmd_DateCheck[] = {0xFF , 0xFF , 0xFF , 0xFF , 0x0A , 0x3A , 0xC1 , 0x02 , 0x00 , 0x0A ,
                           0x3B , 0xE3 , 0x40 , 0x06 , 0x00 , 0x38 , 0x56 , 0xDD , 0x09 , 0x6C};
//---------------------------------------------
//读写器主动查询标签数据
uint8_t Cmd_GetTagId[] = {0xFF , 0xFF , 0xFF , 0xFF , 0x6A , 0xFC , 0xC1 , 0x02 , 0x00 , 0x0C , 0xD3 , 0x74 ,
                          0x40 , 0x07 , 0x00 , 0x37 , 0x00 , 0x01 , 0x56 , 0xFF , 0x26 , 0x59};
						   

//设定读写器功率
uint8_t Cmd_SetRF[] = {0xFF , 0xFF , 0xFF , 0xFF , 0xDB , 0x97 , 0xC1 , 0x02 , 0x00 , 0x07 ,
                       0xFF/*CRC*/ , 0xFF/*CRC*/ , 0x40 , 0x52 , 0x00 , 0x37 , 0xFF/*衰减值0-31*/};

//查询读写器功率
uint8_t Cmd_GetRF[] = {0xFF , 0xFF , 0xFF , 0xFF , 0xCB , 0xB6 , 0xC1 , 0x02 , 0x00 , 0x06 , 0x84 , 0x73 ,
                       0x40 , 0x61 , 0x00 , 0x37};
						  
uint8_t Cmd_ClearTag[] = {0xFF , 0xFF , 0xFF , 0xFF , 0xCB , 0xB6 , 0xC1 , 0x02 , 0x00 , 0x06 , 0x71 , 0x78 ,
                          0x40 , 0x03 , 0x00 , 0x37};
					   
uint16_t fac_ms;//全局变量
uint8_t fac_us;//全局变量
						   
uint16_t warning = 0;//预警
uint16_t alert = 0;//报警

uint8_t RFPower = 0;//读写器功率衰减，0-31
uint8_t RFPowerMin = 31;
uint8_t RFPowerMax = 0;					   
uint8_t RFPowerStep = 1;//步进					   
uint8_t RFPowerSettingButton = 0;//调整功率按钮按下状态，防止连续按的时候命令发送频率过快
						  
uint8_t runstate = RUNSTATEGETRF;//启动后为获取功率状态，功率获取成功后更改为读取标签状态

uint8_t IsActive = 0;//判断是否为触发模式,此模式控制在接收到读写器发送的标签数据后，判断该标签是不是被触发器触发的
						  
Card_t Card_Ids[Card_Num];
const unsigned short crc_ta[16]={0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,0x8108,0x9129,
0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef};

/****************************************************
函数功能：ms级延时
输入参数：nms : 毫秒
输出参数：无
备    注：调用此函数前，需要初始化Delay_Init()函数
*****************************************************/	
uint32_t a = 0;
void delay_ms(uint16_t nms)
{
   	  SysTick->LOAD = (uint32_t)fac_ms*nms-1;//加载时间值
	  SysTick->VAL = 1;//随便写个值，清除加载寄存器的值
	  SysTick->CTRL |= BIT(0);//SysTick使能
	  while(!(SysTick->CTRL&(1<<16))) a = SysTick->CTRL ;//判断是否减到0
	  SysTick->CTRL &=~BIT(0);//关闭SysTick
}

/****************************************************
函数功能：延时初始化
输入参数：SYSCLK : 系统时钟(72)MHZ
输出参数：无
备    注：无
*****************************************************/
void Delay_Init(uint8_t SYSCLK)
{
     SysTick->CTRL &=~BIT(2);//选择外部时钟
	 SysTick->CTRL &=~BIT(1);//关闭定时器减到0后的中断请求
	 fac_us = SYSCLK/8;//计算好SysTick加载值
	 fac_ms = (uint16_t)fac_us*1000;	 
}

/****************************************************
函数功能：us级延时
输入参数：nus : 微秒
输出参数：无
备    注：调用此函数前，需要初始化Delay_Init()函数
*****************************************************/		    								   
void delay_us(uint32_t nus)
{		
	  SysTick->LOAD = (uint32_t)fac_us*nus-1;//加载时间值
	  SysTick->VAL = 1;//随便写个值，清除加载寄存器的值
	  SysTick->CTRL |= BIT(0);//SysTick使能
	  while(!(SysTick->CTRL&(1<<16)));//判断是否减到0
	  SysTick->CTRL &=~BIT(0);//关闭SysTick
}

/****************************************************
函数功能：LED初始化
输入参数：无
输出参数：无
备    注：调用此函数前，需要在LED.h修改宏定义LED引脚
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
函数功能：LED开
输入参数：无
输出参数：无
备    注：调用此函数前，需要在LED.h修改宏定义LED引脚
****************************************************/
void LED_ON(void)
{
		GPIO_ResetBits(LED_PORT, LED_PIN);
}

/****************************************************
函数功能：LED关
输入参数：无
输出参数：无
备    注：调用此函数前，需要在LED.h修改宏定义LED引脚
****************************************************/
void LED_OFF(void)
{
		GPIO_SetBits(LED_PORT, LED_PIN);
}


/****************************************************
函数功能：管脚脚初始化
输入参数：无
输出参数：无
备    注：该管脚为触发报警用管脚
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
函数功能：卡片Card结构体初始化
输入参数：无
输出参数：无
备    注：
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
        if (SysTick_Config(SystemCoreClock / 1000))//设置为 1 毫秒
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
void Delay_ms(__IO uint32_t nTime)//延迟函数，设置为 MS
{
        TimingDelay = nTime;//时钟滴答数
        while(TimingDelay != 0);
}


/****************************************************
函数功能：获取读写器内的标签数据
输入参数：
输出参数：无
备    注：
*****************************************************/	
void GetTags(void)
{
	Send_Data_DMA(Cmd_GetTagId, 22);
}

/****************************************************
函数功能：查询读写器功率
输入参数：
输出参数：无
备    注：
*****************************************************/	
void GetReadRF(void)
{
	Send_Data_DMA(Cmd_GetRF, 16);
}

/****************************************************
函数功能：设置调整读写器功率大小按钮中断
输入参数：
输出参数：无
备    注：PA13 PA14
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
	
	
  /* Configure PA7 pin as input floating *///Pin5只做输入判定，由main函数处理
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉？
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//  /* Connect EXTI7 Line to PA13 pin */
//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);
// 
//  
//  /* Configure EXTI7 line */
//  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//上升沿
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);
 
	
  /* Configure PA7 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉？
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI7 Line to PA13 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource7);
 
  
  /* Configure EXTI7 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//上升沿
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
 
  
    /* Configure PA8 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉？
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI8 Line to PA14 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);
 
  /* Configure EXTI8 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//上升沿
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
 
   /* Enable and set EXTI4_15_IRQHandler Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
 }


/****************************************************
函数功能：设置读写器功率大小
输入参数：Power 衰减功率 0-31 步进为5
输出参数：无
备    注：
*****************************************************/	
void SetRFPower(uint8_t Power)
{
	//uint8_t RFPower = 0;//读写器功率衰减，0-31
    //uint8_t RFPowerMin = 31;
    //uint8_t RFPowerMax = 0;					   
    //uint8_t RFPowerStep = 5;//步进	
	//uint8_t Cmd_SetRF[] = {0xFF , 0xFF , 0xFF , 0xFF , 0xDB , 0x97 , 0xC1 , 0x02 , 0x00 , 0x07 ,
                      // 0xFF/*CRC*/ , 0xFF/*CRC*/ , 0x40 , 0x52 , 0x00 , 0x37 , 0xFF/*衰减值0-31*/};
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
函数功能：清空读写器标签数据
输入参数：
输出参数：无
备    注：
*****************************************************/	
void ClearReadTag(void)
{
	Send_Data_DMA(Cmd_ClearTag, 16);
}



//处理串口上传过来的数据
//uint8_t ab = 0;
uint8_t Tags = 0;
//uint8_t abc[50];
void Process_Data(uint8_t * Data , uint8_t length)
{
	uint16_t Check = 0;
	//uint8_t Tags = 0;//处理读写器上传标签数据中判断有几个标签
	uint16_t Tags_ChufaqiNum = 0;//标签数据中触发器编号的位置
	//uint16_t Tags_ID_Postion = 0;//标签ID位置
	uint16_t i = 0;
	uint16_t j = 0;
	    //ab = 0;
	if(length < 16)//数据长度错误 FF FF FF FF CRC16 CRC16 协议码 协议码 数据长度 数据长度  
		            //CRC16 CRC16 帧选项 命令码 设备地址 设备地址 所以至少应该是长度为16
	{return ;}
	
	Check = Crc16_DATAs(&Data[6] ,4);
	
	if( (Check >> 8) == Data[4] &&  (Check & 0x00FF) == Data[5])//首个校验
	{
		
		if(Data[13] == 0x06)//13为命令码位置,06为时间校验命令
		{
			if(length != 16){return ;}//该指令上传数据长度为16
			
			Check = Crc16_DATAs(&Data[12] ,4);
			if( (Check >> 8) == Data[10] &&  (Check & 0x00FF) == Data[11])//数据项校验
			{
				//注此处应该根据读写器地址，以及系统当前时间向读写器发送时间校验命令，但此处直接回复一个固定值以做测试之用
				Send_Data_DMA(Cmd_DateCheck , 20);
			}
			
		}
		
		if(Data[13] == 0x08)//如果为读写器主动上传标签命令
			                //此处有个细节需要注意，在每次串口发送数据结束后，上位机可能收到了多条信息
		                    //所以如果一次只处理一条数据就会出现漏数据，联想Zigbee协议栈消息处理机制，没处理完一次数据
		                    //还要返回，并再次执行用以判断是否数据都处理完毕，但由于本程序架构的问题，所以此处并未按照
		                    //该方法设计
		{
		
			if( (length > 19/*保证N数据存在，指向该值不溢出*/) && (length >= (9+20*Data[18])) )//再次进行长度判断
			{
				//Data[20]-Data[23]为标签ID
				Tags_ChufaqiNum = 25;//第26个字节
				//Tags_ID_Postion = 20;//ID开始的位置
				//i = 0;
				for(Tags = 0 ; Tags < Data[18] ; Tags++)//单条数据有多个标签信息
				{
				
					    //abc[i++] = Data[Tags_ID_Postion];
						//abc[i++] = Data[Tags_ID_Postion+1];
						//abc[i++] = Data[Tags_ID_Postion+2];
						//abc[i++] = Data[Tags_ID_Postion+3];
					    warning = 35;
						if(Data[Tags_ChufaqiNum] != 0x00 || Data[Tags_ChufaqiNum+1] != 0x00)//有标签进入触发区
						{
							//报警
							//alert = 35;
						}
						Tags_ChufaqiNum += 20;//每一个标签信息数据长度为20
					    //Tags_ID_Postion += 20;
				}
				
				/*
				if((length -10-9-20*Data[18]) > 0)//有多条数据
				{
					Process_Data(&Data[10+9+20*Data[18] - 1] , (length -10-9-20*Data[18]));
				}
				*/
				
			}
			
		
		}//end if 命令代码为08
		//报警单读写器
		if(Data[13] == 0x07)//485获取标签数据---此处没标签时也会增加
		{
			/*
			if(Data[18] > 0)//返回的标签数量大于0时
			{
				if(IsActive == 0)//非触发模式
				{
					alert = 20;//报警次数，此数值需要测试
				}
				else
				{
					//触发模式
					
				}
			}
			*/
			
			if( (length > 19/*保证N数据存在，指向该值不溢出*/) && (length >= (9+20*Data[18])) )//再次进行长度判断
			{
				//Data[20]-Data[23]为标签ID
				Tags_ChufaqiNum = 25;//第26个字节

				for(Tags = 0 ; Tags < Data[18] ; Tags++)//单条数据有多个标签信息
				{
						
					    if(IsActive == 0)//非触发模式
						{							
					 
							if(Data[Tags_ChufaqiNum] == 0x00 || Data[Tags_ChufaqiNum+1] == 0x00)//常发标签进入
							{
								//报警///////////修改（2016/4/27)
								alert = 26;//20
							}

						}
						else//触发模式
						{
							if(Data[Tags_ChufaqiNum] != 0x00 || Data[Tags_ChufaqiNum+1] != 0x00)//触发标签进入
							{
								//报警///////////修改（2016/4/18）
								alert = 39;//20
								//ab++;
							}
						}
						Tags_ChufaqiNum += 20;//每一个标签信息数据长度为20
						
					    
				}
				
				
			}
			
		}
		//设置功率返回命令
		if(Data[13] == 0x52)
		{
			if(Data[16] == 0x00)//成功
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
		//获取读写器功率命令返回数据,获取功率后不启动设备等待按钮启动
		if(Data[13] == 0x61)
		{
			RFPower = Data[16];//返回功率值
			//runstate = RUNSTATEREAD;
			runstate = RUNSTATESETTING;//等待清空读写器内存
		}
		
		//清空读写器内存,成功
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


