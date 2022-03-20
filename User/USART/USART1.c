#include "USART1.h"
#include "DMA.h"

/* USART初始化 */
void USART1_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  //使能GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//使能USART的时钟
	/* USART1的端口配置 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);//配置PA9成第二功能引脚	TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);//配置PA10成第二功能引脚  RX	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	/* USART1的基本配置 */
	USART_InitStructure.USART_BaudRate = baud;              //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);	
	
    USART_ITConfig(USART1,USART_IT_TC,DISABLE); 
    USART_ITConfig(USART1,USART_IT_RXNE,DISABLE); 
    USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
    //采用DMA方式接收 
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    //采用DMA方式发送
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART1, ENABLE);                             //使能USART1
	
	//USART_ITConfig(USART1, USART_IT_IDLE , ENABLE);//开启空闲,帧错,噪声,校验错中断 
	
	/* USART1的NVIC中断配置 */
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
				
}

//=============================================================================
//文件名称：
//功能概要：USART1中断函数
//参数说明：无
//函数返回：无
//=============================================================================
uint16_t DATA_LEN;
void USART1_IRQHandler(void)
{

        if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)//如果为空闲总线中断
		{
                DMA_Cmd(DMA1_Channel3, DISABLE);//关闭DMA,防止处理其间有数据
                //USART_RX_STA = USART1->SR;//先读SR，然后读DR才能清除
				//USART_RX_STA = USART1->DR;
			
                 DATA_LEN=500-DMA_GetCurrDataCounter(DMA1_Channel3); //计算有多少待发送数据
                if(DATA_LEN > 0)//如果接收到数据
				{ 
					Process_Data(USART1_RECEIVE_DATA,DATA_LEN);//处理该条数据
                }
                //DMA_Cmd(DMA1_Channel5, DISABLE);//关闭DMA,防止处理其间有数据
                DMA_ClearFlag(DMA1_FLAG_GL3 | DMA1_FLAG_TC3 | DMA1_FLAG_TE3 | DMA1_FLAG_HT3);//清标志
                DMA1_Channel3->CNDTR = 500;//重装填
                DMA_Cmd(DMA1_Channel3, ENABLE);//处理完,重开DMA
                //读SR后读DR清除Idle
                USART_ClearITPendingBit(USART1,USART_IT_IDLE);
				//USART1->CR1 &= 0x ;
                //i = USART1->DR;
        }
        if(USART_GetITStatus(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE) != RESET)//出错
        {
                USART_ClearITPendingBit(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE);
        }
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);
			
}








