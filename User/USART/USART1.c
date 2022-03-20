#include "USART1.h"
#include "DMA.h"

/* USART��ʼ�� */
void USART1_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  //ʹ��GPIOA��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//ʹ��USART��ʱ��
	/* USART1�Ķ˿����� */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);//����PA9�ɵڶ���������	TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);//����PA10�ɵڶ���������  RX	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	/* USART1�Ļ������� */
	USART_InitStructure.USART_BaudRate = baud;              //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);	
	
    USART_ITConfig(USART1,USART_IT_TC,DISABLE); 
    USART_ITConfig(USART1,USART_IT_RXNE,DISABLE); 
    USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
    //����DMA��ʽ���� 
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    //����DMA��ʽ����
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART1, ENABLE);                             //ʹ��USART1
	
	//USART_ITConfig(USART1, USART_IT_IDLE , ENABLE);//��������,֡��,����,У����ж� 
	
	/* USART1��NVIC�ж����� */
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
				
}

//=============================================================================
//�ļ����ƣ�
//���ܸ�Ҫ��USART1�жϺ���
//����˵������
//�������أ���
//=============================================================================
uint16_t DATA_LEN;
void USART1_IRQHandler(void)
{

        if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)//���Ϊ���������ж�
		{
                DMA_Cmd(DMA1_Channel3, DISABLE);//�ر�DMA,��ֹ�������������
                //USART_RX_STA = USART1->SR;//�ȶ�SR��Ȼ���DR�������
				//USART_RX_STA = USART1->DR;
			
                 DATA_LEN=500-DMA_GetCurrDataCounter(DMA1_Channel3); //�����ж��ٴ���������
                if(DATA_LEN > 0)//������յ�����
				{ 
					Process_Data(USART1_RECEIVE_DATA,DATA_LEN);//�����������
                }
                //DMA_Cmd(DMA1_Channel5, DISABLE);//�ر�DMA,��ֹ�������������
                DMA_ClearFlag(DMA1_FLAG_GL3 | DMA1_FLAG_TC3 | DMA1_FLAG_TE3 | DMA1_FLAG_HT3);//���־
                DMA1_Channel3->CNDTR = 500;//��װ��
                DMA_Cmd(DMA1_Channel3, ENABLE);//������,�ؿ�DMA
                //��SR���DR���Idle
                USART_ClearITPendingBit(USART1,USART_IT_IDLE);
				//USART1->CR1 &= 0x ;
                //i = USART1->DR;
        }
        if(USART_GetITStatus(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE) != RESET)//����
        {
                USART_ClearITPendingBit(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE);
        }
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);
			
}








