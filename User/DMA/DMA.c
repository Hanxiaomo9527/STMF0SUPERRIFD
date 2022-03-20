#include "DMA.h"
#define USART1_RDR_Address   0x40013824 
uint8_t USART1_RECEIVE_DATA[500];
uint8_t USART1_SEND_DATA[100];
uint8_t DMAbusy = 0;

//USART1 DMA �趨
void DMA_Config(void)
{
   DMA_InitTypeDef  DMA_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   /* Enable the DMA periph */
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	  /*------------------------------- DMA---------------------------------------*/   
  /* Common DMA configuration */
  /*USART1 Rx*/
   DMA_DeInit(DMA1_Channel3);
   DMA_InitStructure.DMA_BufferSize = 500;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//������С���ݵ�λ
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//������С���ݵ�λ
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
   
   /* DMA1 Channel3 configuration */
   DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART1_RECEIVE_DATA;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
   DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->RDR; //USART1_RDR_Address;//USART1->RDR;
   DMA_Init(DMA1_Channel3, &DMA_InitStructure);
   
   //DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
   //DMA_ITConfig(DMA1_Channel3, DMA_IT_TE, ENABLE);
  
   //-------------------------------------------------------------------------  
   
   /* Common DMA configuration */
   /*USART1 Tx*/
   DMA_DeInit(DMA1_Channel2);
   DMA_InitStructure.DMA_BufferSize = 100;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//������С���ݵ�λ
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//������С���ݵ�λ
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
   
   /* DMA1 Channel2 configuration */
   DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART1_SEND_DATA;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
   DMA_InitStructure.DMA_Priority = DMA_Priority_Low;//���Ƚ���
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->TDR; //USART1_RDR_Address;//USART1->RDR;
   DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	
   //DMA_RemapConfig(DMA1, uint32_t DMAx_CHy_RemapRequest)
	
   DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
   DMA_ITConfig(DMA1_Channel2, DMA_IT_TE, ENABLE);
   

   DMA_Cmd(DMA1_Channel2, DISABLE); //�رգ�ʲôʱ����ʲôʱ��
  
   NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


}


void DMA1_Channel2_3_IRQHandler(void)
{
  
//  if((DMA_GetITStatus(DMA1_IT_TC3)!= RESET) || (DMA_GetITStatus(DMA1_IT_TE3)!= RESET))//Ch3�ж�
//  {
//	DMA_ClearITPendingBit(DMA1_IT_TC3);
//	DMA_ClearITPendingBit(DMA1_IT_TE3);
//	DMA_Cmd(DMA1_Channel3, DISABLE);//�ر�DMA,��ֹ�������������
//	DMA1_Channel3->CNDTR = 500;//��װ��
//	DMA_Cmd(DMA1_Channel3, ENABLE);//������,�ؿ�DMA
//  }
//  else 
  if((DMA_GetITStatus(DMA1_IT_TC2)!= RESET) || (DMA_GetITStatus(DMA1_IT_TE2)!= RESET))//Ch2�ж�
  {
	DMA_ClearITPendingBit(DMA1_IT_TC2);
    DMA_ClearITPendingBit(DMA1_IT_TE2);
    DMA_Cmd(DMA1_Channel2, DISABLE);//�ر�DMA
	DMAbusy = 0;//DMA����
  }
 
 
}

//����DMA��������USART1
void Send_Data_DMA(uint8_t * Send_Buf , uint8_t length)
{
	//��Ҫ���͵����ݿ��������ͻ�����
	uint8_t i = 0;
	for(;i < length ; i++)
	{
		USART1_SEND_DATA[i] = Send_Buf[i];
	}
	DMAbusy = 1;//DMAæ
	DMA_Cmd(DMA1_Channel2, DISABLE);//�޸ķ������ݵĳ���
	DMA_SetCurrDataCounter (DMA1_Channel2,  length);
    DMA_Cmd(DMA1_Channel2, ENABLE);	
}

