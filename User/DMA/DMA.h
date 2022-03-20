#ifndef __DMA_H
#define __DMA_H

#include "stdint.h"
#include "stm32f0xx.h"

extern uint8_t USART1_RECEIVE_DATA[];
extern uint8_t DMAbusy;
void DMA_Config(void);
//利用DMA发送数据USART1
void Send_Data_DMA(uint8_t * Send_Buf , uint8_t length);

#endif
