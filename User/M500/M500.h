#ifndef __M500_H
#define __M500_H

#include "stdint.h"
#include "stm32f0xx.h"

unsigned char CheckSum(unsigned char *uBuff, unsigned char uBuffLen);
void Set_Output_Power(uint8_t RfPower);
void Signle_Read_EPC(void);

#endif
