#include "M500.h"
#include "DMA.h"

//uint8_t cmd_get_work_antenna[5] = {0xA0 , 0x03 , 0xFF , 0x75 , 0xE9};//���һλΪcheck,��Ҫ����
uint8_t cmd_set_uart_baudrate[] = { 0xA0 , 0x04 , 0xFF , 0x71 , 0x04 , 0xE8 };//����ģ��ͨ��Ƶ��Ϊ115200
uint8_t cmd_set_output_power[] = {0xA0 , 0x04 , 0xFF , 0x76 , 0x00 , 0x00};//���ʡ�У��δ�趨
uint8_t cmd_read_EPC[] = {0xA0 , 0x0A , 0xFF , 0x81 , 0x01 , 0x00 , 0x07 , 0x00 , 0x00 , 0x00 ,0x00 , 0xCE};//������ǩEPC

//����ǩ��ȡEPC
void Signle_Read_EPC(void)
{
	Send_Data_DMA(cmd_read_EPC , 12);
}

//�趨��д������
//[in] RfPower 12 - 19
void Set_Output_Power(uint8_t RfPower)
{
	uint8_t check = 0;
	if(RfPower < 12 || RfPower > 19)
	{
		return ;
	}
	cmd_set_output_power[4] = RfPower;
	
	check = CheckSum(cmd_set_output_power, 5);//У��
	
	cmd_set_output_power[5] = check;
	Send_Data_DMA(cmd_set_output_power , 6);
}


//M500ģ��У�鷽��
unsigned char CheckSum(unsigned char *uBuff, unsigned char uBuffLen)
{
	unsigned char i,uSum=0;
	for(i=0;i<uBuffLen;i++)
	{
	uSum = uSum + uBuff[i];
	}
	uSum = (~uSum) + 1;
	return uSum;
}


