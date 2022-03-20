#ifndef __ToolsFun_H
#define __ToolsFun_H

#include "stdint.h"
#include "stm32f0xx.h"
#include "DMA.h"

#ifndef BIT
#define BIT(x)	(1 << (x)) //��ָ��λ���ָ������X
#endif

#define LED_GPIO_CLK   RCC_AHBPeriph_GPIOA 
#define LED_PORT   	   GPIOA
#define LED_PIN        GPIO_Pin_4

#define Card_Num   4

#define Active_States    1
#define Inactive_States  0

#define RUNSTATEREAD 0 //��ȡ��ǩ״̬
#define RUNSTATESETTING 1 //����״̬
#define RUNSTATEGETRF 2 //��ȡ��д������״̬
#define RUNSTATEWAIT 3 //������״̬����Ϊ��ȡ״̬ʱ���ȴ���ն�д���ڴ��еı�ǩ״̬

//��ʱ
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
void Delay(uint32_t count);
void Delay_Init(uint8_t SYSCLK);

void Systick_Init(void);
void TimingDelay_Decrement(void);
void Delay_ms(__IO uint32_t nTime);

//����
void Alert_Init(void);
void GetTags(void);//��ȡ��ǩ����
void GetReadRF(void);
void SetRFPower(uint8_t Power);
void ClearReadTag(void);
void Process_Data(uint8_t * Data , uint8_t length);//����USART1�ϴ�������
void EXTI13_14_Config(void);//������д�����ʴ�С���ⲿ�ж�
unsigned short Crc16_DATAs(unsigned char *ptr, unsigned short len);


//����LED��
void LED_Init(void);
void LED_ON(void);
void LED_OFF(void);

//���濨ID�ṹ����غ���
void Card_Init(void);

extern uint16_t fac_ms;//ȫ�ֱ���
extern uint8_t fac_us;//ȫ�ֱ���
extern uint16_t warning;//Ԥ��
extern uint16_t alert;//����


extern uint8_t RFPower;//��д������˥����0-31
extern uint8_t RFPowerMin;
extern uint8_t RFPowerMax;					   
extern uint8_t RFPowerStep;//����	
extern uint8_t RFPowerSettingButton;
extern uint8_t runstate;

typedef struct Card
{
  uint8_t ID[4];
  uint8_t States;
  struct Card * Next;
}Card_t;



#endif

