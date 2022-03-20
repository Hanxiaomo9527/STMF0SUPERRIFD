#ifndef __ToolsFun_H
#define __ToolsFun_H

#include "stdint.h"
#include "stm32f0xx.h"
#include "DMA.h"

#ifndef BIT
#define BIT(x)	(1 << (x)) //向指定位填充指定数字X
#endif

#define LED_GPIO_CLK   RCC_AHBPeriph_GPIOA 
#define LED_PORT   	   GPIOA
#define LED_PIN        GPIO_Pin_4

#define Card_Num   4

#define Active_States    1
#define Inactive_States  0

#define RUNSTATEREAD 0 //读取标签状态
#define RUNSTATESETTING 1 //设置状态
#define RUNSTATEGETRF 2 //获取读写器功率状态
#define RUNSTATEWAIT 3 //由其它状态更改为读取状态时，等待清空读写器内存中的标签状态

//延时
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
void Delay(uint32_t count);
void Delay_Init(uint8_t SYSCLK);

void Systick_Init(void);
void TimingDelay_Decrement(void);
void Delay_ms(__IO uint32_t nTime);

//报警
void Alert_Init(void);
void GetTags(void);//获取标签数据
void GetReadRF(void);
void SetRFPower(uint8_t Power);
void ClearReadTag(void);
void Process_Data(uint8_t * Data , uint8_t length);//处理USART1上传的数据
void EXTI13_14_Config(void);//调整读写器功率大小的外部中断
unsigned short Crc16_DATAs(unsigned char *ptr, unsigned short len);


//板载LED灯
void LED_Init(void);
void LED_ON(void);
void LED_OFF(void);

//储存卡ID结构体相关函数
void Card_Init(void);

extern uint16_t fac_ms;//全局变量
extern uint8_t fac_us;//全局变量
extern uint16_t warning;//预警
extern uint16_t alert;//报警


extern uint8_t RFPower;//读写器功率衰减，0-31
extern uint8_t RFPowerMin;
extern uint8_t RFPowerMax;					   
extern uint8_t RFPowerStep;//步进	
extern uint8_t RFPowerSettingButton;
extern uint8_t runstate;

typedef struct Card
{
  uint8_t ID[4];
  uint8_t States;
  struct Card * Next;
}Card_t;



#endif

