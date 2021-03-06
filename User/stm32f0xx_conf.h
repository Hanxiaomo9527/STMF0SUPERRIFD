/***************************************************************************//**
  * @file    stm32f0xx_conf.h 
  * @author  吴国炎
  * @version V1.0.0
  * @date    2011年5月25号 
  * @brief   STM32 的库函数配置文件
  ******************************************************************************
  @verbatim   
  库函数配置文件，需要使用相应的库函数时，去掉相关注释，打开相应的头文件
  并且在工程中加入相应的驱动库C源文件即可。
                
  吴国炎制作
  技术博客网址：http://hi.baidu.com/wuguoyana/home
  @endverbatim
  * @attention
  * 
  * 郑重声明： 
  * 此文件只用于提供开发参考，如果因此文件而产生的任何问题纠纷，我概不负责。
  *
  * <h2><center> &copy; COPYRIGHT 2011 wuguoyana </center></h2>
*******************************************************************************/

/* 防止重定义 ----------------------------------------------------------------*/
#ifndef __STM32F0XX_CONF_H
#define __STM32F0XX_CONF_H

/* 包含头文件 *****************************************************************/
//当需要使用以下功能时，去掉相关注释，并且在工程中加入相应的C源文件
//#include "stm32f0xx_adc.h"
//#include "stm32f0xx_cec.h"
//#include "stm32f0xx_crc.h"
//#include "stm32f0xx_comp.h"
//#include "stm32f0xx_dac.h"
//#include "stm32f0xx_dbgmcu.h"
//#include "stm32f0xx_dma.h"
//#include "stm32f0xx_exti.h"
//#include "stm32f0xx_flash.h"
#include "stm32f0xx_gpio.h"
//#include "stm32f0xx_syscfg.h"
//#include "stm32f0xx_i2c.h"
//#include "stm32f0xx_iwdg.h"
//#include "stm32f0xx_pwr.h"
#include "stm32f0xx_rcc.h"
//#include "stm32f0xx_rtc.h"
//#include "stm32f0xx_spi.h"
//#include "stm32f0xx_tim.h"
//#include "stm32f0xx_usart.h"
//#include "stm32f0xx_wwdg.h"
//#include "stm32f0xx_misc.h"  /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */


/* 取消对下面一行展开assert_param宏用于标准外设库的驱动程序代码*/
/*#define USE_FULL_ASSERT    1 */

/* 导出的宏定义 --------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/***************************************************************************//**
  * @brief   assert_param 用于参数检查.
  * @param   expr: 假如expr是flase, 将会调用 assert_param
  *          报告错误发生所在的源文件名和所在的行数
  *          假如expr 是 true, 将步返回值.
  * @retval  无
*******************************************************************************/
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* 函数定义------------------------------------------------------------------ */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F0XX_CONF_H */

/******************* (C) COPYRIGHT wuguoyana ***************文件结束***********/
