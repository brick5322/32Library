/**
  ******************************************************************************
  * @file    stm32f10x_RCC.h
  * @author  LIUVBC
  * @version V1.0
  * @date    13-Sep-2021
  * @brief   RCC外设固件库头文件
  ******************************************************************************
**/

#pragma once

#include "stm32f10x.h"
#include "astm32f10x.h"
#include "USER_Defines"
/**
 * @defgroup AHB总线地址宏定义
 * @{
 **/
#define RCC_ptr ((RCC_Cls *)(Addr_AHB + 0x1000))

/**
 * @}
 * */

/**
 * @addtogroup 常量实参枚举和宏定义
 * @{
 **/

/**
 * @defgroup APB2总线相关宏定义
 * @{
 **/

#define APB2_AFIO (0x1)      /*!< AFIO时钟在使能和复位寄存器上的偏移量  */
#define APB2_EXTI (0x4)      /*!< EXTI时钟在使能和复位寄存器上的偏移量  */
#define APB2_GPIOA (0x8)     /*!< GPIOA时钟在使能和复位寄存器上的偏移量 */
#define APB2_GPIOB (0xC)     /*!< GPIOB时钟在使能和复位寄存器上的偏移量 */
#define APB2_GPIOC (0x10)    /*!< GPIOC时钟在使能和复位寄存器上的偏移量 */
#define APB2_GPIOD (0x20)    /*!< GPIOD时钟在使能和复位寄存器上的偏移量 */
#define APB2_GPIOE (0x40)    /*!< GPIOE时钟在使能和复位寄存器上的偏移量 */
#define APB2_GPIOF (0x80)    /*!< GPIOF时钟在使能和复位寄存器上的偏移量 */
#define APB2_GPIOG (0x100)   /*!< GPIOG时钟在使能和复位寄存器上的偏移量 */
#define APB2_ADC1 (0x200)    /*!< ADC1时钟在使能和复位寄存器上的偏移量  */
#define APB2_ADC2 (0x400)    /*!< ADC2时钟在使能和复位寄存器上的偏移量  */
#define APB2_TIM1 (0x800)    /*!< TIM1时钟在使能和复位寄存器上的偏移量  */
#define APB2_SPI1 (0x1000)   /*!< SPI1时钟在使能和复位寄存器上的偏移量  */
#define APB2_TIM8 (0x2000)   /*!< TIM8时钟在使能和复位寄存器上的偏移量  */
#define APB2_USART1 (0x4000) /*!< USART1时钟在使能和复位寄存器上的偏移量*/
#define APB2_ADC3 (0x8000)   /*!< ADC3时钟在使能和复位寄存器上的偏移量  */

/**
 * @}
 **/

/**
 * @defgroup RCC外设相关宏定义
 * @{
 **/

#define HSE_OFF ((uint32_t)0x0000)        /*!<  关闭HSE  */
#define HSE_ON  ((uint32_t)0x00010000)    /*!<  打开HSE  */
#define HSE_ByPass ((uint32_t)0x00050000) /*!<  旁路HSE  */

/**
 * @}
 **/

/**
 * @}
 **/
static uint32_t SystemClock;
/**
 * @brief RCC时钟类
 **/
class RCC_Cls : public Perieh_Cls
{
private:
  uint32_t CR;
  uint32_t CFGR;
  uint32_t CIR;
  uint32_t APB2RSTR;
  uint32_t APB1RSTR;
  uint32_t AHBENR;
  uint32_t APB2ENR;
  uint32_t APB1ENR;
  uint32_t BDCR;
  uint32_t CSR;
#ifdef STM32F10X_CL  
  __IO uint32_t AHBRSTR;
  __IO uint32_t CFGR2;
#endif 

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)   
  uint32_t RESERVED0;
  __IO uint32_t CFGR2;
#endif
  void RCC_Reset();
  void RCC_SetSysClock();
public:
  RCC_Cls();
  ~RCC_Cls();
  void APB2_SetEnable(uint32_t APB2_PERIPH);
  void APB2_SetDisable(uint32_t APB2_PERIPH);
  void Set_HSE(uint32_t HSE_Config);
};
