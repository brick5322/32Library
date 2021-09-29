/**
  ******************************************************************************
  * @file    stm32f10x_RCC.cpp
  * @author  LIUVBC
  * @version V1.0
  * @date    13-Sep-2021
  * @brief   RCC外设固件库源文件
  ******************************************************************************
**/

#include "stm32f10x_RCC.h"

RCC_Cls::RCC_Cls()
{
}

/**
 * @brief RCC_Cls的析构函数 \n
 *        重置始终设置，HSI作为系统时钟 \n
 *        关闭HSE、PLL、CSS
 **/
RCC_Cls::~RCC_Cls()
{
  /* Set HSION bit */
  this->CR |= (uint32_t)0x00000001;

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
#ifndef STM32F10X_CL
  this->CFGR &= (uint32_t)0x087F0000;
#else
  this->CFGR &= (uint32_t)0x007F0000;
#endif /* STM32F10X_CL */

  /* Reset HSEON, CSSON and PLLON bits */
  this->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset HSEBYP bit */
  this->CR &= (uint32_t)0xFFFBFFFF;

  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  this->CFGR &= (uint32_t)0xFF80FFFF;

#ifdef STM32F10X_CL
  /* Reset PLL2ON and PLL3ON bits */
  this->CR &= (uint32_t)0xEBFFFFFF;

  /* Disable all interrupts and clear pending bits  */
  this->CIR = 0x00FF0000;

  /* Reset CFGR2 register */
  this->CFGR2 = 0x00000000;
#elif defined(STM32F10X_LD_VL) || defined(STM32F10X_MD_VL) || defined(STM32F10X_HD_VL)
  /* Disable all interrupts and clear pending bits  */
  this->CIR = 0x009F0000;

  /* Reset CFGR2 register */
  this->CFGR2 = 0x00000000;
#else
  /* Disable all interrupts and clear pending bits  */
  this->CIR = 0x009F0000;
#endif /* STM32F10X_CL */
}

/**
 * @brief 设置外部高速时钟
 * @param HSE_Config 外设时钟在使能寄存器上的偏移量
 * \n 见 @ref RCC外设相关宏定义
 **/
void RCC_Cls::Set_HSE(uint32_t HSE_Config)
{
  this->CR &= 0xFFFEFFFF;
  this->CR &= 0xFFFBFFFF;
  /* this->CR &= 0xFFFAFFFF; */
  this->CR |= HSE_Config;
}

/**
 * @brief 启用APB2总线上外设时钟
 * @param APB2_PERIPH 外设时钟在使能寄存器上的偏移量
 * \n 见 @ref APB2总线相关宏定义
 **/
void RCC_Cls::APB2_SetEnable(uint32_t APB2_PERIPH)
{
  this->APB2ENR |= APB2_PERIPH;
}

/**
 * @brief 禁用APB2总线上外设时钟
 * @param APB2_PERIPH 外设时钟在使能寄存器上的偏移量
 * \n 见 @ref APB2总线相关宏定义
 **/
void RCC_Cls::APB2_SetDisable(uint32_t APB2_PERIPH)
{
  this->APB2ENR &= ~APB2_PERIPH;
}
