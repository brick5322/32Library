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

#if AHB_DIV == 1
#define Private_Defines_CFGR_AHB_DIV ((uint32_t)(0x00000000))
#elif AHB_DIV == 2
#define Private_Defines_CFGR_AHB_DIV ((uint32_t)(0x00000080))
#elif AHB_DIV == 4
#define Private_Defines_CFGR_AHB_DIV ((uint32_t)(0x00000090))
#elif AHB_DIV == 8
#define Private_Defines_CFGR_AHB_DIV ((uint32_t)(0x000000A0))
#elif AHB_DIV == 16
#define Private_Defines_CFGR_AHB_DIV ((uint32_t)(0x000000B0))
#elif AHB_DIV == 64
#define Private_Defines_CFGR_AHB_DIV ((uint32_t)(0x000000C0))
#elif AHB_DIV == 128
#define Private_Defines_CFGR_AHB_DIV ((uint32_t)(0x000000D0))
#elif AHB_DIV == 256
#define Private_Defines_CFGR_AHB_DIV ((uint32_t)(0x000000E0))
#elif AHB_DIV == 512
#define Private_Defines_CFGR_AHB_DIV ((uint32_t)(0x000000F0))
#endif

#if APB1_DIV == 1
#define Private_Defines_CFGR_APB1_DIV ((uint32_t)(0x00000000))
#elif APB1_DIV == 2
#define Private_Defines_CFGR_APB1_DIV ((uint32_t)(0x00000400))
#elif APB1_DIV == 4
#define Private_Defines_CFGR_APB1_DIV ((uint32_t)(0x00000500))
#elif APB1_DIV == 8
#define Private_Defines_CFGR_APB1_DIV ((uint32_t)(0x00000600))
#elif APB1_DIV == 16
#define Private_Defines_CFGR_APB1_DIV ((uint32_t)(0x00000700))
#endif

#if APB2_DIV == 1
#define Private_Defines_CFGR_APB2_DIV ((uint32_t)(0x00000000))
#elif APB2_DIV == 2
#define Private_Defines_CFGR_APB2_DIV ((uint32_t)(0x00002000))
#elif APB2_DIV == 4
#define Private_Defines_CFGR_APB2_DIV ((uint32_t)(0x00002800))
#elif APB2_DIV == 8
#define Private_Defines_CFGR_APB2_DIV ((uint32_t)(0x00003000))
#elif APB2_DIV == 16
#define Private_Defines_CFGR_APB2_DIV ((uint32_t)(0x00003800))
#endif

#ifdef STM32F10X_CL
#define Private_Defines_CFGR_PLLMUL ((PLL_MUL == 6.5) ? 0xD << 18 : ((PLL_MUL - 2) << 18))
#define Private_Defines_CFGR2_PLL2MUL ((PLL2_MUL == 20) ? 0xf << 8 : ((PLL2_MUL - 2) << 8))
#define Private_Defines_CFGR2_PLL3MUL ((PLL3_MUL == 20) ? 0xf << 12 : ((PLL3_MUL - 2) << 12))
#define Private_Defines_CFGR2_PRE_DIV1 (PRE_DIV1 - 1)
#define Private_Defines_CFGR2_PRE_DIV2 ((PRE_DIV2 - 1) << 4)

#define Private_Defines_CFGR2_PREDIV1SRC_MASK ((uint32_t)(0x00010000))
#define Private_Defines_CFGR2_PREDIV1SRC_HSE ((uint32_t)(0x00000000))
#define Private_Defines_CFGR2_PREDIV1SRC_PLL2 ((uint32_t)(0x00010000))

#define Private_Defines_CFGR2_PLL2MUL_MASK ((uint32_t)(0x00000F00))
#define Private_Defines_CFGR2_PLL3MUL_MASK ((uint32_t)(0x0000F000))
#define Private_Defines_CFGR2_PRE_DIV1_MASK ((uint32_t)(0x0000000F))
#define Private_Defines_CFGR2_PRE_DIV2_MASK ((uint32_t)(0x000000F0))
#else
#define Private_Defines_CFGR_PLLMUL (((uint32_t)PLL_MUL - 2) << 18)
#endif

#define Private_Defines_CFGR_PLLSRC_MASK ((uint32_t)((uint32_t)(0x00010000))
#ifdef SYSCLK_PLL
#ifdef PLL_HSE
#define Private_Defines_CFGR_PLLSRC ((uint32_t)(0x00010000))
#else
#define Private_Defines_CFGR_PLLSRC ((uint32_t)(0x00000000))
#endif
#endif

#define Private_Defines_HSE_ON ((uint32_t)(0x00010000))
#define Private_Defines_HSE_Ready ((uint32_t)(0x00020000))
#define Private_Defines_CR_PLL2_ON ((uint32_t)(0x04000000))
#define Private_Defines_CR_PLL2_Ready ((uint32_t)(0x08000000))
#define Private_Defines_CR_PLL_ON ((uint32_t)(0x01000000))
#define Private_Defines_CR_PLL_Ready ((uint32_t)(0x02000000))

#define Private_Defines_CFGR_AHB_DIV_MASK ((uint32_t)(0x000000F0))
#define Private_Defines_CFGR_APB1_DIV_MASK ((uint32_t)(0x00000700))
#define Private_Defines_CFGR_APB2_DIV_MASK ((uint32_t)(0x00003800))

#define Private_Defines_CFGR_SW_MASK ((uint32_t)(0x00000003))
#define Private_Defines_CFGR_SW_HSI ((uint32_t)(0x00000000))
#define Private_Defines_CFGR_SW_HSE ((uint32_t)(0x00000001))
#define Private_Defines_CFGR_SW_PLL ((uint32_t)(0x00000002))

#define Private_Defines_CFGR_SWstatus_HSE ((uint32_t)(0x00000004))
#define Private_Defines_CFGR_SWstatus_PLL ((uint32_t)(0x00000008))

#define Private_Defines_CFGR_PLLXTPRE_MASK ((uint32_t)(0x00020000))
#define Private_Defines_CFGR_PLLXTPRE_SET ((uint32_t)(0x00020000))

#define Private_Defines_CFGR_PLLMUL_MASK ((uint32_t)(0x003c0000))

#define Private_Defines_RCCFL_ACR_LATENCY_MASK ((uint32_t)(0x00000007))
#define Private_Defines_RCCFL_ACR_LATENCY_0 ((uint32_t)(0x00000000))
#define Private_Defines_RCCFL_ACR_LATENCY_1 ((uint32_t)(0x00000001))
#define Private_Defines_RCCFL_ACR_LATENCY_2 ((uint32_t)(0x00000002))

#define Private_Defines_RCCFL_ACR_PRFTBE ((uint32_t)(0x00000010))

RCC_Cls::RCC_Cls()
{
  RCC_Reset();
  RCC_SetSysClock();
}

/**
 * @brief RCC_Cls的析构函数 \n
 *        重置时钟时钟设置，HSI作为系统时钟 \n
 *        关闭HSE、PLL、CSS
 **/
RCC_Cls::~RCC_Cls()
{
  RCC_Reset();
}

void RCC_Cls::RCC_Reset()
{
   /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  /* Set HSION bit */
  this->CR |= (uint32_t)0x00000001;

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
#ifndef STM32F10X_CL
  this->CFGR &= (uint32_t)0xF8FF0000;
#else
  this->CFGR &= (uint32_t)0xF0FF0000;
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
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)
  /* Disable all interrupts and clear pending bits  */
  this->CIR = 0x009F0000;

  /* Reset CFGR2 register */
  this->CFGR2 = 0x00000000;      
#else
  /* Disable all interrupts and clear pending bits  */
  this->CIR = 0x009F0000;
#endif /* STM32F10X_CL */
    
#if defined (STM32F10X_HD) || (defined STM32F10X_XL) || (defined STM32F10X_HD_VL)
  #ifdef DATA_IN_ExtSRAM
    SystemInit_ExtMemCtl(); 
  #endif /* DATA_IN_ExtSRAM */
#endif 
}

/**
 * 
 **/
void RCC_Cls::RCC_SetSysClock()
{

  uint32_t &FLASH_ACR = *(uint32_t *)(Addr_AHB + 0x2000);
  this->CR |= HSE_ON;
  for (int i = 0; i < Clock_Init_Delay; i++)
    if (this->CR & Private_Defines_HSE_Ready)
      break;
  if (!(this->CR & Private_Defines_HSE_Ready))
    Err_TimeOut();

/* 配置Flash */
#if !defined(STM32F10X_LD_VL) && !defined(STM32F10X_LD_VL) && !defined(STM32F10X_HD_VL)
  FLASH_ACR |= Private_Defines_RCCFL_ACR_PRFTBE;
  FLASH_ACR &= ~Private_Defines_RCCFL_ACR_LATENCY_MASK;
#if DefaultSystemClock <= 2400000
  FLASH_ACR |= Private_Defines_RCCFL_ACR_LATENCY_0;
#elif DefaultSystemClock <= 4800000
  FLASH_ACR |= Private_Defines_RCCFL_ACR_LATENCY_1;
#else
  FLASH_ACR |= Private_Defines_RCCFL_ACR_LATENCY_2;
#endif
#endif

  /* 配置启动时钟 */
  this->CFGR &= (uint32_t)(~(Private_Defines_CFGR_SW_MASK));
#ifdef SYSCLK_HSI
  while (this->CFGR & (~(Private_Defines_CFGR_SW_HSI));
#endif

#ifdef SYSCLK_HSE
         this->CFGR |= (uint32_t)Private_Defines_CFGR_SW_HSE;
         while (!(this->CFGR & Private_Defines_CFGR_SWstatus_HSE));
#endif

#ifdef SYSCLK_PLL
         /* 配置分频和倍频 */
         this->CFGR &= (uint32_t) ~(Private_Defines_CFGR_PLLXTPRE_MASK | Private_Defines_CFGR_PLLSRC_MASK | Private_Defines_CFGR_PLLMUL_MASK | Private_Defines_CFGR_AHB_DIV_MASK | Private_Defines_CFGR_APB1_DIV_MASK | Private_Defines_CFGR_APB2_DIV_MASK));
#ifdef PRE_DIV2_ON
  this->CFGR |= (uint32_t)Private_Defines_CFGR_PLLXTPRE_SET;
#ifdef STMF10X_CL
  this->CFGR2 |= (Private_Defines_CFGR2_PREDIV1SRC_PLL2);
#endif
#endif

#ifdef STM32F10X_CL
  (uint16_t)(this->CFGR2) = 0;
  this->CFGR2 |= Private_Defines_CFGR2_PLL2MUL |
                 Private_Defines_CFGR2_PLL3MUL |
                 Private_Defines_CFGR2_PRE_DIV1 |
                 Private_Defines_CFGR2_PRE_DIV2;
#endif
  this->CFGR |= (Private_Defines_CFGR_PLLSRC | Private_Defines_CFGR_PLLMUL | Private_Defines_CFGR_APB1_DIV | Private_Defines_CFGR_APB2_DIV | Private_Defines_CFGR_AHB_DIV);

#ifdef STM32F10X_CL
  this->CR |= Private_Defines_CR_PLL2_ON;
  while (!(this->CR & Private_Defines_CR_PLL2_Ready))
    ;
#endif
  this->CR |= Private_Defines_CR_PLL_ON;
	
  for (int i = 0; i < Clock_Init_Delay; i++)
    if (this->CR & Private_Defines_CR_PLL_Ready)
      break;
	if (!(this->CR & Private_Defines_CR_PLL_Ready))
    Err_TimeOut();
  this->CFGR &= Private_Defines_CFGR_SW_MASK;
  this->CFGR |= (uint32_t)Private_Defines_CFGR_SW_PLL;
		
  for (int i = 0; i < Clock_Init_Delay; i++)
    if (this->CFGR & Private_Defines_CFGR_SW_PLL)
      break;
	if (!(this->CFGR & Private_Defines_CFGR_SW_PLL))
    Err_TimeOut();

#endif
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
