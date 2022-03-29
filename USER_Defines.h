/**
  ******************************************************************************
  * @file    USER_Defines.h
  * @author  LIUVBC
  * @version V1.0
  * @date    30-Sep-2021
  * @brief   需要用户自定义的宏和函数
  ******************************************************************************
**/

#pragma once
#ifndef _USER_DEfines_
#define _USER_DEfines_

#define UnDefine (0)

/**
 * @defgroup 用户自定义宏
 * @{
 **/

/**
 * @defgroup 芯片类型
 * @{
 **/
//#define STM32F10X_LD
//#define STM32F10X_MD
//#define STM32F10X_HD
//#define STM32F10X_CL
//#define STM32F10X_LD_VL
//#define STM32F10X_MD_VL
//#define STM32F10X_HD_VL
/**
 * @}
 **/

/**
 * @defgroup 时钟倍频分频配置
 * @{
 **/

/**
 * @defgroup SYSCLK时钟选择
 * @param SYSCLK_HSE  使用HSE作为SYSCLK
 * @param SYSCLK_HSI  使用HSI作为SYSCLK
 * @param SYSCLK_PLL  使用PLL作为SYSCLK
 * @param PRE_DIV2_ON 开启两次分频（非互联型仅2分频）
 * @{
 **/

/* #define SYSCLK_HSE */
/* #define SYSCLK_HSI */
#define SYSCLK_PLL
/* #define PRE_DIV2_ON */

/**
 * @}
 **/

/**
 * @defgroup 外频速率
 * @{
 **/
#define HSE_Value (8000000) /*!<  单位：Hz*/
/**
 * @}
 **/

/**
 *  @defgroup PLL输入选择
 * @param PLL_HSE HSE输入
 * @param PLL_HSI HSI二分频输入
 * @{
 **/
/* #define PLL_HSI */
#define PLL_HSE
/**
 *  @}
 **/

/**
 * @defgroup PLL倍频配置
 * @{
 **/

#define PLL_MUL (9)         /*!<  PLL  x倍频输出，x取整数2~16(互联型取4~9，含6.5)       */
#define PLL2_MUL (UnDefine) /*!<  PLL2 x倍频输出，x取整数8~14,16,20                    */
#define PLL3_MUL (UnDefine) /*!<  PLL3 x倍频输出，x取整数8~14,16,20                    */

/**
 * @}
 **/

/**
 * @defgroup 分频配置
 * @{
 **/

#define AHB_DIV (1)        /*!< AHB分频系数，1~512分频（仅2^n）   */
#define APB1_DIV (2)       /*!< APB1分频系数，1~16分频（仅2^n）  */
#define APB2_DIV (1)       /*!< APB2分频系数，1~16分频（仅2^n） */
#define PRE_DIV1 (UnDefine) /*!< PREDIV1分频系数，1~16分频（整数）*/
#ifdef STM32F10X_CL
#define PRE_DIV2 (UnDefine) /*!< PREDIV2分频系数，1~16分频（整数）*/
#else
#define PRE_DIV2 (2)
#endif
/**
 * @}
 **/

/**
 * @defgroup 时钟频率
 * @{
 **/
#ifdef SYSCLK_HSE
#define DefaultSystemClock HSE_Value

#elif defined(SYSCLK_HSI)
#define DefaultSystemClock (8000000)

#elif defined(PLL_HSE)


#if !defined(STMF10X_CL) && !defined(PRE_DIV2_ON)
#define DefaultSystemClock (HSE_Value * PLL_MUL) 
#elif !defined(STMF10X_CL) && defined(PRE_DIV2_ON)
#define DefaultSystemClock (HSE_Value * PLL_MUL / PRE_DIV2)
#elif defined(STMF10X_CL) && !defined(PRE_DIV2_ON)
#define DefaultSystemClock (HSE_Value / PRE_DIV1 * PLL_MUL)  
#else
#define DefaultSystemClock (HSE_Value / PRE_DIV2 * PLL2_MUL / PRE_DIV1 * PLL_MUL) 
#endif

#elif defined(PLL_HSI)

#ifdef STMF10X_CL
#define DefaultSystemClock (4000000 * PLL_MUL)
#else
#define DefaultSystemClock (4000000 * PLL_MUL)
#endif

#endif


/**
 * @}
 **/

/**
 * @defgroup 时钟启动等待时间
 * @details 默认0x500 指令时间
 * @{
 **/

#define Clock_Init_Delay (0x500)

/**
 * @}
 **/

/**
 * @}
 **/
 
#endif /*_USER_DEfines_*/

void Err_TimeOut();

//void SystemInit();
