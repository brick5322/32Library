/**
  ******************************************************************************
  * @file    stm32f10x.h
  * @author  LIUVBC
  * @version V1.0
  * @date    13-Sep-2021
  * @brief  stm32f10x固件库头文件
  ******************************************************************************
**/

/**
 * @defgroup 常量实参枚举和宏定义
 * @{
 **/

/**
 * @defgroup 常量参数优化宏定义
 * @{
 **/

/**
 * @}
 **/

/**
 * @}
 * */

#pragma once

#define Dynatic_VecTable

#define Success (false)
#define Failed (true)

typedef unsigned int size_t;
typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

#ifdef _DEBUG

#define assert_param(arg) ((arg) ? (false)) \
                           : assert_failed((uint8_t *)__FILE__, __LINE__))

void assert_failed(uint8_t *file, uint32_t line);

#endif

/**
 * @defgroup 地址宏定义
 * @{
 **/
#define PERIEH_BASE (0X40000000)

/**
 * @defgroup APB1总线地址宏定义
 * @{
 **/
#define Addr_APB1 (PERIEH_BASE)

/**
 * @}
 * */

/**
 * @defgroup APB2总线地址宏定义
 * @{
 **/
#define Addr_APB2 ((PERIEH_BASE + 0X10000))

#define AFIO_ptr ((AFIO_Cls *)Addr_APB2)

#define GPIOA_ptr ((GPIO_Cls *)(Addr_APB2 + 0x0800))
#define GPIOB_ptr ((GPIO_Cls *)(Addr_APB2 + 0x0C00))
#define GPIOC_ptr ((GPIO_Cls *)(Addr_APB2 + 0x1000))
#define GPIOD_ptr ((GPIO_Cls *)(Addr_APB2 + 0x1400))
#define GPIOE_ptr ((GPIO_Cls *)(Addr_APB2 + 0x1800))
#define GPIOF_ptr ((GPIO_Cls *)(Addr_APB2 + 0x1C00))
#define GPIOG_ptr ((GPIO_Cls *)(Addr_APB2 + 0x2000))

#define EXTI_ptr ((EXTI_Cls)*Addt_APB2 + 400)

#define ADC1_ptr ((ADC_Cls *)(Addr_APB2 + 0x2400))
#define ADC2_ptr ((ADC_Cls *)(Addr_APB2 + 0x2800))
#define ADC3_ptr ((ADC_Cls *)(Addr_APB2 + 0x3C00))

#define TIM1_ptr ((TIM_Cls *)(Addr_APB2 + 0x2C00))
#define TIM8_ptr ((TIM_Cls *)(Addr_APB2 + 0x3400))
#define TIM9_ptr ((TIM_Cls *)(Addr_APB2 + 0x4C00))
#define TIM10_ptr ((TIM_Cls *)(Addr_APB2 + 0x5000))
#define TIM11_ptr ((TIM_Cls *)(Addr_APB2 + 0x5400))

#define SPI1_ptr ((SPI_Cls *)(Addr_APB2 + 0x3000))

#define USART1_ptr ((USART_Cls *)(Addr_APB2) + 0x3800)

/**
 * @}
 * */
 

 
/**
 * @defgroup AHB总线地址宏定义
 * @{
 **/
#define Addr_AHB ((PERIEH_BASE + 0x20000))

#define CRC_ptr ((CRC_Cls *)Addr_AHB + 0x3000)
/**
 * @}
 * */

/**
 * @}
 * */

/**
 * @addtogroup 常量实参枚举和宏定义
 * @{
 **/

/**
 * @addtogroup stm32f10x中断号枚举
 * @{
 **/

/**
 * @brief 中断号枚举 \n
 * 需要设备宏定义
 **/
typedef enum IRQn
{
  /******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn = -14,   /*!< 2 Non Maskable Interrupt                             */
  MemoryManagement_IRQn = -12, /*!< 4 Cortex-M3 Memory Management Interrupt              */
  BusFault_IRQn = -11,         /*!< 5 Cortex-M3 Bus Fault Interrupt                      */
  UsageFault_IRQn = -10,       /*!< 6 Cortex-M3 Usage Fault Interrupt                    */
  SVCall_IRQn = -5,            /*!< 11 Cortex-M3 SV Call Interrupt                       */
  DebugMonitor_IRQn = -4,      /*!< 12 Cortex-M3 Debug Monitor Interrupt                 */
  PendSV_IRQn = -2,            /*!< 14 Cortex-M3 Pend SV Interrupt                       */
  SysTick_IRQn = -1,           /*!< 15 Cortex-M3 System Tick Interrupt                   */

  /******  STM32 specific Interrupt Numbers *********************************************************/
  WWDG_IRQn = 0,           /*!< Window WatchDog Interrupt                            */
  PVD_IRQn = 1,            /*!< PVD through EXTI Line detection Interrupt            */
  TAMPER_IRQn = 2,         /*!< Tamper Interrupt                                     */
  RTC_IRQn = 3,            /*!< RTC global Interrupt                                 */
  FLASH_IRQn = 4,          /*!< FLASH global Interrupt                               */
  RCC_IRQn = 5,            /*!< RCC global Interrupt                                 */
  EXTI0_IRQn = 6,          /*!< EXTI Line0 Interrupt                                 */
  EXTI1_IRQn = 7,          /*!< EXTI Line1 Interrupt                                 */
  EXTI2_IRQn = 8,          /*!< EXTI Line2 Interrupt                                 */
  EXTI3_IRQn = 9,          /*!< EXTI Line3 Interrupt                                 */
  EXTI4_IRQn = 10,         /*!< EXTI Line4 Interrupt                                 */
  DMA1_Channel1_IRQn = 11, /*!< DMA1 Channel 1 global Interrupt                      */
  DMA1_Channel2_IRQn = 12, /*!< DMA1 Channel 2 global Interrupt                      */
  DMA1_Channel3_IRQn = 13, /*!< DMA1 Channel 3 global Interrupt                      */
  DMA1_Channel4_IRQn = 14, /*!< DMA1 Channel 4 global Interrupt                      */
  DMA1_Channel5_IRQn = 15, /*!< DMA1 Channel 5 global Interrupt                      */
  DMA1_Channel6_IRQn = 16, /*!< DMA1 Channel 6 global Interrupt                      */
  DMA1_Channel7_IRQn = 17, /*!< DMA1 Channel 7 global Interrupt                      */

#ifdef STM32F10X_LD
  ADC1_2_IRQn = 18,          /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn = 19,  /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn = 20, /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn = 21,        /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn = 22,        /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn = 23,         /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn = 24,        /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn = 25,         /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn = 26,    /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn = 27,         /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn = 28,            /*!< TIM2 global Interrupt                                */
  TIM3_IRQn = 29,            /*!< TIM3 global Interrupt                                */
  I2C1_EV_IRQn = 31,         /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn = 32,         /*!< I2C1 Error Interrupt                                 */
  SPI1_IRQn = 35,            /*!< SPI1 global Interrupt                                */
  USART1_IRQn = 37,          /*!< USART1 global Interrupt                              */
  USART2_IRQn = 38,          /*!< USART2 global Interrupt                              */
  EXTI15_10_IRQn = 40,       /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn = 41,        /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn = 42        /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
#endif                       /* STM32F10X_LD */

#ifdef STM32F10X_LD_VL
  ADC1_IRQn = 18,               /*!< ADC1 global Interrupt                                */
  EXTI9_5_IRQn = 23,            /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM15_IRQn = 24,     /*!< TIM1 Break and TIM15 Interrupts                      */
  TIM1_UP_TIM16_IRQn = 25,      /*!< TIM1 Update and TIM16 Interrupts                     */
  TIM1_TRG_COM_TIM17_IRQn = 26, /*!< TIM1 Trigger and Commutation and TIM17 Interrupt     */
  TIM1_CC_IRQn = 27,            /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn = 28,               /*!< TIM2 global Interrupt                                */
  TIM3_IRQn = 29,               /*!< TIM3 global Interrupt                                */
  I2C1_EV_IRQn = 31,            /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn = 32,            /*!< I2C1 Error Interrupt                                 */
  SPI1_IRQn = 35,               /*!< SPI1 global Interrupt                                */
  USART1_IRQn = 37,             /*!< USART1 global Interrupt                              */
  USART2_IRQn = 38,             /*!< USART2 global Interrupt                              */
  EXTI15_10_IRQn = 40,          /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn = 41,           /*!< RTC Alarm through EXTI Line Interrupt                */
  CEC_IRQn = 42,                /*!< HDMI-CEC Interrupt                                   */
  TIM6_DAC_IRQn = 54,           /*!< TIM6 and DAC underrun Interrupt                      */
  TIM7_IRQn = 55                /*!< TIM7 Interrupt                                       */
#endif                          /* STM32F10X_LD_VL */

#ifdef STM32F10X_MD
  ADC1_2_IRQn = 18,          /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn = 19,  /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn = 20, /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn = 21,        /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn = 22,        /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn = 23,         /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn = 24,        /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn = 25,         /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn = 26,    /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn = 27,         /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn = 28,            /*!< TIM2 global Interrupt                                */
  TIM3_IRQn = 29,            /*!< TIM3 global Interrupt                                */
  TIM4_IRQn = 30,            /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn = 31,         /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn = 32,         /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn = 33,         /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn = 34,         /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn = 35,            /*!< SPI1 global Interrupt                                */
  SPI2_IRQn = 36,            /*!< SPI2 global Interrupt                                */
  USART1_IRQn = 37,          /*!< USART1 global Interrupt                              */
  USART2_IRQn = 38,          /*!< USART2 global Interrupt                              */
  USART3_IRQn = 39,          /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn = 40,       /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn = 41,        /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn = 42        /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
#endif                       /* STM32F10X_MD */

#ifdef STM32F10X_MD_VL
  ADC1_IRQn = 18,               /*!< ADC1 global Interrupt                                */
  EXTI9_5_IRQn = 23,            /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM15_IRQn = 24,     /*!< TIM1 Break and TIM15 Interrupts                      */
  TIM1_UP_TIM16_IRQn = 25,      /*!< TIM1 Update and TIM16 Interrupts                     */
  TIM1_TRG_COM_TIM17_IRQn = 26, /*!< TIM1 Trigger and Commutation and TIM17 Interrupt     */
  TIM1_CC_IRQn = 27,            /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn = 28,               /*!< TIM2 global Interrupt                                */
  TIM3_IRQn = 29,               /*!< TIM3 global Interrupt                                */
  TIM4_IRQn = 30,               /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn = 31,            /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn = 32,            /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn = 33,            /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn = 34,            /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn = 35,               /*!< SPI1 global Interrupt                                */
  SPI2_IRQn = 36,               /*!< SPI2 global Interrupt                                */
  USART1_IRQn = 37,             /*!< USART1 global Interrupt                              */
  USART2_IRQn = 38,             /*!< USART2 global Interrupt                              */
  USART3_IRQn = 39,             /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn = 40,          /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn = 41,           /*!< RTC Alarm through EXTI Line Interrupt                */
  CEC_IRQn = 42,                /*!< HDMI-CEC Interrupt                                   */
  TIM6_DAC_IRQn = 54,           /*!< TIM6 and DAC underrun Interrupt                      */
  TIM7_IRQn = 55                /*!< TIM7 Interrupt                                       */
#endif                          /* STM32F10X_MD_VL */

#ifdef STM32F10X_HD
  ADC1_2_IRQn = 18,          /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn = 19,  /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn = 20, /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn = 21,        /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn = 22,        /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn = 23,         /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn = 24,        /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn = 25,         /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn = 26,    /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn = 27,         /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn = 28,            /*!< TIM2 global Interrupt                                */
  TIM3_IRQn = 29,            /*!< TIM3 global Interrupt                                */
  TIM4_IRQn = 30,            /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn = 31,         /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn = 32,         /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn = 33,         /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn = 34,         /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn = 35,            /*!< SPI1 global Interrupt                                */
  SPI2_IRQn = 36,            /*!< SPI2 global Interrupt                                */
  USART1_IRQn = 37,          /*!< USART1 global Interrupt                              */
  USART2_IRQn = 38,          /*!< USART2 global Interrupt                              */
  USART3_IRQn = 39,          /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn = 40,       /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn = 41,        /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn = 42,       /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
  TIM8_BRK_IRQn = 43,        /*!< TIM8 Break Interrupt                                 */
  TIM8_UP_IRQn = 44,         /*!< TIM8 Update Interrupt                                */
  TIM8_TRG_COM_IRQn = 45,    /*!< TIM8 Trigger and Commutation Interrupt               */
  TIM8_CC_IRQn = 46,         /*!< TIM8 Capture Compare Interrupt                       */
  ADC3_IRQn = 47,            /*!< ADC3 global Interrupt                                */
  FSMC_IRQn = 48,            /*!< FSMC global Interrupt                                */
  SDIO_IRQn = 49,            /*!< SDIO global Interrupt                                */
  TIM5_IRQn = 50,            /*!< TIM5 global Interrupt                                */
  SPI3_IRQn = 51,            /*!< SPI3 global Interrupt                                */
  UART4_IRQn = 52,           /*!< UART4 global Interrupt                               */
  UART5_IRQn = 53,           /*!< UART5 global Interrupt                               */
  TIM6_IRQn = 54,            /*!< TIM6 global Interrupt                                */
  TIM7_IRQn = 55,            /*!< TIM7 global Interrupt                                */
  DMA2_Channel1_IRQn = 56,   /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn = 57,   /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn = 58,   /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_5_IRQn = 59  /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
#endif                       /* STM32F10X_HD */

#ifdef STM32F10X_HD_VL
  ADC1_IRQn = 18,               /*!< ADC1 global Interrupt                                */
  EXTI9_5_IRQn = 23,            /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM15_IRQn = 24,     /*!< TIM1 Break and TIM15 Interrupts                      */
  TIM1_UP_TIM16_IRQn = 25,      /*!< TIM1 Update and TIM16 Interrupts                     */
  TIM1_TRG_COM_TIM17_IRQn = 26, /*!< TIM1 Trigger and Commutation and TIM17 Interrupt     */
  TIM1_CC_IRQn = 27,            /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn = 28,               /*!< TIM2 global Interrupt                                */
  TIM3_IRQn = 29,               /*!< TIM3 global Interrupt                                */
  TIM4_IRQn = 30,               /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn = 31,            /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn = 32,            /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn = 33,            /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn = 34,            /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn = 35,               /*!< SPI1 global Interrupt                                */
  SPI2_IRQn = 36,               /*!< SPI2 global Interrupt                                */
  USART1_IRQn = 37,             /*!< USART1 global Interrupt                              */
  USART2_IRQn = 38,             /*!< USART2 global Interrupt                              */
  USART3_IRQn = 39,             /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn = 40,          /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn = 41,           /*!< RTC Alarm through EXTI Line Interrupt                */
  CEC_IRQn = 42,                /*!< HDMI-CEC Interrupt                                   */
  TIM12_IRQn = 43,              /*!< TIM12 global Interrupt                               */
  TIM13_IRQn = 44,              /*!< TIM13 global Interrupt                               */
  TIM14_IRQn = 45,              /*!< TIM14 global Interrupt                               */
  TIM5_IRQn = 50,               /*!< TIM5 global Interrupt                                */
  SPI3_IRQn = 51,               /*!< SPI3 global Interrupt                                */
  UART4_IRQn = 52,              /*!< UART4 global Interrupt                               */
  UART5_IRQn = 53,              /*!< UART5 global Interrupt                               */
  TIM6_DAC_IRQn = 54,           /*!< TIM6 and DAC underrun Interrupt                      */
  TIM7_IRQn = 55,               /*!< TIM7 Interrupt                                       */
  DMA2_Channel1_IRQn = 56,      /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn = 57,      /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn = 58,      /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_5_IRQn = 59,    /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
  DMA2_Channel5_IRQn = 60       /*!< DMA2 Channel 5 global Interrupt (DMA2 Channel 5 is 
                                             mapped at position 60 only if the MISC_REMAP bit in 
                                             the AFIO_MAPR2 register is set)                      */
#endif                          /* STM32F10X_HD_VL */

#ifdef STM32F10X_XL
  ADC1_2_IRQn = 18,             /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn = 20,    /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn = 21,           /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn = 22,           /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn = 23,            /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM9_IRQn = 24,      /*!< TIM1 Break Interrupt and TIM9 global Interrupt       */
  TIM1_UP_TIM10_IRQn = 25,      /*!< TIM1 Update Interrupt and TIM10 global Interrupt     */
  TIM1_TRG_COM_TIM11_IRQn = 26, /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn = 27,            /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn = 28,               /*!< TIM2 global Interrupt                                */
  TIM3_IRQn = 29,               /*!< TIM3 global Interrupt                                */
  TIM4_IRQn = 30,               /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn = 31,            /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn = 32,            /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn = 33,            /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn = 34,            /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn = 35,               /*!< SPI1 global Interrupt                                */
  SPI2_IRQn = 36,               /*!< SPI2 global Interrupt                                */
  USART1_IRQn = 37,             /*!< USART1 global Interrupt                              */
  USART2_IRQn = 38,             /*!< USART2 global Interrupt                              */
  USART3_IRQn = 39,             /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn = 40,          /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn = 41,           /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn = 42,          /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
  TIM8_BRK_TIM12_IRQn = 43,     /*!< TIM8 Break Interrupt and TIM12 global Interrupt      */
  TIM8_UP_TIM13_IRQn = 44,      /*!< TIM8 Update Interrupt and TIM13 global Interrupt     */
  TIM8_TRG_COM_TIM14_IRQn = 45, /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn = 46,            /*!< TIM8 Capture Compare Interrupt                       */
  ADC3_IRQn = 47,               /*!< ADC3 global Interrupt                                */
  FSMC_IRQn = 48,               /*!< FSMC global Interrupt                                */
  SDIO_IRQn = 49,               /*!< SDIO global Interrupt                                */
  TIM5_IRQn = 50,               /*!< TIM5 global Interrupt                                */
  SPI3_IRQn = 51,               /*!< SPI3 global Interrupt                                */
  UART4_IRQn = 52,              /*!< UART4 global Interrupt                               */
  UART5_IRQn = 53,              /*!< UART5 global Interrupt                               */
  TIM6_IRQn = 54,               /*!< TIM6 global Interrupt                                */
  TIM7_IRQn = 55,               /*!< TIM7 global Interrupt                                */
  DMA2_Channel1_IRQn = 56,      /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn = 57,      /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn = 58,      /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_5_IRQn = 59     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
#endif                          /* STM32F10X_XL */

#ifdef STM32F10X_CL
  ADC1_2_IRQn = 18,        /*!< ADC1 and ADC2 global Interrupt                       */
  CAN1_TX_IRQn = 19,       /*!< USB Device High Priority or CAN1 TX Interrupts       */
  CAN1_RX0_IRQn = 20,      /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn = 21,      /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn = 22,      /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn = 23,       /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn = 24,      /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn = 25,       /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn = 26,  /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn = 27,       /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn = 28,          /*!< TIM2 global Interrupt                                */
  TIM3_IRQn = 29,          /*!< TIM3 global Interrupt                                */
  TIM4_IRQn = 30,          /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn = 31,       /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn = 32,       /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn = 33,       /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn = 34,       /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn = 35,          /*!< SPI1 global Interrupt                                */
  SPI2_IRQn = 36,          /*!< SPI2 global Interrupt                                */
  USART1_IRQn = 37,        /*!< USART1 global Interrupt                              */
  USART2_IRQn = 38,        /*!< USART2 global Interrupt                              */
  USART3_IRQn = 39,        /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn = 41,      /*!< RTC Alarm through EXTI Line Interrupt                */
  OTG_FS_WKUP_IRQn = 42,   /*!< USB OTG FS WakeUp from suspend through EXTI Line Interrupt */
  TIM5_IRQn = 50,          /*!< TIM5 global Interrupt                                */
  SPI3_IRQn = 51,          /*!< SPI3 global Interrupt                                */
  UART4_IRQn = 52,         /*!< UART4 global Interrupt                               */
  UART5_IRQn = 53,         /*!< UART5 global Interrupt                               */
  TIM6_IRQn = 54,          /*!< TIM6 global Interrupt                                */
  TIM7_IRQn = 55,          /*!< TIM7 global Interrupt                                */
  DMA2_Channel1_IRQn = 56, /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn = 57, /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn = 58, /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_IRQn = 59, /*!< DMA2 Channel 4 global Interrupt                      */
  DMA2_Channel5_IRQn = 60, /*!< DMA2 Channel 5 global Interrupt                      */
  ETH_IRQn = 61,           /*!< Ethernet global Interrupt                            */
  ETH_WKUP_IRQn = 62,      /*!< Ethernet Wakeup through EXTI line Interrupt          */
  CAN2_TX_IRQn = 63,       /*!< CAN2 TX Interrupt                                    */
  CAN2_RX0_IRQn = 64,      /*!< CAN2 RX0 Interrupt                                   */
  CAN2_RX1_IRQn = 65,      /*!< CAN2 RX1 Interrupt                                   */
  CAN2_SCE_IRQn = 66,      /*!< CAN2 SCE Interrupt                                   */
  OTG_FS_IRQn = 67         /*!< USB OTG FS global Interrupt                          */
#endif                     /* STM32F10X_CL */
} IRQn_Type;

/**
 * @}
 * */

/**
 * @}
 * */

/**
 * @brief 外设基类
 * @details 定义了基类下new和delete的方式 \n
 *          基于此类的派生类将只允许栈上构造和placement new \n
 *          delete将不会释放内存
 **/
class Perieh_Cls
{
private:
  void *operator new(size_t);

public:
  //void *operator new(size_t, void *p) { return p; };
  void operator delete(void *p, size_t){};
};
