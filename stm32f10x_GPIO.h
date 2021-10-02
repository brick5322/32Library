/**
  ******************************************************************************
  * @file    stm32f10x_GPIO.h
  * @author  LIUVBC
  * @version V1.0
  * @date    17-Sep-2021
  * @brief  GPIO外设固件库头文件
  ******************************************************************************
**/

#pragma once
#include "stm32f10x.h"
#include "astm32f10x.h"
/**
 * @addtogroup APB2总线地址宏定义
 * @{
 * */
#define GPIOA_ptr ((GPIO_Cls *)(Addr_APB2 + 0x0800))
#define GPIOB_ptr ((GPIO_Cls *)(Addr_APB2 + 0x0C00))
#define GPIOC_ptr ((GPIO_Cls *)(Addr_APB2 + 0x1000))
#define GPIOD_ptr ((GPIO_Cls *)(Addr_APB2 + 0x1400))
#define GPIOE_ptr ((GPIO_Cls *)(Addr_APB2 + 0x1800))
#define GPIOF_ptr ((GPIO_Cls *)(Addr_APB2 + 0x1C00))
#define GPIOG_ptr ((GPIO_Cls *)(Addr_APB2 + 0x2000))

/**
 * @}
 * */

/**
 * @addtogroup 常量实参枚举和宏定义
 * @{
 **/

/**
 * @defgroup GPIO相关宏定义
 * @{
 **/

#define GPIO_Mode_IN_A (0x00)       /*!< 模拟输入     */
#define GPIO_Mode_IN_FLOATING (0x4) /*!< 浮空输入     */
#define GPIO_Mode_IN_PD (0x18)      /*!< 下拉输入     */
#define GPIO_Mode_IN_PU (0x28)      /*!< 上拉输入     */
#define GPIO_Mode_Out_OD (0x07)     /*!< 开漏输出     */
#define GPIO_Mode_Out_PP (0x06)     /*!< 推挽输出     */
#define GPIO_Mode_AF_OD (0x0f)      /*!< 复用开漏输出 */
#define GPIO_Mode_AF_PP (0x0b)      /*!< 复用推挽输出 */

#define GPIO_Mode_2Mhz (0x2)  /*!< 输出频率2Mhz */
#define GPIO_Mode_10Mhz (0x1) /*!< 输出频率10Mhz */
#define GPIO_Mode_50Mhz (0x3) /*!< （默认）输出频率50Mhz */

#define GPIO_Pin_0 ((uint16_t)0x0001)   /*!< 引脚0    */
#define GPIO_Pin_1 ((uint16_t)0x0002)   /*!< 引脚1    */
#define GPIO_Pin_2 ((uint16_t)0x0004)   /*!< 引脚2    */
#define GPIO_Pin_3 ((uint16_t)0x0008)   /*!< 引脚3    */
#define GPIO_Pin_4 ((uint16_t)0x0010)   /*!< 引脚4    */
#define GPIO_Pin_5 ((uint16_t)0x0020)   /*!< 引脚5    */
#define GPIO_Pin_6 ((uint16_t)0x0040)   /*!< 引脚6    */
#define GPIO_Pin_7 ((uint16_t)0x0080)   /*!< 引脚7    */
#define GPIO_Pin_8 ((uint16_t)0x0100)   /*!< 引脚8    */
#define GPIO_Pin_9 ((uint16_t)0x0200)   /*!< 引脚9    */
#define GPIO_Pin_10 ((uint16_t)0x0400)  /*!< 引脚10   */
#define GPIO_Pin_11 ((uint16_t)0x0800)  /*!< 引脚11   */
#define GPIO_Pin_12 ((uint16_t)0x1000)  /*!< 引脚12   */
#define GPIO_Pin_13 ((uint16_t)0x2000)  /*!< 引脚13   */
#define GPIO_Pin_14 ((uint16_t)0x4000)  /*!< 引脚14   */
#define GPIO_Pin_15 ((uint16_t)0x8000)  /*!< 引脚15   */
#define GPIO_Pin_All ((uint16_t)0xFFFF) /*!< 全部引脚 */

/**
 * @}
 **/

/**
 * @}
 **/

/**
 * @brief GPIO类
 * */
class GPIO_Cls : public Perieh_Cls
{
private:
  uint32_t CRL;
  uint32_t CRH;

  uint16_t IDR_reserved;
  uint16_t IDR;

  uint16_t ODR_reserved;
  uint16_t ODR;

  uint16_t BSRR_reset;
  uint16_t BSRR_set;

  uint16_t BRR_reserved;
  uint16_t BRR;

  bool LCKR_LCKK;
  uint16_t LCKR_LCK;

public:
  GPIO_Cls(uint16_t Pin, uint8_t Mode);
  ~GPIO_Cls();
  void Set_Bit(uint16_t Pin);
  void Reset_Bit(uint16_t Pin);
  bool Read_Bit(uint16_t Pin);
  void Set_AllBits();
  void Reset_AllBits();
  uint16_t Read_AllBits();
  void Write_AllBits(uint16_t Data);
  friend bool Lock_Config(GPIO_Cls &GPIOx, uint16_t Pin);
};

/**
 * @brief AFIO类
 **/
class AFIO_Cls : public Perieh_Cls
{
private:
  uint8_t AFIO_EVCR_reserved[3];
  uint8_t AFIO_MAPR_SWJ;
  uint8_t AFIO_MAPR_TIM5; /*  非互联型包含ADC，互联型包含CAN2和MAC  */
  uint16_t AFIO_MAPR;
  uint16_t AFIO_EXTICR1_reserved;
  uint16_t AFIO_EXTICR1;
  uint16_t AFIO_EXTICR2_reserved;
  uint16_t AFIO_EXTICR2;
  uint16_t AFIO_EXTICR3_reserved;
  uint16_t AFIO_EXTICR3;
  uint16_t AFIO_EXTICR4_reserved;
  uint16_t AFIO_EXTICR4;

public:
  static void AFIO_Remap(void *Periph, uint8_t Mode);
  static void Debug_Mode(uint8_t Mode);
};

/**
 * @addtogroup 常量实参枚举和宏定义
 * @{
 **/

/**
 * @defgroup AFIO相关宏定义
 * @{
 **/

#define AFIO_Default_Remap (0x00)    /*!< 默认映射              */
#define AFIO_Partical_Remap_1 (0x01) /*!< 部分引脚重映射（方式1）*/
#define AFIO_Partical_Remap_2 (0x02) /*!< 部分引脚重映射（方式2）*/
#define AFIO_Full_Remap (0x3)        /*!< 全部引脚重映射         */

/**
 * @}
 **/

/**
 * @}
 **/

/*
  这里要有匹配性的判断
*/

/**
 * @addtogroup 常量实参枚举和宏定义
 * @{
 **/

/**
 * @defgroup 调试模式宏定义
 * @{
 **/
#define Debug_SWJ_FULL (0x0)      /*!<启用JTAG和SW-DP*/
#define Debug_SWJ_No_nJTRST (0x1) /*!<禁用nJTRST*/
#define Debug_SW_Only (0x2)       /*!<仅启用SW-DP*/
#define Debug_None (0x4)          /*!<禁用SW-DP和JTAG*/

/**
 * @}
 **/

/**
 * @}
 **/
