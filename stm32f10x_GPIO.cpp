/**
  ******************************************************************************
  * @file    stm32f10x_GPIO.cpp
  * @author  LIUVBC
  * @version V1.0
  * @date    17-Sep-2021  
  * @brief  GPIO外设固件库源文件
  ******************************************************************************
**/

#include "stm32f10x_GPIO.h"

/**
 * @brief 构造GPIO对象（包括端口和引脚）并打开时钟
 * @param GPIO_Port GPIO端口对应的地址
 *   @arg GPIOx(x为A~G)
 * @param Pin   GPIO待设置的引脚
 *   @arg GPIO_Pin_x(x为0~15或All)
 *   \n 见 @ref  GPIO相关宏定义
 * @param GPIO_Mode 对 Pin 设置
 *   @arg GPIO_Mode_xx 
 *   \n 见 @ref  GPIO相关宏定义
*/
GPIO_Cls::GPIO_Cls(uint16_t Pin, uint8_t Mode)
{
    int &APB2_GPIO_Bit = *(int *)(0x42420300 + (uint8_t)((uint32_t)this >> 8));
    APB2_GPIO_Bit = 1;
    bool pullup = Mode & 0x10;
    bool pulldown = Mode & 0x20;
    Mode &= 0xf;
    for (int i = 0; i < 8; i++)
    {
        if (Pin & 1)
        {
            this->CRL |= (Mode << (i * 4));
            if (pullup)
                this->BSRR_set = (1 << i);
            else if (pulldown)
                this->BSRR_reset = (1 << i);
        }
        Pin >>= 1;
    }
    for (int i = 0; i < 8; i++)
    {
        if (Pin & 1)
        {
            this->CRH |= (Mode << (i * 4));
            if (pullup)
                this->BSRR_set = (1 << i);
            else if (pulldown)
                this->BSRR_reset = (1 << i);
        }
        Pin >>= 1;
    }
}

/**
 * @brief GPIO的析构函数，关闭时钟，复位寄存器
 **/
GPIO_Cls::~GPIO_Cls()
{
    int &APB2_GPIO_Bit = *(int *)(0x42420300 + (uint8_t)((uint32_t)this >> 8));
    APB2_GPIO_Bit = 0;
    this->CRH = 0X44444444;
    this->CRL = 0x44444444;
    this->ODR = 0X0000;
    this->BSRR_reset = 0x0000;
    this->BSRR_set = 0X0000;
    this->BRR = 0X0000;
    this->LCKR_LCK = 0X0000;
    this->LCKR_LCKK = 0X0;
}

/**
 * @brief 引脚置位
 * @param Pin 引脚
 * \n 见 @ref  GPIO相关宏定义
 **/
void GPIO_Cls::Set_Bit(uint16_t Pin)
{
    this->BSRR_set = Pin;
}

/**
 * @brief 引脚复位
 * @param Pin 引脚
 * \n 见 @ref  GPIO相关宏定义
 **/
void GPIO_Cls::Reset_Bit(uint16_t Pin)
{
    this->BRR = Pin;
}

/**
 * @brief 读取单个引脚的值
 * @param Pin 单个引脚
 * \n 见 @ref  GPIO相关宏定义
 * @return 引脚值
 **/
bool GPIO_Cls::Read_Bit(uint16_t Pin)
{
    return (this->IDR & Pin) ? true : false;
}

/**
 * @brief 置位全部引脚
 **/
void GPIO_Cls::Set_AllBits()
{
    this->BSRR_set = 0xffff;
}

/**
 * @brief 复位全部引脚
 **/
void GPIO_Cls::Reset_AllBits()
{
    this->BRR = 0x0000;
}

/**
 * @brief 读取全部引脚
 * @return GPIO端口的16位数 
 **/
uint16_t GPIO_Cls::Read_AllBits()
{
    return this->IDR;
}

/**
 * @brief 按16位写入
 **/
void GPIO_Cls::Write_AllBits(uint16_t Data)
{
    this->ODR = Data;
}

/**
 * @brief 锁定GPIO对象的引脚配置(GPIO_Mode)，复位前保持锁定
 * @param GPIO 要锁定配置的GPIO对象
 * @param Pin 锁定的端口
 * \n 见 @ref  GPIO相关宏定义
 * @return 
 *    @arg 0 成功
 *    @arg 1 失败
 **/
bool Lock_Config(GPIO_Cls &GPIOx, uint16_t Pin)
{
    bool ret;
    if (GPIOx.LCKR_LCKK)
        return Failed;
    GPIOx.LCKR_LCK = Pin;
    GPIOx.LCKR_LCKK = 1;
    GPIOx.LCKR_LCKK = 0;
    GPIOx.LCKR_LCKK = 1;
    ret = GPIOx.LCKR_LCKK;
    ret = GPIOx.LCKR_LCKK;
    return !ret;
}

/**
 * @brief 修改复用端口重映射的配置
 *        调试状态下,会检查参数匹配
 *        详见官方文档
 * @param Periph 对应的寄存器地址
 * @param Mode 重映射类型
 * \n 见@ref  AFIO相关宏定义
 **/
void AFIO_Cls::AFIO_Remap(void *Periph, uint8_t Mode)
{
    switch ((int)Periph)
    {
    case 0:

        break;

    default:
        break;
    }
}

/**
 * @brief SWJ调试模式
 * @param Mode
 * \n 见 @ref 调试模式
 **/
void AFIO_Cls::Debug_Mode(uint8_t Mode)
{
    AFIO_ptr->AFIO_MAPR_SWJ = Mode;
}
