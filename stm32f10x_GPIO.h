/**
  ******************************************************************************
  * @file    stm32f10x_GPIO.h
  * @author  LIUV
  * @version V1.0
  * @date    13-Sep-2021
  ******************************************************************************
**/

#pragma once
#include "stm32f10x.h"

#define GPIO_Mode_AIN (0x00)
#define GPIO_Mode_IN_FLOATING (0x4)
#define GPIO_Mode_IPD (0x18)
#define GPIO_Mode_IPU (0x28)
#define GPIO_Mode_Out_OD (0x07)
#define GPIO_Mode_Out_PP (0x06)
#define GPIO_Mode_AF_OD (0x0f)
#define GPIO_Mode_AF_PP (0x0b)

#define GPIO_Mode_2Mhz (0x2)
#define GPIO_Mode_10Mhz (0x1)
#define GPIO_Mode_50Mhz (0x3) //输出默认

typedef uint_8 GPIO_Mode;

#define GPIO_Pin_0 ((uint_16)0x0001)
#define GPIO_Pin_1 ((uint_16)0x0002)
#define GPIO_Pin_2 ((uint_16)0x0004)
#define GPIO_Pin_3 ((uint_16)0x0008)
#define GPIO_Pin_4 ((uint_16)0x0010)
#define GPIO_Pin_5 ((uint_16)0x0020)
#define GPIO_Pin_6 ((uint_16)0x0040)
#define GPIO_Pin_7 ((uint_16)0x0080)
#define GPIO_Pin_8 ((uint_16)0x0100)
#define GPIO_Pin_9 ((uint_16)0x0200)
#define GPIO_Pin_10 ((uint_16)0x0400)
#define GPIO_Pin_11 ((uint_16)0x0800)
#define GPIO_Pin_12 ((uint_16)0x1000)
#define GPIO_Pin_13 ((uint_16)0x2000)
#define GPIO_Pin_14 ((uint_16)0x4000)
#define GPIO_Pin_15 ((uint_16)0x8000)
#define GPIO_Pin_All ((uint_16)0xFFFF)

#define GPIOA ((GPIO_Typedef *)(Add_APB2 + 0x800))
#define GPIOB ((GPIO_Typedef *)(Add_APB2 + 0xC00))
#define GPIOC ((GPIO_Typedef *)(Add_APB2 + 0x1000))
#define GPIOD ((GPIO_Typedef *)(Add_APB2 + 0x1400))
#define GPIOE ((GPIO_Typedef *)(Add_APB2 + 0x1800))
#define GPIOF ((GPIO_Typedef *)(Add_APB2 + 0x1C00))
#define GPIOG ((GPIO_Typedef *)(Add_APB2 + 0x2000))

class GPIO
{
private:
    GPIO_Typedef *GPIO_Port;
    uint_16 Pin;

public:
    GPIO(GPIO_Typedef *GPIO_Port, uint_16 Pin, GPIO_Mode Mode);
    ~GPIO();
    void Set_Bit(uint_16 PIN);
    void Reset_Bit(uint_16 PIN);
    void Set_AllBits();
    void Reset_AllBits();
};
