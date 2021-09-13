/**
  ******************************************************************************
  * @file    stm32f10x_GPIO.cpp
  * @author  LIUV
  * @version V1.0
  * @date    13-Sep-2021
  ******************************************************************************
**/

#include "stm32f10x_GPIO.h"
#include "stm32f10x_RCC.h"

GPIO::GPIO(GPIO_Typedef *GPIO_Port, uint_16 Pin, GPIO_Mode Mode) : GPIO_Port(GPIO_Port), Pin(Pin)
{
    APB2_SetEnable(1 << (((int)GPIO_Port - Add_APB2) / 0x400));
    bool pullup = Mode & 0x10;
    bool pulldown = Mode & 0x20;
    Mode &= 0xf;
    for (int i = 0; i < 8; i++)
    {
        if (Pin & 1)
        {
            GPIO_Port->CRL |= (Mode << (i * 4));
            if (pullup)
                GPIO_Port->BSRR_set = (1 << i);
            else if (pulldown)
                GPIO_Port->BSRR_reset = (1 << i);
        }
        Pin >>= 1;
    }
    for (int i = 0; i < 8; i++)
    {
        if (Pin & 1)
        {
            GPIO_Port->CRH |= (Mode << (i * 4));
            if (pullup)
                GPIO_Port->BSRR_set = (1 << i);
            else if (pulldown)
                GPIO_Port->BSRR_reset = (1 << i);
        }
        Pin >>= 1;
    }
}

GPIO::~GPIO()
{
    APB2_SetDisable((1 << ((int)GPIO_Port - Add_APB2) / 0x400));
    GPIO_Port->CRH = 0X44444444;
    GPIO_Port->CRL = 0x44444444;
    GPIO_Port->ODR = 0X0000;
    GPIO_Port->BSRR_reset = 0x0000;
    GPIO_Port->BSRR_set = 0X0000;
    GPIO_Port->BRR = 0X0000;
    GPIO_Port->LCKR_LCK = 0X0000;
    GPIO_Port->LCKR_LCKK = 0X0;
}

void GPIO::Set_Bit(uint_16 Pin)
{
    GPIO_Port->BSRR_set = Pin;
}

void GPIO::Reset_Bit(uint_16 Pin)
{
    GPIO_Port->BRR = Pin;
}

void GPIO::Set_AllBits()
{
    GPIO_Port->BSRR_set = Pin;
}
void GPIO::Reset_AllBits()
{
    GPIO_Port->BRR = Pin;
}