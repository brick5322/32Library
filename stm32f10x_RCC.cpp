/**
  ******************************************************************************
  * @file    stm32f10x_RCC.cpp
  * @author  LIUV
  * @version V1.0
  * @date    13-Sep-2021
  ******************************************************************************
**/

#include "stm32f10x_RCC.h"

void APB2_SetEnable(uint_32 APB2_PERIPH)
{
    RCC->RCC_APB2ENR |= APB2_PERIPH;
}

void APB2_SetDisable(uint_32 APB2_PERIPH)
{
    RCC->RCC_APB2ENR &= ~APB2_PERIPH;
}