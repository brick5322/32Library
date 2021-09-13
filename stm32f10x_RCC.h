/**
  ******************************************************************************
  * @file    stm32f10x_RCC.h
  * @author  LIUV
  * @version V1.0
  * @date    13-Sep-2021
  ******************************************************************************
**/

#pragma once

#include "stm32f10x.h"

#define APB2_AFIO (0x1)
#define APB2_EXTI (0x4)
#define APB2_GPIOA (0x8)
#define APB2_GPIOB (0xc)
#define APB2_GPIOC (0x10)
#define APB2_GPIOD (0x20)
#define APB2_GPIOE (0x40)
#define APB2_GPIOF (0x80)
#define APB2_GPIOG (0x100)
#define APB2_ADC1 (0x200)
#define APB2_ADC2 (0x400)
#define APB2_TIM1 (0x800)
#define APB2_SPI1 (0x1000)
#define APB2_TIM8 (0x2000)
#define APB2_USART1 (0x4000)
#define APB2_ADC3 (0x8000)

void APB2_SetEnable(uint_32 APB2_PERIPH);

void APB2_SetDisable(uint_32 APB2_PERIPH);
