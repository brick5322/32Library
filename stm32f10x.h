/**
  ******************************************************************************
  * @file    stm32f10x.h
  * @author  LIUV
  * @version V1.0
  * @date    13-Sep-2021
  ******************************************************************************
**/

#pragma once

typedef unsigned int uint_32;
typedef unsigned short uint_16;
typedef unsigned char uint_8;

#define PERIEH_BASE (0X40000000)

#define Add_APB1 (PERIEH_BASE)
#define Add_APB2 ((PERIEH_BASE + 0X10000))
#define Add_AHB ((PERIEH_BASE + 0X20000))

#define RCC ((RCC_Typedef *)(Add_AHB + 0x1000))

typedef struct GPIO_Typedef
{
    uint_32 CRL;
    uint_32 CRH;

    uint_16 IDR_reserved;
    uint_16 IDR;

    uint_16 ODR_reserved;
    uint_16 ODR;

    uint_16 BSRR_reset;
    uint_16 BSRR_set;

    uint_16 BRR_reserved;
    uint_16 BRR;

    bool LCKR_LCKK;
    uint_16 LCKR_LCK;
} GPIO_Typedef;

typedef struct RCC_Typedef
{
    uint_32 RCC_CR;
    uint_32 RCC_CFGR;
    uint_32 RCC_CIR;
    uint_32 RCC_APB2RSTR;
    uint_32 RCC_APB1RSTR;
    uint_32 RCC_AHBENR;
    uint_32 RCC_APB2ENR;
    uint_32 RCC_APB1ENR;
    uint_32 RCC_BDCR;
    uint_32 RCC_CSR;
} RCC_Typedef;
