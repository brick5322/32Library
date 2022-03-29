/**
  ******************************************************************************
  * @file    stm32f10x_NVIC.cpp
  * @author  LIUVBC
  * @version V1.0
  * @date    29-Sep-2021
  * @brief   NVIC固件库源文件
  ******************************************************************************
**/

#include "stm32f10x_NVIC.h"

/**
 * @brief 构造函数，设置寄存器组
 * @param NVIC_PriorityGroup 占先优先级和副优先级的分配
 * \n 见 @ref NVIC相关宏定义
 * @param Vec_Base 向量表地址(使用常量优化)\n
 *        参数仅在定义宏 Dynatic_VecTable 时可用
 * \n 见 @ref 常量参数优化宏定义
 **/
#ifdef Dynatic_VecTable
NVIC_Cls::NVIC_Cls(uint32_t NVIC_PriorityGroup, Addr Vec_Base = Arg_Vectable(VecTable_Area_FLASH, 0))
{
    SCB->AIRCR = 0x05FA0000 | NVIC_PriorityGroup;
    if (Vec_Base == VecTable_Area_Dynatic)
        Dynatic_VectorTable();
    else
        SCB->VTOR = Vec_Base;
}
#else
NVIC_Cls::NVIC_Cls(uint32_t NVIC_PriorityGroup)
{
    SCB->AIRCR = 0x05FA0000 | NVIC_PriorityGroup;
}
#endif

NVIC_Cls::~NVIC_Cls()
{
    SCB->AIRCR = 0x05FA0000;
}

/**
 * @brief 设置NVIC寄存器以启用中断 IRQ 
 * @param IRQ 中断号
 * @param PreemptionPriority 占先优先级(使用常量优化)
 * \n 见 @ref 常量参数优化宏定义
 * @param SubPriority 副优先级(使用常量优化)
 * \n 见 @ref 常量参数优化宏定义
 **/
void NVIC_Cls::Enable_IRQ(IRQn_Type IRQ, Priority PreemptionPriority, Priority SubPriority)
{
    uint8_t tmpPriority = 0x0F;
    uint8_t PreempBits = ((0x700 - ((SCB->AIRCR) & (uint32_t)0x700)) >> 0x08);

    tmpPriority <<= PreempBits;
    tmpPriority &= (PreemptionPriority << PreempBits);
    tmpPriority |= SubPriority;

    this->IP[IRQ] = tmpPriority;

    this->ISER[IRQ >> 5] = (1 << (IRQ & 0X1F));
}

/**
 * @brief 禁用中断 
 * @param IRQ 中断号
 * \n 见 @ref stm32f10x中断号枚举
 **/
void NVIC_Cls::Disable_IRQ(IRQn_Type IRQ)
{
    this->ICER[IRQ >> 5] = (1 << (IRQ & 0X1F));
}

/**
 * @brief 检查中断号是否被挂起 
 * @param IRQ 中断号
 * \n 见 @ref stm32f10x中断号枚举
 **/
bool NVIC_Cls::IRQ_isPending(IRQn_Type IRQ)
{
    return this->ISPR[(uint32_t)(IRQ) >> 5] & (1 << ((uint32_t)IRQ & 0x1F)) ? 1 : 0;
}

/**
 * @brief 搁置中断 
 * @param IRQ 中断号
 * \n 见 @ref stm32f10x中断号枚举
 **/
void NVIC_Cls::Pend_IRQ(IRQn_Type IRQ)
{
    this->ISPR[((uint32_t)(IRQ) >> 5)] = (1 << ((uint32_t)(IRQ)&0x1F));
}

/**
 * @brief 解除中断搁置状态 
 * @param IRQ 中断号
 * \n 见 @ref stm32f10x中断号枚举
 **/
void NVIC_Cls::ClearPending_IRQ(IRQn_Type IRQ)
{
    this->ICPR[((uint32_t)(IRQ) >> 5)] = (1 << ((uint32_t)(IRQ)&0x1F));
}

#ifdef Dynatic_VecTable

/**
 * @brief 将自定义中断处理函数映射到中断向量表中 \n
 *        仅在定义宏 Dynatic_VecTable 时可用
 * @param IRQ 中断号
 * \n 见 @ref stm32f10x中断号枚举
 * @param IRQ_Entrance 中断处理函数指针
 * */
void NVIC_Cls::Set_IRQfunc(IRQn_Type IRQ, IRQ_func IRQ_Entrance)
{
    ((IRQ_func *)SCB->VTOR)[IRQ + 16] = IRQ_Entrance;
}

/**
 * @brief 设置向量表寄存器，使之与FLASH或SRAM上向量表位置一致 \n
 *        仅在定义宏 Dynatic_VecTable 时可用
 * @param Vec_Base 向量表地址(使用常量优化)
 * \n 见 @ref 常量参数优化宏定义
 **/
void NVIC_Cls::LinkVectorTable(Addr Vec_Base)
{
    SCB->VTOR = Vec_Base;
}

void NVIC_Cls::Dynatic_VectorTable()
{
    IRQ_func *Vec_Base = (IRQ_func *)SCB->VTOR;
    for (int i = 0; i < Dynatic_Vector_Size; i++)
        DVec_ptr[i] = Vec_Base[i];
    SCB->VTOR = (uint32_t)DVec_ptr;
}
#endif
