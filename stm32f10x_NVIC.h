/**
  ******************************************************************************
  * @file    stm32f10x_NVIC.h
  * @author  LIUVBC
  * @version V1.0
  * @date    29-Sep-2021  
  * @brief   NVIC固件库头文件
  ******************************************************************************
**/
#pragma once
#include "stm32f10x.h"
#include "core_cm3.h"

/**
 * @addtogroup 地址宏定义
 * @{
 **/

/**
 * @defgroup Cortex-M3 内核地址宏定义
 * @{
 **/

#define NVIC_ptr ((NVIC_Cls *)0xE000E100)

/**
 * @}
 **/

/**
 * @}
 **/

/**
 * @addtogroup 常量实参枚举和宏定义
 * @{
 **/

/**
 * @defgroup NVIC相关宏定义
 * @{
 **/
typedef void (*IRQ_func)(void);
#define Preemption_priority_0bit ((uint32_t)0x700) /*!< 0位占先优先级 */
#define Preemption_priority_1bit ((uint32_t)0x600) /*!< 1位占先优先级 */
#define Preemption_priority_2bit ((uint32_t)0x500) /*!< 2位占先优先级 */
#define Preemption_priority_3bit ((uint32_t)0x400) /*!< 3位占先优先级 */
#define Preemption_priority_4bit ((uint32_t)0x300) /*!< 4位占先优先级 */

/**
 * @}
 **/

/**
 * @}
 **/

/**
 * @addtogroup 常量参数优化宏定义
 * @{
 * */

typedef int Addr;
/**
 * @brief 处理中断向量表地址的常量实参
 * @param Area 
 *   @arg VecTable_Area_SRAM 映射在SRAM上
 *   @arg VecTable_Area_FLASH 映射在FLASH
 * @param Offset 偏移量,须被0x200整除
 * */
#define Arg_Vectable(Area, Offset) Area ? ((Addr)(Area | (Offset & 0x1FFFFF80))) : 0

typedef uint8_t Priority;
/**
 * @brief 处理优先级的常量实参
 * @param priority 占先优先级或副优先级
 * */
#define Arg_priority(priority) ((Priority)(priority << 4))
/**
 * @}
 **/

#define VecTable_Area_SRAM 0x20000000  /*!<向量表映射在SRAM首地址 */
#define VecTable_Area_FLASH 0x08000000 /*!<向量表映射在FLASH首地址*/

#ifdef Dynatic_VecTable
#define VecTable_Area_Dynatic 0x00000000 /*!<向量表动态映射需要配置启动文件*/
static IRQ_func DVec_ptr[100];
static int Dynatic_Vector_Size = 100;
#endif

/**
 * @brief NVIC类
 * */
class NVIC_Cls : public Perieh_Cls
{
private:
  uint32_t ISER[8];           /*!<   Interrupt Set Enable Register           */
  uint32_t RESERVED0[24];
  uint32_t ICER[8];           /*!<   Interrupt Clear Enable Register         */
  uint32_t RSERVED1[24];
  uint32_t ISPR[8];           /*!<   Interrupt Set Pending Register          */
  uint32_t RESERVED2[24];
  uint32_t ICPR[8];           /*!<   Interrupt Clear Pending Register        */
  uint32_t RESERVED3[24];
  uint32_t IABR[8];           /*!<   Interrupt Active bit Register           */
  uint32_t RESERVED4[56];
  uint8_t IP[240];            /*!<   Interrupt Priority Register (8Bit wide) */
  uint32_t RESERVED5[644];
  uint32_t STIR;              /*!<   Software Trigger Interrupt Register     */
#ifdef Dynatic_VecTable
  void Dynatic_VectorTable();
#endif
public:
#ifdef Dynatic_VecTable
  NVIC_Cls(uint32_t NVIC_PriorityGroup, Addr Vec_Base);
#else
  NVIC_Cls(uint32_t NVIC_PriorityGroup);
#endif

  ~NVIC_Cls();
  void Enable_IRQ(IRQn_Type IRQ, Priority PreemptionPriority, Priority SubPriority);
  void Disable_IRQ(IRQn_Type IRQ);
  bool IRQ_isPending(IRQn_Type IRQ);
  void Pend_IRQ(IRQn_Type IRQ);
  void ClearPending_IRQ(IRQn_Type IRQ);

#ifdef Dynatic_VecTable
  void Set_IRQfunc(IRQn_Type IRQ, IRQ_func func);
  void LinkVectorTable(Addr Vec_Base);
#endif
};
