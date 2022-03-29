#include "USER_Defines.h"
#include "stm32f10x_RCC.h"

/**
 * @brief 时钟初始化超时处理
 * */
void Err_TimeOut()
{
	while(1);
}


/**
 * @brief SystemInit接口
 **/
extern"C" void SystemInit()
{
	new (RCC_ptr) RCC_Cls;
}
