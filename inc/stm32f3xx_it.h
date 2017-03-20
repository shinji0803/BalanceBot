/********************************************************************************/
/*!
	@file			stm32f3xx_it.h
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        1.00
    @date           2014.07.01
	@brief          Cortex-M4 Processor Exceptions Handlers.				@n
					And STM32F3xx Peripherals Interrupt Handlers.			@n
					Device Dependent Section.

    @section HISTORY
		2014.07.01	V1.00	Start Here.

    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/
#ifndef __STM32F3XX_IT_H
#define __STM32F3XX_IT_H 0x0100

#ifdef __cplusplus
 extern "C" {
#endif

/* General Inclusion */
#include "stm32f3xx.h"

/* Macros */

/* Externals */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void EXTI0_IRQHandler(void);


#ifdef __cplusplus
}
#endif

#endif /* __STM32F3XX_IT_H*/
