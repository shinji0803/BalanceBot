/********************************************************************************/
/*!
	@file			hw_config.c
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        2.00
    @date           2014.07.01
	@brief          Configure Basis System on STM32F3-Discovery.

    @section HISTORY
		2014.03.04	V1.00	Start Here.
		2014.07.01	V2.00	Adopted STM32 HAL Drivers.

    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

/* Defines -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
__IO uint16_t CmdKey=0;

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**************************************************************************/
/*! 
    @brief	Configures Main system clocks & power.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void Set_System(void)
{
	/* SystemInit was already executed in asm startup */
	
	/* NVIC Configuration */
	/*
	 *NVIC_PriorityGroup_0: 0 Pre-emption priorities, 16 Sub-priorities
	 *NVIC_PriorityGroup_1: 2 Pre-emption priorities, 8 Sub-priorities
	 *NVIC_PriorityGroup_2: 4 Pre-emption priorities, 4 Sub-priorities
	 *NVIC_PriorityGroup_3: 8 Pre-emption priorities, 2 Sub-priorities
	 *NVIC_PriorityGroup_4: 16 Pre-emption priorities, 0 Sub-priorities
	 */
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_1);

#if defined(USE_SEMIHOSTING)
	initialise_monitor_handles();
#endif

	/* Configure the LED  */
	LED_Configuration();

}


/**************************************************************************/
/*! 
    @brief	Configures the LED on STM32F3Discovery.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void LED_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO_LED clock */
	RCC->AHBENR |= (RCC_AHBPeriph_GPIO_LED);

	/* Configure GPIO for LEDs as Output push-pull */
	GPIO_InitStructure.Pin 			= LED_D3;
	GPIO_InitStructure.Mode 		= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull 		= GPIO_NOPULL;
	GPIO_InitStructure.Speed 		= GPIO_SPEED_HIGH;
	GPIO_InitStructure.Alternate 	= 0;
	HAL_GPIO_Init(GPIO_LED, &GPIO_InitStructure);

	LED_D3_OFF();
}


/**************************************************************************/
/*! 
    @brief	Configures the KEY-Input.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void KEY_Configuration(void)
{

}


/* End Of File ---------------------------------------------------------------*/
