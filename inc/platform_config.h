/********************************************************************************/
/*!
	@file			platform_config.h
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        1.00
    @date           2014.06.17
	@brief          Based on ST Microelectronics's Sample Thanks!

    @section HISTORY
		2014.06.17	V1.00	Start Here.

    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H 0x0100

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"
#include "stm32f3xx_hal_conf.h"

#ifndef BSRR
#define BSRR BSRRL			/* Set   */
#define BRR  BSRRH			/* Reset */
#endif

/* Exported types ------------------------------------------------------------*/
/* LED Definitions */
#define LED_OFF(y,x)			(y ->BRR = x)
#define LED_ON(y,x)				(y ->BSRR  = x)

/* NUCLEO-F303K8 */
/* Green LED : PB3 */
#define GPIO_LED               	(GPIOB)
#define RCC_AHBPeriph_GPIO_LED	(RCC_AHBENR_GPIOBEN)
#define LED_D3      			(GPIO_PIN_3)
#define LED_D3_ON()				LED_ON(GPIO_LED,LED_D3)
#define LED_D3_OFF()			LED_OFF(GPIO_LED,LED_D3)
#define LED_D3_TOGGLE()			(GPIO_LED->ODR ^= (1 << 3))


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif	/* __PLATFORM_CONFIG_H */
