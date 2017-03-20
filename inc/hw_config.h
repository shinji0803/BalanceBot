/********************************************************************************/
/*!
	@file			hw_config.h
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
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H 0x0200

#ifdef __cplusplus
 extern "C" {
#endif

/* General Inclusion */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
#include "platform_config.h"

/* Function Inclusion */
#include "uart_support.h"
#include "timer.h"
#include "myMath.h"
#include "AHRS.h"
#include "MPU9250.h"
#include "motor.h"

/* Macros */
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Externals */
extern __IO uint16_t CmdKey;
extern void Set_System(void);
extern void NVIC_Configuration(void);
extern void LED_Configuration(void);
extern void KEY_Configuration(void);
extern void disk_timerproc(void);
extern void JoyInp_Chk(void);
extern void Ext_SramInit(void);
extern void initialise_monitor_handles();

#ifdef __cplusplus
}
#endif

#endif  /* __HW_CONFIG_H */
