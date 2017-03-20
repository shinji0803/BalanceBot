/********************************************************************************/
/*!
	@file			uart_support.h
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        6.00
    @date           2015.09.18
	@brief          Based on ST Microelectronics Sample Thanks!

    @section HISTORY
		2014.06.24	V1.00	Start Here.
		2014.07.01	V2.00	Adopted STM32 HAL Drivers.
		2014.07.11	V3.00	Simplified some functions.
		2014.07.19	V4.00	Added Struct Clear on Init.
		2015.01.11	V5.00	Added buffered UART information.
		2015.09.18	V6.00	Fixed Wrong Expression.

    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/
#ifndef __UART_SUPPORT_H
#define __UART_SUPPORT_H	0x0600

#ifdef __cplusplus
 extern "C" {
#endif

/* General Inclusion */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>

/* uC Specific Inclusion */
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USART Definition */
#define UART_BUFSIZE			128		/* Buffer size MUST Takes power of 2(64,128,256,512...) */
#define UART_BAUDRATE			230400UL
#define UART_INTERRUPT_MODE		/* If u want polling mode, uncomment this */


#if   defined(USE_STM32F3DISCOVERY)
 #define UART_DEFAULT_NUM		1
 #define UARTx					USART1
 #define USARTx_IRQHandler		USART1_IRQHandler
 #define USARTx_AF				GPIO_AF7_USART1
#elif defined(USE_NUCLEO_F303K8)
 #define UART_DEFAULT_NUM		2
 #define UARTx					USART2
 #define USARTx_IRQHandler		USART2_IRQHandler
 #define USARTx_AF				GPIO_AF7_USART2
#endif

/* General Definition */
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Funcion Prototypes */
extern void conio_init(uint32_t port, uint32_t baudrate);
extern void putch(uint8_t c);
extern uint8_t getch(void);
extern uint8_t keypressed(void);
extern void cputs(char *s);
extern void cgets(char *s, int bufsize);

/* Structs of UART(This is Based on AVRX uC Sample!!!) */
/* @brief USART transmit and receive ring buffer. */
typedef struct USART_Buffer
{
	/* @brief Receive buffer. */
	volatile uint8_t RX[UART_BUFSIZE];
	/* @brief Transmit buffer. */
	volatile uint8_t TX[UART_BUFSIZE];
	/* @brief Receive buffer head. */
	volatile unsigned int RX_Head;
	/* @brief Receive buffer tail. */
	volatile unsigned int RX_Tail;
	/* @brief Transmit buffer head. */
	volatile unsigned int TX_Head;
	/* @brief Transmit buffer tail. */
	volatile unsigned int TX_Tail;
} USART_Buffer_t;

/* Externs */
extern USART_InitTypeDef USART_InitStructure;
extern USART_Buffer_t USARTx_Buf;


#ifdef __cplusplus
}
#endif

#endif	/* __UART_SUPPORT_H */
