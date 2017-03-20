
#ifndef __TIMER_H
#define __TIMER_H 0x0200

// Timer Functions

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx_it.h"

/* Systick Interval milliSecond order */
#define INTERVAL	1000

#define SYSTICK_CLOCK	64000000

// Systick Timer Initialize Function
void init_SysTick(void);

// Return Systick Counter Value
uint32_t get_SysTick_RawCount(void);

// Return Milliseconds Time since uC
uint32_t millis(void);

// Return Microseconds Time since uC
uint32_t micros(void);

void delay_ms(__IO uint32_t msec);

void delay_us(__IO uint32_t usec);


#ifdef __cplusplus
}
#endif

#endif
