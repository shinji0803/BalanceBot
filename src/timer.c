
// Timer Functions

#include "timer.h"

static volatile uint32_t millis_timer;

// Initialize Systick Timer
// Interrupt interval = 1msec
void init_SysTick(void)
{
	SystemCoreClockUpdate();
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	
	if (SysTick_Config(SystemCoreClock / INTERVAL)) {
		while (1);
	}
}

// Return msec time since boot uC
uint32_t millis(void)
{
	return millis_timer;
}

// Return usec time since boot uC
uint32_t micros(void)
{
	uint32_t micros = ((SYSTICK_CLOCK / INTERVAL - 1) - SysTick->VAL) / (SYSTICK_CLOCK / 1000000);
	return micros + (millis_timer * 1000);
}

uint32_t get_SysTick_RawCount(void)
{
	uint32_t millis_factor = millis_timer * (SYSTICK_CLOCK / INTERVAL - 1);
	
	return (uint32_t)((SYSTICK_CLOCK / INTERVAL - 1) - SysTick->VAL + millis_factor);
}

void delay_ms(__IO uint32_t msec)
{
	uint32_t start = millis_timer;
	uint32_t now = millis_timer;
	
	while((now - start) < msec){
		now = millis_timer;
	}
}

void delay_us(__IO uint32_t usec)
{
	uint32_t start = micros();
	uint32_t now = micros();
	
	while((now - start) < usec){
		now = micros();
	}
}

void SysTick_Handler(void)		/* 1kHz Timer ISR */
{
	millis_timer ++;
}
