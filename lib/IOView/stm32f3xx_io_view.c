/**************************************************************************/
/*!
	@file			stm32f3xx_io_view.c
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        1.00
    @date           2014.07.01
	@brief          Debugging I/O Definitions For STM32F3xx Devices.

    @section HISTORY
		2014.07.01	V1.00	Start Here.
		
    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"

/* Defines -------------------------------------------------------------------*/
typedef union GPIODATA_t {

	__IO uint16_t DUMMY;
	struct {
		__IO	uint16_t DR_00:	1;
		__IO	uint16_t DR_01:	1;
		__IO	uint16_t DR_02:	1;
		__IO	uint16_t DR_03:	1;
		__IO	uint16_t DR_04:	1;
		__IO	uint16_t DR_05:	1;
		__IO	uint16_t DR_06:	1;
		__IO	uint16_t DR_07:	1;
		__IO	uint16_t DR_08:	1;
		__IO	uint16_t DR_09:	1;
		__IO	uint16_t DR_10:	1;
		__IO	uint16_t DR_11:	1;
		__IO	uint16_t DR_12:	1;
		__IO	uint16_t DR_13:	1;
		__IO	uint16_t DR_14:	1;
		__IO	uint16_t DR_15:	1;
	} b_DR;
} GPIODATA_t;

/* Constants -----------------------------------------------------------------*/
volatile TIM_TypeDef 			* const IO_TIM2            __attribute__ ((section(".ioview"))) = TIM2;
volatile TIM_TypeDef 			* const IO_TIM3			   __attribute__ ((section(".ioview"))) = TIM3;
#if !defined (STM32F334x8)
volatile TIM_TypeDef 			* const IO_TIM4			   __attribute__ ((section(".ioview"))) = TIM4;
#endif
volatile TIM_TypeDef 			* const IO_TIM6			   __attribute__ ((section(".ioview"))) = TIM6;
volatile TIM_TypeDef 			* const IO_TIM7			   __attribute__ ((section(".ioview"))) = TIM7;
volatile RTC_TypeDef 			* const IO_RTC			   __attribute__ ((section(".ioview"))) = RTC;
volatile WWDG_TypeDef 			* const IO_WWDG			   __attribute__ ((section(".ioview"))) = WWDG;
volatile IWDG_TypeDef 			* const IO_IWDG			   __attribute__ ((section(".ioview"))) = IWDG;
#if !defined (STM32F334x8)
volatile SPI_TypeDef 			* const IO_I2S2ext		   __attribute__ ((section(".ioview"))) = I2S2ext;
volatile SPI_TypeDef 			* const IO_SPI2			   __attribute__ ((section(".ioview"))) = SPI2;
volatile SPI_TypeDef 			* const IO_SPI3			   __attribute__ ((section(".ioview"))) = SPI3;
volatile SPI_TypeDef 			* const IO_I2S3ext		   __attribute__ ((section(".ioview"))) = I2S3ext;
#endif
volatile USART_TypeDef 			* const IO_USART2		   __attribute__ ((section(".ioview"))) = USART2;
volatile USART_TypeDef 			* const IO_USART3		   __attribute__ ((section(".ioview"))) = USART3;
#if !defined (STM32F334x8)
volatile USART_TypeDef 			* const IO_UART4		   __attribute__ ((section(".ioview"))) = UART4;
volatile USART_TypeDef 			* const IO_UART5		   __attribute__ ((section(".ioview"))) = UART5;
#endif
volatile I2C_TypeDef 			* const IO_I2C1			   __attribute__ ((section(".ioview"))) = I2C1;
#if !defined (STM32F334x8)
volatile I2C_TypeDef 			* const IO_I2C2			   __attribute__ ((section(".ioview"))) = I2C2;
#if !defined (STM32F303xC)
volatile CAN_TypeDef 			* const IO_CAN1            __attribute__ ((section(".ioview"))) = CAN1;
#endif
#endif
volatile PWR_TypeDef 			* const IO_PWR             __attribute__ ((section(".ioview"))) = PWR;
volatile DAC_TypeDef 			* const IO_DAC             __attribute__ ((section(".ioview"))) = DAC;
volatile SYSCFG_TypeDef 		* const IO_SYSCFG          __attribute__ ((section(".ioview"))) = SYSCFG;
volatile COMP_TypeDef 			* const IO_COMP            __attribute__ ((section(".ioview"))) = COMP;
volatile COMP_TypeDef 			* const IO_COMP2           __attribute__ ((section(".ioview"))) = COMP2;
volatile COMP_TypeDef 			* const IO_COMP4           __attribute__ ((section(".ioview"))) = COMP4;
volatile COMP_TypeDef 			* const IO_COMP6           __attribute__ ((section(".ioview"))) = COMP6;
#if !defined (STM32F334x8)
volatile COMP_TypeDef 			* const IO_COMP1           __attribute__ ((section(".ioview"))) = COMP1;
volatile COMP_TypeDef 			* const IO_COMP3           __attribute__ ((section(".ioview"))) = COMP3;
volatile COMP_TypeDef 			* const IO_COMP5           __attribute__ ((section(".ioview"))) = COMP5;
volatile COMP_TypeDef 			* const IO_COMP7           __attribute__ ((section(".ioview"))) = COMP7;
#endif
volatile OPAMP_TypeDef 			* const IO_OPAMP           __attribute__ ((section(".ioview"))) = OPAMP;
volatile OPAMP_TypeDef 			* const IO_OPAMP2          __attribute__ ((section(".ioview"))) = OPAMP2;
#if !defined (STM32F334x8)
volatile OPAMP_TypeDef 			* const IO_OPAMP1          __attribute__ ((section(".ioview"))) = OPAMP1;
volatile OPAMP_TypeDef 			* const IO_OPAMP3          __attribute__ ((section(".ioview"))) = OPAMP3;
volatile OPAMP_TypeDef 			* const IO_OPAMP4          __attribute__ ((section(".ioview"))) = OPAMP4;
#endif
volatile EXTI_TypeDef 			* const IO_EXTI            __attribute__ ((section(".ioview"))) = EXTI;
volatile TIM_TypeDef 			* const IO_TIM1            __attribute__ ((section(".ioview"))) = TIM1;
volatile SPI_TypeDef			* const IO_SPI1            __attribute__ ((section(".ioview"))) = SPI1;
#if !defined (STM32F334x8)
volatile TIM_TypeDef 			* const IO_TIM8            __attribute__ ((section(".ioview"))) = TIM8;
#endif
volatile USART_TypeDef 			* const IO_USART1          __attribute__ ((section(".ioview"))) = USART1;
volatile TIM_TypeDef 			* const IO_TIM15           __attribute__ ((section(".ioview"))) = TIM15;
volatile TIM_TypeDef 			* const IO_TIM16           __attribute__ ((section(".ioview"))) = TIM16;
volatile TIM_TypeDef 			* const IO_TIM17           __attribute__ ((section(".ioview"))) = TIM17;
volatile DBGMCU_TypeDef 		* const IO_DBGMCU          __attribute__ ((section(".ioview"))) = DBGMCU;
volatile DMA_TypeDef 			* const IO_DMA1            __attribute__ ((section(".ioview"))) = DMA1;
volatile DMA_Channel_TypeDef 	* const IO_DMA1_Channel1   __attribute__ ((section(".ioview"))) = DMA1_Channel1;
volatile DMA_Channel_TypeDef 	* const IO_DMA1_Channel2   __attribute__ ((section(".ioview"))) = DMA1_Channel2;
volatile DMA_Channel_TypeDef 	* const IO_DMA1_Channel3   __attribute__ ((section(".ioview"))) = DMA1_Channel3;
volatile DMA_Channel_TypeDef 	* const IO_DMA1_Channel4   __attribute__ ((section(".ioview"))) = DMA1_Channel4;
volatile DMA_Channel_TypeDef 	* const IO_DMA1_Channel5   __attribute__ ((section(".ioview"))) = DMA1_Channel5;
volatile DMA_Channel_TypeDef 	* const IO_DMA1_Channel6   __attribute__ ((section(".ioview"))) = DMA1_Channel6;
volatile DMA_Channel_TypeDef 	* const IO_DMA1_Channel7   __attribute__ ((section(".ioview"))) = DMA1_Channel7;
#if !defined (STM32F334x8)
volatile DMA_TypeDef 			* const IO_DMA2            __attribute__ ((section(".ioview"))) = DMA2;
volatile DMA_Channel_TypeDef 	* const IO_DMA2_Channel1   __attribute__ ((section(".ioview"))) = DMA2_Channel1;
volatile DMA_Channel_TypeDef 	* const IO_DMA2_Channel2   __attribute__ ((section(".ioview"))) = DMA2_Channel2;
volatile DMA_Channel_TypeDef 	* const IO_DMA2_Channel3   __attribute__ ((section(".ioview"))) = DMA2_Channel3;
volatile DMA_Channel_TypeDef 	* const IO_DMA2_Channel4   __attribute__ ((section(".ioview"))) = DMA2_Channel4;
volatile DMA_Channel_TypeDef 	* const IO_DMA2_Channel5   __attribute__ ((section(".ioview"))) = DMA2_Channel5;
#endif
volatile RCC_TypeDef 			* const IO_RCC             __attribute__ ((section(".ioview"))) = RCC;
volatile FLASH_TypeDef 			* const IO_FLASH           __attribute__ ((section(".ioview"))) = FLASH ;
volatile OB_TypeDef 			* const IO_OB              __attribute__ ((section(".ioview"))) = OB;
volatile CRC_TypeDef 			* const IO_CRC             __attribute__ ((section(".ioview"))) = CRC;
volatile TSC_TypeDef 			* const IO_TSC             __attribute__ ((section(".ioview"))) = TSC;
volatile GPIO_TypeDef         	* const IO_GPIOA		   __attribute__ ((section(".ioview"))) = GPIOA;
volatile GPIODATA_t           	* const IO_GPIOA_ODR       __attribute__ ((section(".ioview"))) = (GPIODATA_t *)&GPIOA->ODR;
volatile GPIO_TypeDef         	* const IO_GPIOB		   __attribute__ ((section(".ioview"))) = GPIOB;
volatile GPIODATA_t           	* const IO_GPIOB_ODR       __attribute__ ((section(".ioview"))) = (GPIODATA_t *)&GPIOB->ODR;
volatile GPIO_TypeDef         	* const IO_GPIOC		   __attribute__ ((section(".ioview"))) = GPIOC;
volatile GPIODATA_t           	* const IO_GPIOC_ODR       __attribute__ ((section(".ioview"))) = (GPIODATA_t *)&GPIOC->ODR;
volatile GPIO_TypeDef         	* const IO_GPIOD		   __attribute__ ((section(".ioview"))) = GPIOD;
volatile GPIODATA_t           	* const IO_GPIOD_ODR       __attribute__ ((section(".ioview"))) = (GPIODATA_t *)&GPIOD->ODR;
#if !defined (STM32F334x8)
volatile GPIO_TypeDef         	* const IO_GPIOE		   __attribute__ ((section(".ioview"))) = GPIOE;
volatile GPIODATA_t           	* const IO_GPIOE_ODR       __attribute__ ((section(".ioview"))) = (GPIODATA_t *)&GPIOE->ODR;
#endif
volatile GPIO_TypeDef         	* const IO_GPIOF		   __attribute__ ((section(".ioview"))) = GPIOF;
volatile GPIODATA_t           	* const IO_GPIOF_ODR       __attribute__ ((section(".ioview"))) = (GPIODATA_t *)&GPIOF->ODR;
volatile ADC_TypeDef 			* const IO_ADC1            __attribute__ ((section(".ioview"))) = ADC1;
volatile ADC_TypeDef 			* const IO_ADC2            __attribute__ ((section(".ioview"))) = ADC2;
#if !defined (STM32F334x8)
volatile ADC_TypeDef 			* const IO_ADC3            __attribute__ ((section(".ioview"))) = ADC3;
volatile ADC_TypeDef 			* const IO_ADC4            __attribute__ ((section(".ioview"))) = ADC4;
#if !defined (STM32F303xC)
volatile ADC_Common_TypeDef 	* const IO_ADC1_2          __attribute__ ((section(".ioview"))) = ADC1_2;
volatile ADC_Common_TypeDef 	* const IO_ADC3_4          __attribute__ ((section(".ioview"))) = ADC3_4;
#endif
#endif

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/* End Of File ---------------------------------------------------------------*/
