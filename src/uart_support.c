/********************************************************************************/
/*!
	@file			uart_support.c
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

/* Includes ------------------------------------------------------------------*/
#include "uart_support.h"
/* check header file version for fool proof */
#if __UART_SUPPORT_H!= 0x0600
#error "header file version is not correspond!"
#endif

/* Defines -------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
static USART_Buffer_t* pUSART_Buf;
USART_Buffer_t USARTx_Buf;

/* Constants -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**************************************************************************/
/*! 
    Initialize UART.
*/
/**************************************************************************/
/* Initialize serial console */
void conio_init(uint32_t port, uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	UART_HandleTypeDef UartHandle;

	/* Zero-Clear Struct */ 
	HAL_CLR_STRUCT(UartHandle);

	/* Turn on USART*/
	switch (port)
	{
		case 1 :	/* Currentry not Supported! */
		break;
		
		case 2 :	/* UART2 Settings */
			UartHandle.Instance        = USART2;
			
			/* Enable GPIO clock */
			RCC->AHBENR |= (RCC_AHBENR_GPIOAEN);
			/* Enable UART clock */
			RCC->APB1ENR |= (RCC_APB1ENR_USART2EN);
			
			/* Configure USART Tx as alternate function  */
			GPIO_InitStructure.Pin 			= GPIO_PIN_2;
			GPIO_InitStructure.Mode 		= GPIO_MODE_AF_PP;
			GPIO_InitStructure.Pull 		= GPIO_PULLUP;
			GPIO_InitStructure.Speed 		= GPIO_SPEED_HIGH;
			GPIO_InitStructure.Alternate 	= GPIO_AF7_USART2;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

			/* Configure USART Rx as alternate function  */
			GPIO_InitStructure.Pin 			= GPIO_PIN_15;
			GPIO_InitStructure.Mode 		= GPIO_MODE_AF_PP;
			GPIO_InitStructure.Pull 		= GPIO_PULLUP;
			GPIO_InitStructure.Speed 		= GPIO_SPEED_HIGH;
			GPIO_InitStructure.Alternate 	= GPIO_AF7_USART2;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

			UartHandle.Init.BaudRate   		= baudrate;
			UartHandle.Init.WordLength 		= UART_WORDLENGTH_8B;
			UartHandle.Init.StopBits   		= UART_STOPBITS_1;
			UartHandle.Init.Parity     		= UART_PARITY_NONE;
			UartHandle.Init.Mode       		= UART_MODE_TX_RX;
			UartHandle.Init.HwFlowCtl  		= UART_HWCONTROL_NONE;
			UartHandle.Init.OverSampling  	= UART_OVERSAMPLING_16;
			UartHandle.Init.OneBitSampling  = UART_ONEBIT_SAMPLING_DISABLED;
			UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
			if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
			{
				/* Capture error */ 
				while (1);
			}  
			if(HAL_UART_Init(&UartHandle) != HAL_OK)
			{
				/* Capture error */ 
				while (1);
			}
			
			/* Enable the USART2 Interrupt */
			NVIC_SetPriority(USART2_IRQn,
					         NVIC_EncodePriority(NVIC_GetPriorityGrouping(), /* Set to Group_1 */
							 1,			/* PreemptPriority Group_1: 0~1 */
						     2));		/* SubPriority Group_1: 0~7 */
			NVIC_EnableIRQ(USART2_IRQn);

			/* Init Ring Buffer */
			pUSART_Buf = &USARTx_Buf;
			USARTx_Buf.RX_Tail = 0;
			USARTx_Buf.RX_Head = 0;
			USARTx_Buf.TX_Tail = 0;
			USARTx_Buf.TX_Head = 0;
			
			/* Enable UART Receive interrupts */
			__HAL_UART_ENABLE_IT(&UartHandle, USART_IT_RXNE);
		break;
		
		case 3 :	/* Currentry not Supported! */
		break;
	}

}

/**************************************************************************/
/*! 
    Check UART TX Buffer Empty.
*/
/**************************************************************************/
bool USART_TXBuffer_FreeSpace(USART_Buffer_t* USART_buf)
{
	/* Make copies to make sure that volatile access is specified. */
	unsigned int tempHead = (USART_buf->TX_Head + 1) & (UART_BUFSIZE-1);
	unsigned int tempTail = USART_buf->TX_Tail;

	/* There are data left in the buffer unless Head and Tail are equal. */
	return (tempHead != tempTail);
}

/**************************************************************************/
/*! 
    Put Bytedata with Buffering.
*/
/**************************************************************************/
bool USART_TXBuffer_PutByte(USART_Buffer_t* USART_buf, uint8_t data)
{

	unsigned int tempTX_Head;
	bool TXBuffer_FreeSpace;

	TXBuffer_FreeSpace = USART_TXBuffer_FreeSpace(USART_buf);


	if(TXBuffer_FreeSpace)
	{
	  	tempTX_Head = USART_buf->TX_Head;
		
		__disable_irq();
	  	USART_buf->TX[tempTX_Head]= data;
		/* Advance buffer head. */
		USART_buf->TX_Head = (tempTX_Head + 1) & (UART_BUFSIZE-1);
		__enable_irq();

		/* Enable TXE interrupt. */
		UARTx->CR1 |= USART_CR1_TXEIE;
	}
	return TXBuffer_FreeSpace;
}

/**************************************************************************/
/*! 
    Check UART RX Buffer Empty.
*/
/**************************************************************************/
bool USART_RXBufferData_Available(USART_Buffer_t* USART_buf)
{
	/* Make copies to make sure that volatile access is specified. */
	unsigned int tempHead = pUSART_Buf->RX_Head;
	unsigned int tempTail = pUSART_Buf->RX_Tail;

	/* There are data left in the buffer unless Head and Tail are equal. */
	return (tempHead != tempTail);
}

/**************************************************************************/
/*! 
    Get Bytedata with Buffering.
*/
/**************************************************************************/
uint8_t USART_RXBuffer_GetByte(USART_Buffer_t* USART_buf)
{
	uint8_t ans;

	__disable_irq();
	ans = (pUSART_Buf->RX[pUSART_Buf->RX_Tail]);

	/* Advance buffer tail. */
	pUSART_Buf->RX_Tail = (pUSART_Buf->RX_Tail + 1) & (UART_BUFSIZE-1);
	
	__enable_irq();

	return ans;
}

/**************************************************************************/
/*! 
    High Level function.
*/
/**************************************************************************/
/* Send 1 character */
inline void putch(uint8_t data)
{
#if defined(UART_INTERRUPT_MODE)
	/* Interrupt Version */
	while(!USART_TXBuffer_FreeSpace(pUSART_Buf));
	USART_TXBuffer_PutByte(pUSART_Buf,data);
#else 
	/* Polling version */
	while (!(UARTx->ISR & USART_ISR_TXE));
	UARTx->TDR = data;
#endif
}

/**************************************************************************/
/*! 
    High Level function.
*/
/**************************************************************************/
/* Receive 1 character */
uint8_t getch(void)
{
#if defined(UART_INTERRUPT_MODE)
	if (USART_RXBufferData_Available(pUSART_Buf))  return USART_RXBuffer_GetByte(pUSART_Buf);
	else										   return false;
#else
	/* Polling version */
	while (!(UARTx->ISR & USART_ISR_RXNE));
	return (uint8_t)(UARTx->RDR);
#endif
}

/**************************************************************************/
/*! 
    High Level function.
*/
/**************************************************************************/
/* Return 1 if key pressed */
uint8_t keypressed(void)
{
#if defined(UART_INTERRUPT_MODE)
	return (USART_RXBufferData_Available(pUSART_Buf));
#else
	return (UARTx->ISR & USART_ISR_RXNE);
#endif
}

/**************************************************************************/
/*! 
    High Level function.
*/
/**************************************************************************/
/* Send a string */
void cputs(char *s)
{
	while (*s)
    putch(*s++);
}

/**************************************************************************/
/*! 
    High Level function.
*/
/**************************************************************************/
/* Receive a string, with rudimentary line editing */
void cgets(char *s, int bufsize)
{
	char *p;
	int c;

	memset(s, 0, bufsize);

	p = s;

	for (p = s; p < s + bufsize-1;)
	{
		/* 20090521Nemui */
		do{		
			c = getch();
		}while(c == false);
		/* 20090521Nemui */
		switch (c)
		{
			case '\r' :
			case '\n' :
				putch('\r');
				putch('\n');
				*p = '\n';
			return;

			case '\b' :
				if (p > s)
				{
				  *p-- = 0;
				  putch('\b');
				  putch(' ');
				  putch('\b');
				}
			break;

			default :
				putch(c);
				*p++ = c;
			break;
		}
	}
	return;
}

#if defined(UART_INTERRUPT_MODE)
/**************************************************************************/
/*! 
    @brief	Handles UARTx global interrupt request.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void USARTx_IRQHandler(void)
{
	uint32_t IntStat = UARTx->ISR;

	if(IntStat & USART_ISR_RXNE)
	{
		/* Advance buffer head. */
		unsigned int tempRX_Head = ((&USARTx_Buf)->RX_Head + 1) & (UART_BUFSIZE-1);

		/* Check for overflow. */
		unsigned int tempRX_Tail = (&USARTx_Buf)->RX_Tail;
		uint8_t data =  UARTx->RDR;

		if (tempRX_Head == tempRX_Tail) {
			/* Overflow MAX size Situation */
			/* Disable the UART Receive interrupt */
			UARTx->CR1 &= ~(USART_CR1_RXNEIE);
		}else{
			(&USARTx_Buf)->RX[(&USARTx_Buf)->RX_Head] = data;
			(&USARTx_Buf)->RX_Head = tempRX_Head;
		}
	}

	if(IntStat & USART_ISR_TXE)
	{   
		/* Check if all data is transmitted. */
		unsigned int tempTX_Tail = (&USARTx_Buf)->TX_Tail;
		if ((&USARTx_Buf)->TX_Head == tempTX_Tail){
			/* Overflow MAX size Situation */
			/* Disable the UART Transmit interrupt */
			UARTx->CR1 &= ~(USART_CR1_TXEIE);
		}else{
			/* Start transmitting. */
			uint8_t data = (&USARTx_Buf)->TX[(&USARTx_Buf)->TX_Tail];
			UARTx->TDR = data;

			/* Advance buffer tail. */
			(&USARTx_Buf)->TX_Tail = ((&USARTx_Buf)->TX_Tail + 1) & (UART_BUFSIZE-1);
		}
	}
}
#endif

/* End Of File ---------------------------------------------------------------*/
