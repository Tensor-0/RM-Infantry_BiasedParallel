#ifndef BSP_UART_H
#define BSP_UART_H

#include "stm32h7xx.h"

extern void BSP_USART_Init(void);
	   
extern void usart_printf(const char *fmt,...);		 
		 
		 
#endif