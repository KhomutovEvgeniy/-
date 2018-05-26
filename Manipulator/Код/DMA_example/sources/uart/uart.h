#pragma once

#include "stm32f10x.h"

#define PC_LINK_UART	USART1
#define PC_LINK_UART_PORT	GPIOA
#define PC_LINK_UART_TX_PIN	GPIO_Pin_9
#define PC_LINK_UART_RX_PIN	GPIO_Pin_10

#define PC_LINK_UART_BR	115200

// macros
#define PC_LINK_UART_RCC_CMD(x) RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, x)
#define PC_LINK_UART_PORT_RCC_CMD(x) RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, x)

// global functions
void InitLinkPcUart( void );

void SendToPC( char* buf, uint32_t len );


