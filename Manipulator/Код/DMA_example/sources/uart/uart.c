#include "uart.h"

// local functions
void initUartPins( void );

static char* txBuf;
static uint32_t byteNumberToSend;
static uint32_t sendCnt;

void InitLinkPcUart()
{
	PC_LINK_UART_RCC_CMD(ENABLE);
	
	initUartPins();
	
	// config PC_UART
	USART_InitTypeDef uartInitStruct;
	USART_StructInit( &uartInitStruct );
	uartInitStruct.USART_BaudRate = PC_LINK_UART_BR;
	uartInitStruct.USART_Mode = USART_Mode_Tx;
	USART_Init( PC_LINK_UART, &uartInitStruct );
	
	
	// UART for DMA config
	
	
	
	
	
// 	NVIC_InitTypeDef nvicInitStruct;
// 	nvicInitStruct.NVIC_IRQChannel = USART1_IRQn;
// 	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
// 	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;
// 	nvicInitStruct.NVIC_IRQChannelSubPriority = 0;
// 	NVIC_Init( &nvicInitStruct );
	
	USART_Cmd( PC_LINK_UART, ENABLE );
	
	// IT config
	USART_DMACmd( PC_LINK_UART, USART_DMAReq_Tx, ENABLE );
	
	// init DMA
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );
}

void initUartPins()
{
	PC_LINK_UART_PORT_RCC_CMD(ENABLE);
	
	GPIO_InitTypeDef gpioInitStruct;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	gpioInitStruct.GPIO_Pin = PC_LINK_UART_TX_PIN;
	GPIO_Init( PC_LINK_UART_PORT, &gpioInitStruct );
	
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStruct.GPIO_Pin = PC_LINK_UART_RX_PIN;
	GPIO_Init( PC_LINK_UART_PORT, &gpioInitStruct );
}

void SendToPC( char* buf, uint32_t len )
{
	DMA_DeInit( DMA1_Channel4 );
	
	DMA_InitTypeDef dmaInitStruct;
	DMA_StructInit( &dmaInitStruct );
	dmaInitStruct.DMA_BufferSize = len;
	dmaInitStruct.DMA_MemoryBaseAddr = (uint32_t) buf;
	dmaInitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	dmaInitStruct.DMA_M2M = DMA_M2M_Disable;
	dmaInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dmaInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dmaInitStruct.DMA_Mode = DMA_Mode_Normal;
	dmaInitStruct.DMA_PeripheralBaseAddr = (uint32_t) &(PC_LINK_UART->DR);
	dmaInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dmaInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Init( DMA1_Channel4, &dmaInitStruct );
	
	DMA_Cmd( DMA1_Channel4, ENABLE );
}

void USART1_IRQHandler()
{
	if( USART_GetFlagStatus( PC_LINK_UART, USART_FLAG_TXE ) )
	{
		if ( byteNumberToSend == sendCnt )
		{
			USART_ITConfig( PC_LINK_UART, USART_IT_TXE, DISABLE );
		}
		
		sendCnt++;
		PC_LINK_UART->DR = txBuf[ sendCnt ];
	}
}

