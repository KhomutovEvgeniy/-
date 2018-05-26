/**************************************************************************************************

**************************************************************************************************/

#include "stm32f10x.h"
#include "fput_debug.h"
#include "uart/uart.h"


const char message[] = "Hallo, Ich heisse Egor!";
int size;
static volatile uint32_t timeStampMs = 0;

// local functions signatures

void allGPIOInit( void );
void DelayMs(uint32_t delay);

// global functions

int main( void )
{		
	SysTick_Config( SystemCoreClock/1000 );
	
	InitLinkPcUart();
	
	size = sizeof(message);
	
	while(1)
	{					
		DelayMs( 100 );
		SendToPC( (char*)message, sizeof(message)-1 );
	}
}


void SysTick_Handler()
{
	timeStampMs++;
}


// local functions

void DelayMs(uint32_t delay)
{
	uint32_t currentTimeMs = timeStampMs;
	while( timeStampMs < currentTimeMs + delay );
}

void allGPIOInit( void )
{
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE);
	
	GPIO_InitTypeDef	gpioInitStructure;
	GPIO_StructInit( &gpioInitStructure );
	gpioInitStructure.GPIO_Pin = 0xFFFF;
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	
	GPIO_Init( GPIOC, &gpioInitStructure );
	GPIO_Init( GPIOD, &gpioInitStructure );
	GPIO_Init( GPIOE, &gpioInitStructure );
#ifdef DEBUF_VIEWER_PRINTF_USE
	gpioInitStructure.GPIO_Pin = 0xFFFF ^ ( GPIO_Pin_3 | GPIO_Pin_4 );	
#endif	
	GPIO_Init( GPIOB, &gpioInitStructure );
	
	gpioInitStructure.GPIO_Pin = 0xFFFF ^ ( GPIO_Pin_13 | GPIO_Pin_14 );	// ноги SW-отладчика
	GPIO_Init( GPIOA, &gpioInitStructure );
}


void assert_failed(uint8_t* file, uint32_t line)
{
	while(1);
}



