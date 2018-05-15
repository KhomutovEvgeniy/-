#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "misc.h"
#include "stdio.h"

#define ServoPrivod_PORT1 GPIOx
#define ServoPrivod_PORT2 GPIOx
#define USART1_PORT_Tx GPIOA
#define USART1_PORT_Rx GPIOA
#define SERVO_1_PIN GPIO_Pin_x
#define SERVO_2_PIN GPIO_Pin_x
#define SERVO_3_PIN GPIO_Pin_x
#define SERVO_4_PIN GPIO_Pin_x
#define SERVO_5_PIN GPIO_Pin_x

// Массив (буфер) показаний резисторов, считывающихся с ADC
static volatile uint16_t ADC_buffer[] = {0, 0, 0, 0, 0};
// Массив принятых значений с USART1
static volatile uint8_t USART_buffer[] = {0, 0, 0, 0, 0};
static volatile uint32_t timeStampMs = 0;

// Local functions prototype
void usart_init (void);
void dma_usart_init (void);
void DelayMs(uint32_t delay);

void usart_init (void)
{
  /* Enable USART1 and GPIOA PORT */
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE); //Разрешить тактирование ножек Rx и Tx
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE); //Разрешить тактирование USART1
  /* Configure the GPIOs */
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART1_PORT_Tx, &GPIO_InitStructure);

  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART1_PORT_Rx, &GPIO_InitStructure);

  /* Configure the USART1 */
  USART_InitTypeDef USART_InitStructure;

/* USART1 configuration ------------------------------------------------------*/
  /* USART1 configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the middle
        - USART LastBit: The clock pulse of the last data bit is not output to
                         the SCLK pin
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE); //Enable USART1
}
/*
void USART_Send(uint16_t * pucBuffer)
{
  while (counter != 5)
  {
    USART_SendData(USART1, pucBuffer[counter++]);
    while( ! USART_GetFlagStatus(USART1, USART_FLAG_TXE )); // Ждем пока не освободится USART
  }
}
*/

void dma_usart_init (void)
{
  // Разрешение тактирвоания модуля DMA
  RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1, ENABLE); 
  
  // Настройка модуля DMA1 - c USART1 в буфер
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) USART_buffer;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART1->DR);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = sizeof(USART_buffer);
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);	//4 канал - Tx USART1
   
  DMA_Cmd(DMA1_Channel4, ENABLE); //Включаем прямой доступ к памяти
  // Активируем передачу в последовательный порт по запросу DMA 
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); 
  
  // Установка прерываний от DMA по окончании приема 
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);  
}
// Временная задержка через SysTick
void DelayMs(uint32_t delay)
{
	uint32_t currentTimeMs = timeStampMs;
	while( timeStampMs < currentTimeMs + delay );
}
// Обработчик прерывания по событию - DMA1 передал показания всех 5-ти резисторов в регистр USART1
void DMA1_Channel4_IRQHandler(void)
{
  DMA_ClearITPendingBit(DMA1_IT_TC4);
	DMA_Cmd(DMA1_Channel4, DISABLE);
}

void SysTick_Handler()
{
	timeStampMs++;
}

int main(void)
{
  SysTick_Config(SystemCoreClock / 1000);
  while (1)
  {
    usart_init();
    dma_usart_init();
    DelayMs(100);
  }
}

// классический ассерт для STM32
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t * file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
     
    (void)file;
    (void)line;

    __disable_irq();
    while(1)
    {
        // это ассемблерная инструкция "отладчик, стой тут"
        // если вы попали сюда, значит вы ошиблись в параметрах. Смотрите в call stack
        __BKPT(0xAB);
    }
}
#endif
