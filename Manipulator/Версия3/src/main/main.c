#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "misc.h"
#include "stdio.h"

#define Resistor_PORT1 GPIOA
#define Resistor_PORT2 GPIOC
#define RES_1_PIN GPIO_Pin_0
#define RES_2_PIN GPIO_Pin_1
#define RES_3_PIN GPIO_Pin_4
#define RES_4_PIN GPIO_Pin_1
#define RES_5_PIN GPIO_Pin_0
/*
volatile char buffer[50] = {'\0'};
volatile short FLAG_ECHO = 0;
*/
/*
void TIM4_IRQHandler (void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    FLAG_ECHO = 1;
    TIM_ClearITPendingBit (TIM4, TIM_IT_Update);
  }
}
*/

void usart_init (void)
{
  /* Enable USART1 and GPIOA clock */
  
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE); //Разрешить тактирование ножек Rx и Tx
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE); //Разрешить тактирование USART1
  /* Configure the GPIOs */
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

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


volatile uint32_t ADCbuffer[5] = {1280, 1023, 1500, 2000, 500}; //Создание массива (буфера) значений, считывающихся с резисторов

volatile uint16_t counter = 0;
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
  // Разрешить тактирвоание модуля DMA
  RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1, ENABLE); 
  
  //RCC->APB2ENR |= RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN;
  //AFIO->MAPR |= AFIO_MAPR_USART2_REMAP;
  
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADCbuffer;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART1->DR);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = sizeof (ADCbuffer);
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);	//4 канал - Tx USART1
  
  // Активируем передачу в последовательный порт по запросу DMA 
  DMA_Cmd(DMA1_Channel4, ENABLE); //Включаем прямой доступ к памяти
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); 
  
 /* Установка прерываний от DMA по окончании передачи */
 DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
 NVIC_EnableIRQ(DMA1_Channel4_IRQn);  
 

}

void DMA1_Channel4_IRQHandler(void)
{
  /*
	if (DMA_GetITStatus(DMA1_IT_TC4) == SET)
  {
    DMA_ClearITPendingBit(DMA1_IT_TC4);
  }
  */
  DMA_ClearITPendingBit(DMA1_IT_TC4);
	DMA_Cmd(DMA1_Channel4, DISABLE);
  
}

int main(void)
{
  
  const unsigned char mytext[] = " Hello World\n!";
 // __enable_irq();
  //USART
  usart_init();
  dma_usart_init();

  while (1)
  {
  //  USART_Send(ADCbuffer);
  //  __NOP();
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
