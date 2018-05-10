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
__IO uint16_t Counter = 0;
__IO uint16_t TempValue;
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
  /* Enable USART2 and GPIOA clock */
  
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE); //Разрешить тактирование ножек Rx и Tx
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_USART2, ENABLE); //Разрешить тактирование USART2
  /* Configure the GPIOs */
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART2 Tx (PA.02) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART2 Rx (PA.03) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure the USART2 */
  USART_InitTypeDef USART_InitStructure;

/* USART2 configuration ------------------------------------------------------*/
  /* USART2 configured as follow:
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
  USART_Init(USART2, &USART_InitStructure);
  USART_Cmd(USART2, ENABLE); //Enable USART2
}

void USART_Send(const unsigned char *pucBuffer)
{
//  while (*pucBuffer)
//  {
//      USART_SendData(USART2, *pucBuffer++);
//      while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
//      {
//      }
//  }
  char s = 'e';
  while( ! USART_GetFlagStatus(USART2, USART_FLAG_TXE ) );
  USART_SendData(USART2, s);
  
}

volatile uint16_t ADCbuffer[100] = {1, 2, 3, 4, 5}; //Создание массива (буфера) значений, считывающихся с резисторов




int main(void)
{
    const unsigned char mytext[] = " Hello World!\r\n";
 
    //USART
    usart_init();
    
  
    while (1)
    {
       USART_Send(mytext);
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
