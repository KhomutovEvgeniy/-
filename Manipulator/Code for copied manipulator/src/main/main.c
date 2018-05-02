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

volatile uint16_t ADC1buffer[] = {0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA}; //Создание массива (буфера) значений, считывающихся с резисторов
volatile char buffer[80] = {'\0'};
__IO uint16_t Counter = 0;
__IO uint16_t TempValue;
volatile short FLAG_ECHO = 0;
void Init_ADC_DMA (void)
{
  //Разрешить тактирование портов А и С
//  RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN, ENABLE); 
  
//  RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPCEN, ENABLE); //Разрешить тактирование порта C
//  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN; 
//  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
  
  
  RCC_ADCCLKConfig (RCC_PCLK2_Div6); //Настройка часов АЦП
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO | 
                          RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
  RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1, ENABLE); // Разрешить тактирвоание модуля DMA
  //Настройка первого входа АЦП (резистор в крутящемся основании)
  GPIO_InitTypeDef GPIO_InitStructure1;
  GPIO_InitStructure1.GPIO_Pin = RES_1_PIN | RES_2_PIN | RES_3_PIN;
  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init (Resistor_PORT1, &GPIO_InitStructure1);

  //Настройка четвертого входа АЦП (поворотная ось схвата)
  GPIO_InitTypeDef GPIO_InitStructure2;
  GPIO_InitStructure2.GPIO_Pin = RES_4_PIN || RES_5_PIN;
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init (Resistor_PORT2, &GPIO_InitStructure2);

  //Настройка DMA
  RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1, ENABLE); // Разрешить тактирвоание модуля DMA
  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint16_t)&ADC1->DR; //Откуда копировать считанное значение с АЦП
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint16_t)&ADC1buffer[0]; //Куда сохранять очередное значение
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //Указываем, что переферия(АЦП)является приемником
  DMA_InitStructure.DMA_BufferSize = 5; //пять каналов
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //Регистр периферийного адреса не увеличивается
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //Размер данных с АЦП (2байта)
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //Размер элементов массива (буфера) (2байта)
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //Режим циклического буфера
  DMA_InitStructure.DMA_Priority = DMA_Priority_High; //Высокий приоритет прог.обеспечения для DMA
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //memory-to-memory-disable
  DMA_Init (DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd (DMA1_Channel1, ENABLE); //Разрешение тактирования DMA
  
  //Настройка АЦП
  ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // Независимый режим (т.к. используем 1 АЦП)
  ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Сканирующий режим (т.к. сигнал будем снимать не с одной ножки)
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // Циклическое сканирование
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; // Источник запуска АЦП - 1 канал 1 таймера 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // Выравнимание битов по правому краю
  ADC_InitStructure.ADC_NbrOfChannel = 5; // Количество каналов сканирования
  ADC_Init (ADC1, &ADC_InitStructure);
  
  // Определение регулярной группы сканирования АЦП
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5); // Sample time equal to 55.5 cycles
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 4, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 5, ADC_SampleTime_55Cycles5);
  
  ADC_Cmd (ADC1, ENABLE); // Включение ADC1
  ADC_DMACmd (ADC1, ENABLE); // Включение ADC1 DMA
  ADC_ResetCalibration (ADC1);
  
  while (ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration (ADC1);
  
  while (ADC_GetCalibrationStatus(ADC1));
  ADC_SoftwareStartConvCmd (ADC1, ENABLE); // Запуск преобразований ADC
  
  //USART_DMACmd (USART2, USART_DMAReq_Tx, ENABLE); //Передача с помощью DMA на USART
  
  DMA_ITConfig (DMA1_Channel1, DMA_IT_TC, ENABLE); //Включение прерывания по окончании передачи
  NVIC_EnableIRQ(DMA1_Channel1_IRQn); //Разрешение прерываний DMA
  
/*
 //Настройка прерывания (которое по окончании трансфера)
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStructure);
*/
}

void usart_init (void)
{
  /* Enable USART2 and GPIOA clock */
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA, ENABLE); //Разрешить тактирование USART2

  /* NVIC Configuration */
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


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

  /* Enable USART2 */
  USART_Cmd(USART2, ENABLE);

  /* Enable the USART2 Receive interrupt: this interrupt is generated when the
       USART2 receive data register is not empty */
  //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void USARTSend(const unsigned char *pucBuffer)
{
  while (*pucBuffer)
  {
      USART_SendData(USART2, *pucBuffer++);
      while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
      {
      }
  }
}


void TIM4_IRQHandler (void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    FLAG_ECHO = 1;
    TIM_ClearITPendingBit (TIM4, TIM_IT_Update);
  }
}

uint32_t a = 0;
void delay(uint32_t i) 
{
  
  volatile uint32_t j;
  for (j=0; j!= i * 5000; j++);
}

void SetSysClockTo72(void)
{
  ErrorStatus HSEStartUpStatus;
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig( RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
      /* Enable Prefetch Buffer */
      //FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);

      /* Flash 2 wait state */
      //FLASH_SetLatency( FLASH_Latency_2);

      /* HCLK = SYSCLK */
      RCC_HCLKConfig (RCC_SYSCLK_Div1);

      /* PCLK2 = HCLK */
      RCC_PCLK2Config (RCC_HCLK_Div1);

      /* PCLK1 = HCLK/2 */
      RCC_PCLK1Config (RCC_HCLK_Div2);

      /* PLLCLK = 8MHz * 9 = 72 MHz */
      RCC_PLLConfig (0x00010000, RCC_PLLMul_9);

      /* Enable PLL */
      RCC_PLLCmd (ENABLE);

      /* Wait till PLL is ready */
      while (RCC_GetFlagStatus (RCC_FLAG_PLLRDY) == RESET)
      {
      }

      /* Select PLL as system clock source */
      RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);

      /* Wait till PLL is used as system clock source */
      while (RCC_GetSYSCLKSource() != 0x08)
      {
      }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock configuration.
   User can add here some code to deal with this error */

      /* Go to infinite loop */
      while (1)
      {
      }
  }
}

int main(void)
{
//  int i;
//  /* Initialize Leds mounted on STM32 board */
//  GPIO_InitTypeDef  GPIO_InitStructure;
//  /* Initialize LED which connected to PC13, Enable the Clock*/
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//  /* Configure the GPIO_LED pin */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

/*    
 GPIO_InitTypeDef PORT;
//Включаем порты А и С
RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA) , ENABLE);

  //Настраиваем ноги PA5 на выход. Там у нас висит светодиод
PORT.GPIO_Pin = (GPIO_Pin_5);

PORT.GPIO_Mode = GPIO_Mode_Out_PP;

PORT.GPIO_Speed = GPIO_Speed_2MHz;

GPIO_Init( GPIOA , &PORT);

//Порт A настраивать смысла нет, все его ноги по умолчанию входы что нам и нужно

RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE); //Включаем тактирование АЦП

ADC1->CR2 |= ADC_CR2_CAL; //Запуск калибровки АЦП

while (!(ADC1->CR2 & ADC_CR2_CAL)); //Ожидаем окончания калибровки

ADC1->SMPR2 |= (ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0); //Задаем

// длительность выборки

ADC1->CR2 |= ADC_CR2_JEXTSEL; //Преобразование инжектированной группы

//запустится установкой бита JSWSTART

ADC1->CR2 |= ADC_CR2_JEXTTRIG; //Разрешаем внешний запуск инжектированной группы

ADC1->CR2 |= ADC_CR2_CONT; //Преобразования запускаются одно за другим

ADC1->CR1 |= ADC_CR1_JAUTO; //Разрешить преобразование инжектированной группы

//после регулярной. Не понятно зачем, но без этого не работает

ADC1->JSQR |= (1<<15); //Задаем номер канала (выбран ADC1)

ADC1->CR2 |= ADC_CR2_ADON;//Теперь включаем АЦП

ADC1->CR2 |= ADC_CR2_JSWSTART; //Запуск преобразований

while (!(ADC1->SR & ADC_SR_JEOC)); //ждем пока первое преобразование завершится


//Теперь можно читать результат из JDR1

 //int32_t a; //Использовал переменную для отладки. Можно и без неё  
    

    
    
  while (1)
  {
a = ADC1->JDR1 - 1330;

delay(a); //Исходя из значения АЦП делаем задержку

 GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_RESET); //Гасим диод...

//// GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_SET); // Зажигаем диод...

//adcres=ADC1->JDR1;

delay(a); //Всё повторяется...

//// GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_RESET);

 GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_SET);
      
      
      
  }*/
  
SetSysClockTo72();
 
    const unsigned char mytext[] = " Hello World!\r\n";
 
    //USART
    usart_init();
    USARTSend(mytext);
 
    //ADC
    Init_ADC_DMA();
 
    // TIMER4
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 7200;
    TIMER_InitStructure.TIM_Period = 5000;
    TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
 
    /* NVIC Configuration */
    /* Enable the TIM4_IRQn Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    while (1)
    {
        if (FLAG_ECHO == 1) {
          sprintf(buffer, "\r\n%d : %d : %d : %d : %d\r\n", ADC1buffer[0], ADC1buffer[1], ADC1buffer[2], ADC1buffer[3], ADC1buffer[4]);
            USARTSend(buffer);
            FLAG_ECHO = 0;
        }
    }
}
/*
void DMA1_Channel1_IRQHandler (void)
{
  DMA_ClearITPendingBit (DMA_IT_TC);
  DMA_ITConfig (DMA1_Channel1, DMA_IT_TC, DISABLE);
  TempValue = ADC1Value[Counter];
  DMA_ITConfig (DMA1_Channel1, DMA_IT_TC, ENABLE);
}
*/
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
