#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "misc.h"
#include "stdio.h"

#define Resistor_PORT1 GPIOA
#define Resistor_PORT2 GPIOC
#define USART1_PORT_Tx GPIOA
#define USART1_PORT_Rx GPIOA
#define RES_1_PIN GPIO_Pin_0
#define RES_2_PIN GPIO_Pin_1
#define RES_3_PIN GPIO_Pin_4
#define RES_4_PIN GPIO_Pin_1
#define RES_5_PIN GPIO_Pin_0

// Преобразованный массив показаний для отправки по USART1
static volatile uint8_t USART_buffer[] = {0, 0, 0};
// Массив (буфер) показаний резисторов, считывающихся с ADC
static volatile uint16_t ADC_buffer[] = {0, 0, 0};
// static volatile uint32_t timeStampMs = 0; Необ для задержки

// Local functions prototype
void usart_init (void);
void dma_usart_init (void);
void adc_init (void);
void dma_adc_init (void);
void array_for_usart(void);
void DMA1_Channel1_IRQHandler(void);

void usart_init (void)
{
  /* Enable USART1 and GPIOA PORT */
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE); //Разрешить тактирование ножки Tx
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE); //Разрешить тактирование USART1
  /* Configure the GPIOs */
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART1_PORT_Tx, &GPIO_InitStructure);

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
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE); //Enable USART1
}

void dma_usart_init (void)
{
  // Разрешение тактирвоания модуля DMA
  RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1, ENABLE); 
  
  // Настройка модуля DMA1 - из буфера на USART1
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) USART_buffer;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART1->DR);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = sizeof(USART_buffer);
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);	//4 канал - Tx USART1
  
  DMA_Cmd(DMA1_Channel4, ENABLE); //Включаем прямой доступ к памяти
  // Активируем передачу в последовательный порт по запросу DMA 
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);   
}

void adc_init (void)
{
  // Разрешение тактирования портов А и С (К ним подключены резисторы)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); 
  // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE); Отключен т.к. у имеющегося манипулятора только 3 степени подвижности
  // Разрешение тактирования модуля АЦП1
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_ADC1, ENABLE);
  // Предделитель АЦП (default)
//  RCC_ADCCLKConfig(RCC_PCLK2_Div2);
  // Настройка АЦП (первые три степени свободы)
  GPIO_InitTypeDef GPIO_InitStructure1;
  GPIO_InitStructure1.GPIO_Pin = RES_1_PIN | RES_2_PIN | RES_3_PIN;
  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init (Resistor_PORT1, &GPIO_InitStructure1);
/*
  // Настройка АЦП (крайние две степени свободы)
  GPIO_InitTypeDef GPIO_InitStructure2;
  GPIO_InitStructure2.GPIO_Pin = RES_4_PIN || RES_5_PIN;
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init (Resistor_PORT2, &GPIO_InitStructure2);
*/
  // Настройка АЦП
  ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // Независимый режим (т.к. используем 1 АЦП)
  ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Сканирующий режим (т.к. сигнал будем снимать не с одной ножки)
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // Циклическое сканирование
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // Источник запуска АЦП - нет  
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // Выравнимание битов по правому краю
  ADC_InitStructure.ADC_NbrOfChannel = 3; // Количество каналов сканирования
  
  // Определение регулярной группы сканирования АЦП  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5); // Время выборки для канала - 239.5 циклов
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_239Cycles5);
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 4, ADC_SampleTime_239Cycles5);
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 5, ADC_SampleTime_239Cycles5);
  ADC_Init (ADC1, &ADC_InitStructure);
  
  // Запуск АЦП
  ADC_Cmd(ADC1, ENABLE);
  
  // Включение DMA с ADC
  ADC_DMACmd(ADC1, ENABLE); 
  
  // Калибровка АЦП
  while (ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration (ADC1);
  while (ADC_GetCalibrationStatus(ADC1));
  // Запуск преобразований ADC
  ADC_SoftwareStartConvCmd (ADC1, ENABLE); 
}

void dma_adc_init ()
{
  // Разрешение тактирвоания модуля DMA
  RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1, ENABLE); 
  // Настройка модуля DMA1 - режим работы: показание с ADC1 в буфер
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC_buffer;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(ADC1->DR);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = sizeof(ADC_buffer);
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);	//1 канал - ADC1
  // Активация передачи в последовательный порт по запросу DMA 
  DMA_Cmd(DMA1_Channel1, ENABLE); //Включаем прямой доступ к памяти
  // Включение прерывания DMA по завершении цикла обработки
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);  
}
// Преобразование показаний, полученных с АЦП, для отправки по USART 
// в этом случае на выходе не потребуется из байтиков лепить большие оригинальные значения
void array_for_usart(void)
{
  float temp = 0;
  for (int i = 0; i < 3; i++)
  {
    // alfa-betta filter (alfa = 0.3)
    temp = USART_buffer[i] * (1 - 0.3) + 0.3 * (ADC_buffer[i] >> 4);
    USART_buffer[i] = temp;
  }
}

// Обработчик прерывания по событию - DMA1 записал показания всех 3-x резисторов в буфер
void DMA1_Channel1_IRQHandler(void)
{   
  array_for_usart();
  // Сбрасывание флага прерывания
 // DMA_ClearITPendingBit(DMA1_IT_TC1);
}

int main(void)
{
  adc_init();
  dma_adc_init();
  usart_init();
  dma_usart_init();  
  while (1)
  {   
    //USART_SendData(USART1, 50);
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