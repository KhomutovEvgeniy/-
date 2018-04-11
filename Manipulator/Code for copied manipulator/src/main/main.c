#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

uint32_t a = 0;
void delay(uint32_t i) 
{
  volatile uint32_t j;
  for (j=0; j!= i * 5000; j++);
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

    
 GPIO_InitTypeDef PORT;
//Включаем порты А и С
RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA) , ENABLE);

  //Настраиваем ноги PA5 на выход. Там у нас висят светодиод
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
