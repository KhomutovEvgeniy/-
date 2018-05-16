#include <CMSIS/stm32f10x.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/*
К выводу РС8 подключен светодиод. Необходимо плавно изменять яркость горения светодиода от минимума до максимума и обратно до минимума. 
Период изменения яркости - 2 секунды.
Для этого необходимо генерировать ШИМ; причем коэффициент заполнения должен плавно (например, линейно) меняться от 0 до максимума и обратно до минимума. 
Частота ШИМ должна быть не менее 100 Герц, чтобы изменение коэффициента заполнения воспринимались человеческим глазом как изменение якрости.

Задаем частоту работы таймера: 1 МГц, что соответствет периоду в 1 мкс.
Таймер считает с частотой 1 МГц, отсчет ведется каждые 1 мкс.
Для этого предделитель дожен быть равен 72. Ибо 72000000/72 = 1000000 = 1 МГц.

Задаю период ШИМ в 10 мс(частота 100 Гц).
Период(T) = PERIOD/(72 МГц/PRESCALER).

Используется TIM2 CHANNEL 2.

Использую PA1 вместо PC8.

Для организации временной задержки использую TIM3.
Таймер перезагружаетяс через 10 мс.
*/
 
#define PWM_PRESCALER 72  																	// значение предделителя для таймера, генерирующего ШИМ. 72 000 000/72 = 1 МГц.
#define PWM_PERIOD 10000 																		// значения периода для таймера, генерирующего ШИМ. 1 000 000/ 10000 = 100 Гц.

#define TIMER_PRESCALER 720                                 // значения предделителя для таймера, обеспечивающего задержку. 72 000 000/720 = 100 000 Гц.
#define TIMER_PERIOD 10000                  							  // значения периода(количество тиков) для таймера, обеспечивающего задержку. 100 000/10 000 = 10 Гц. 10 Гц = 0.1 с

#define PORT_MANAGE_LED GPIOA                               // порт для подключения светодиода.
#define PIN_MANAGE_LED GPIO_Pin_1                           // пин для подключения светодиода.
 
// типовой вариант функции assert
void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
  {
	;
  }
}

void Set_Pin()                                               // настройка подключения периферии.
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);     // тактирую порт А.
	
  GPIO_InitTypeDef gpioStruct;                               // структура сигнального пина.
  GPIO_StructInit(&gpioStruct);
	gpioStruct.GPIO_Speed = GPIO_Speed_2MHz;
	gpioStruct.GPIO_Mode = GPIO_Mode_AF_PP;                    // буду использовать альтернативный режим, а не обычный GPIO.
	gpioStruct.GPIO_Pin = PIN_MANAGE_LED;                      // настраиваю ногу (PА1) к которой подключен светодиод.
  GPIO_Init(PORT_MANAGE_LED, &gpioStruct);          				 // инициализирую структуру
}

void Set_Timer()
{
	// настройка базового таймера
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);       // тактирую таймер 2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);       // тактирую таймер 3
                                                             
	TIM_TimeBaseInitTypeDef Tim;                               // создаю экземпляр структуры, которая задает частоту через предделитель, период и направление счета.
  TIM_TimeBaseStructInit(&Tim);                              // заполняю структуру значениями по умолчанию
	Tim.TIM_Prescaler = PWM_PRESCALER;                         // предделитель для таймера
	Tim.TIM_CounterMode = TIM_CounterMode_Up;                  // направление счета
	Tim.TIM_Period = PWM_PERIOD;                               // период 
	TIM_TimeBaseInit(TIM2, &Tim);                              // инициализирую таймер
	
	Tim.TIM_Prescaler = TIMER_PRESCALER;                                   
	Tim.TIM_Period = TIMER_PERIOD;                          
	TIM_TimeBaseInit(TIM3, &Tim);                              
	
	// настройка канала ШИМа
	TIM_OCInitTypeDef TimPWM;                                  // конфигурация выхода канала таймера. Структура для генерации ШИМ.
	TIM_OCStructInit(&TimPWM);
	TimPWM.TIM_OCMode = TIM_OCMode_PWM1;                       // конфигурирую выход таймера, режим PWM1 
	TimPWM.TIM_OutputState = TIM_OutputState_Enable;           // выход включен
	TIM_OC2Init(TIM2, &TimPWM);                                // заношу данные во 2ой канал
	
	TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);                  // режим генерации прерывания по обновлению, переполнению
	TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);   
	TIM_Cmd(TIM2, ENABLE);                                     // запуск таймера
} 

uint16_t brightness[] = {0,1000,2000,3000,4000,5000,6000,7000,8000,9000,9999}; // массив значений для заполнения ШИМа.

void Delay()                           											 // функция, обеспечивающая задержку в 0.1 с.
{
	TIM_Cmd(TIM3, ENABLE);         														 // в момент вызова произвести запуск таймера
	while((TIM3->SR & TIM_SR_UIF)==0)                          //дождаться конца задержки
	{
	}		
  TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	               //сбросить флаг
}

void Bright()                                                // функция изменения яркости светодиодов при управлении ШИМом
{
	for (int i = 0; i <= 10; i++)                              // яркость увеличивается от 0 до максимального значения
	{
		TIM_SetCompare2(TIM2,brightness[i]);
		Delay();      																									
	}
	for (int i = 9; i >= 1; i--)                               // яркость уменьшается от максимального значения до 0
	{
		TIM2->CCR2 = brightness[i];
		Delay();      
	}
}

int main()
{
	Set_Pin();
	Set_Timer();
	while(1)
	{
		Bright();
	}
	return 0;
}