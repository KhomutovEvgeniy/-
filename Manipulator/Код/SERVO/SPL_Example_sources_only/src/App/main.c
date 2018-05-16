#include <CMSIS/stm32f10x.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define SERVO_60 2100
#define SERVO_0 900

#define SERVO_MAX_ANGLE 60
#define SERVO_SHORTEST_PULSE 45

#define TIMER_PRESCALER 72
#define TIMER_PERIOD 20000

#define PORT_MANAGE GPIOB
#define PIN_MANAGE GPIO_Pin_10

/*
Задаем частоту работы таймера: 1 МГц, что соответствет периоду в 1 мкс.
Таймер считает с частотой 1 МГц, отсчет ведется каждые 1 мкс.
Для этого предделитель дожен быть равен 72. Ибо 72000000/72 = 1000000 = 1 МГц.

Необходимо задать период ШИМ в 20 мс, с частотой 50 Гц.
Что есть 20000 отрезков по 1 мкс.
Следовательно период 20 мс(50 Гц).

Ширина управляющего импульса от 900 мкс (0°) до 2100 мкс (60°)

Частота импульсов переменной ширины: 20 мс(50 Гц)

Диапазон вращения: 60`

На один градус поворота 2100-900/60 = 20 мкс

Известно: 

1500 мкс - 0`
1950 мкс - 45` по часовой
1050 мкс - 45` против часовой

Класть в регистр сравнения число, равное 45(900 мкс / 20 мкс) + задаваемый угол.

Регистр сравнения лучше обновлять строго в момент окончания периода во избежание дёргания сервы

Используется TIM2 Channel 3
*/

// типовой вариант функции assert
void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
  {
	;
  }
}

volatile uint32_t TimingDelay;

void Delay(uint32_t Time) // функция временной задержки
{
	TimingDelay = Time;
	
	while(TimingDelay != 0);
}


void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0)
		{
			TimingDelay--;
		}
}

void SysTick_Handler(void)                // system timer interrupt handler
{
	TimingDelay_Decrement();
}

void Set_Pin()                                               // настройка подключения периферии
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);     //Тактируем порт B
	
  GPIO_InitTypeDef gpioStruct;                               // структура сигнального пина
  GPIO_StructInit(&gpioStruct);
	gpioStruct.GPIO_Speed = GPIO_Speed_2MHz;
	gpioStruct.GPIO_Mode = GPIO_Mode_AF_PP;                    //Будем использовать альтернативный режим а не обычный GPIO
	gpioStruct.GPIO_Pin = PIN_MANAGE;                          // Настроим ногу (PA1) к которой подключен сервопривод
  GPIO_Init(PORT_MANAGE, &gpioStruct);
}

void Set_Timer()
{
	// настройка базового таймера
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);       //Тактируем Таймер 2
                                                             // Частота счёта, отсчет ведется каждые - 1 мкс, период ШИМ - 20 мс(50 Гц)
	TIM_TimeBaseInitTypeDef Tim;                               // создаем экземпляр структуры, которая задает частоту через предделитель, период и направление счета
  TIM_TimeBaseStructInit(&Tim);                              // заполняем структуру значениями по умолчанию
	Tim.TIM_Prescaler = TIMER_PRESCALER;                       // предделитель для таймера, значение, досчитав до которго, таймер сгенерирует прерывание. За 10 Кгц один тик
	Tim.TIM_Period = TIMER_PERIOD;                                    // период ШИМ, такой, чтобы частота 50 Гц.
	TIM_TimeBaseInit(TIM2, &Tim);                              // инициализируем таймер
	
	// настройка ШИМа. Настройка канала.  начальное заполнение: 90 тиков (900 мкс)
	TIM_OCInitTypeDef TimPWM;                                  // конфигурация выхода таймера. Структура для генерации ШИМ
	TIM_OCStructInit(&TimPWM);
	TimPWM.TIM_OCMode = TIM_OCMode_PWM1;                       // конфигурируем выход таймера, режим PWM1. Помимо PWM1 есть еще PWM2. Это всего лишь разные режимы ШИМ – с выравниванием по границе и по центру
	TimPWM.TIM_OutputState = TIM_OutputState_Enable;           // выход включен
	TimPWM.TIM_Pulse = SERVO_SHORTEST_PULSE;                   // чатсота ШИМ, заполнение, скважность.
	TIM_OC3Init(TIM2, &TimPWM);                                // заносим данные во 3ий канал
	
	// разрешение прерываний и работы таймера
	TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);                  // режим генерации прерывания по обновлению, переполнению
	TIM_Cmd(TIM2, ENABLE);                                     // запуск таймера
	
	// настройка системного таймера
	__disable_irq();
	SysTick_Config(SystemCoreClock / 1000);
	__enable_irq();
}

// volatile uint16_t pulse = 0;

// // Функция устанавливает позицию вала (в градусах)
// void Angle(int angle) 
// {
//   pulse = angle*(SERVO_60 - SERVO_0) /180;
// 	TIM_SetCompare2(TIM2, pulse);
// // 	TIM2->CCR2 = pulse + SERVO_0;
// }

// double old_angle1 = 0;
// void Angle_slow(double angle) 
// {
// 	  int i = 0;
// 		for(i = old_angle1; i <= angle; i=i+5)
// 			{
// 				pulse = i * (SERVO_180-SERVO_0) / 180;//пересчет угла в пульс
// 				Tim2_config(pulse);
// // 				delay();
// 			}

// 		for(i = old_angle1; i >= angle; i=i-5)
// 			{
// 				pulse = i * (SERVO_180-SERVO_0) / 180;//пересчет угла в пульс
// 				Tim2_config(pulse);
// // 				delay();
// 			}
// 	old_angle1 = angle;
// }

// void set_pos(uint8_t pos)
// {

// 	uint32_t tmp=(SERVO_180 - SERVO_0) /180 ;

// 	TIM2->CCR2 = SERVO_0 + tmp * pos;

// }

int main()
{
	Set_Pin();
	Set_Timer();
	int delay = 50;	 
	while(1)
	{
		TIM2->CCR3 = 900;
	}
}
	

