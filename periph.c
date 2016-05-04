#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <misc.h>
#include "periph.h"
#include "stm32f4_adc.h"

#ifndef __IO
#define __IO volatile
#define __I  const volatile
#define __O  volatile
#endif

__IO uint32_t time;
__IO uint32_t delay;

void Delayus(__IO uint32_t t) {
	delay = t;

	while (delay != 0);
}

void Delayms(__IO uint32_t t) {
	delay = 1000 * t;

	while (delay != 0);
}

void initSysTick()
{
	if(SysTick_Config(SystemCoreClock / 1000000))
		while(1);
}

uint8_t Tickms(__IO uint32_t t)
{
	if(DelayTime() >= (1000 * t))
	{
		DelaySetTime(0);
		return 1;
	}
	else
	{
		return 0;
	}
}

void waitForHSEStartUp()
{
	RCC_HSEConfig(RCC_HSE_ON);

	while(!RCC_WaitForHSEStartUp());
}

/*
 * TIMER 4 servo 6,7,8,9
 */

void Timer4Init(int period)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;

    timerInitStructure.TIM_Prescaler = 84;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = period;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM4, &timerInitStructure);
    TIM_Cmd(TIM4, ENABLE);
}

void PWM4Ch1Init(int Duty)
{
	TIM_OCInitTypeDef outputChannelInit = {0,};

	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = Duty; //czas swiecenia diody zielonej
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &outputChannelInit);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
}

void PWM4Ch2Init(int Duty)
{
	TIM_OCInitTypeDef outputChannelInit = {0,};

	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = Duty; //czas swiecenia diody pomaranczowej
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC2Init(TIM4, &outputChannelInit);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
}

void PWM4Ch3Init(int Duty)
{
	TIM_OCInitTypeDef outputChannelInit = {0,};

	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = Duty; //czas swiecenia diody czerwonej
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM4, &outputChannelInit);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
}

void PWM4Ch4Init(int Duty)
{
	TIM_OCInitTypeDef outputChannelInit = {0,};

	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = Duty; //czas swiecenia diody niebieskiej
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC4Init(TIM4, &outputChannelInit);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
}

/*
 * TIMER 3 servo 3,4,5
 */

void Timer3Init(int period)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

//    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM_TimeBaseInitTypeDef timerInitStructure;

    timerInitStructure.TIM_Prescaler = 84;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = period;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM3, &timerInitStructure);
    TIM_Cmd(TIM3, ENABLE);
}

void PWM3Ch1Init(int Duty)
{
	TIM_OCInitTypeDef outputChannelInit = {0,};

	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = Duty; //czas swiecenia diody pomaranczowej
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &outputChannelInit);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
}

void PWM3Ch2Init(int Duty)
{
	TIM_OCInitTypeDef outputChannelInit = {0,};

	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = Duty; //czas swiecenia diody pomaranczowej
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC2Init(TIM3, &outputChannelInit);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
}

void PWM3Ch3Init(int Duty)
{
	TIM_OCInitTypeDef outputChannelInit = {0,};

	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = Duty; //czas swiecenia diody pomaranczowej
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM3, &outputChannelInit);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
}

/*
 * TIMER 12 servo 1,2
 */

void Timer12Init(int period)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

//    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;

    TIM_TimeBaseInitTypeDef timerInitStructure;

    timerInitStructure.TIM_Prescaler = 84;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = period;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM12, &timerInitStructure);
    TIM_Cmd(TIM12, ENABLE);
}

void PWM12Ch1Init(int Duty)
{
	TIM_OCInitTypeDef outputChannelInit = {0,};

	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = Duty;
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM12, &outputChannelInit);
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12);
}

void PWM12Ch2Init(int Duty)
{
	TIM_OCInitTypeDef outputChannelInit = {0,};

	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = Duty;
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC2Init(TIM12, &outputChannelInit);
	TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);
}

/*
 * koniec timerów
 */

void PWMLEDInit()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef gpioStructure;

    gpioStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioStructure.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOD, &gpioStructure);
}

void initServoPins()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef gpioStructure;

    gpioStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 ;
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioStructure.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOC, &gpioStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    gpioStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_14 | GPIO_Pin_15 ;
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioStructure.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOB, &gpioStructure);
}

/*
 * LEDy PE0-7
 */

void initLEDs()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef gpioStructure;

    gpioStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 ;
    gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOE, &gpioStructure);
}

void initADC()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpioStructure;

    gpioStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_5 ;
    gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOC, &gpioStructure);

    gpioStructure.GPIO_Pin = GPIO_Pin_1;
    gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &gpioStructure);

	TM_ADC_InitADC(ADC1);
/*	TM_ADC_InitADC(ADC2);
	TM_ADC_InitADC(ADC3);*/
	TM_ADC_Init(ADC1, TM_ADC_Channel_1);
	TM_ADC_Init(ADC1, TM_ADC_Channel_3);
	TM_ADC_Init(ADC1, TM_ADC_Channel_5);
	TM_ADC_Init(ADC1, TM_ADC_Channel_7);
}

void setADCaddr(uint8_t addr)
{
	if(addr & 0b1)
		GPIOC->BSRRL = GPIO_Pin_5;
	else
		GPIOC->BSRRH = GPIO_Pin_5;

	if(addr & 0b10)
		GPIOB->BSRRL = GPIO_Pin_1;
	else
		GPIOB->BSRRH = GPIO_Pin_1;

	if(addr & 0b100)
		GPIOC->BSRRL = GPIO_Pin_1;
	else
		GPIOC->BSRRH = GPIO_Pin_1;
}

void initUserButton()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpioStructure;

    gpioStructure.GPIO_Pin = GPIO_Pin_0 ;
    gpioStructure.GPIO_Mode = GPIO_Mode_IN;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpioStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOA, &gpioStructure);
}

void initButtons()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpioStructure;

    gpioStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
    gpioStructure.GPIO_Mode = GPIO_Mode_IN;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpioStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOD, &gpioStructure);
}

uint8_t UserButton()
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
}

uint8_t toggleUserButton()
{
	static uint8_t slope = 0;
	static uint8_t temp = 0;

	if (UserButton() & ~temp)
	{
		if (1 == slope)
			slope = 0;
		else
			slope = 1;
	}

	temp = UserButton();

	return slope;
}

uint8_t UserButtonRisingEdge()
{
	static uint8_t temp = 0;

	if (UserButton() & ~temp)
	{
		temp = UserButton();
		return 1;
	}
	else
	{
		temp = UserButton();
		return 0;
	}
}

uint8_t ButtonRisingEdge(uint8_t buttonNumber)
{
	static uint8_t temp[7] = { 0, 0, 0, 0, 0, 0, 0 };

	if(buttonNumber > 0 && buttonNumber <= 7)
	{
		if (GPIO_ReadInputDataBit(GPIOA, 1<<(buttonNumber+8)) & ~temp[buttonNumber])
		{
			temp[buttonNumber-1] = GPIO_ReadInputDataBit(GPIOA, 1<<(buttonNumber+8));
			return 1;
		}
		else
		{
			temp[buttonNumber-1] = GPIO_ReadInputDataBit(GPIOA, 1<<(buttonNumber+8));
			return 0;
		}
	}
	return 0;
}

/*
 * HANDLERY
 */

void SysTick_Handler(void)
{
	time++;

	if (delay != 0x00) {
		delay--;
	}
}
