/****************************************************************
 * Funkcje inicjalizacji peryferiów                             *
 *                                                              *
 * Autor: Jêdrzej Mruga³a                                       *
 *                                                              *
 ****************************************************************/

#ifndef PERIPH_H_
#define PERIPH_H_

#include "stdint.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"

// Start zegara
void waitForHSEStartUp();

// Systick
extern __IO uint32_t time;
extern __IO uint32_t delay;

#define DelayTime()					(time)
#define DelaySetTime(t)				(time = (t))

void SysTick_Handler(void); 		// systick interrupt handler

void initSysTick();
uint8_t Tickms(__IO uint32_t t);
void Delatms(__IO uint32_t t);
void Delayus(__IO uint32_t t);

// Timer 1,3,12 - tryb PWM
void Timer4Init(int period);
void PWM4Ch1Init(int Duty);
void PWM4Ch2Init(int Duty);
void PWM4Ch3Init(int Duty);
void PWM4Ch4Init(int Duty);

void Timer3Init(int period);
void PWM3Ch1Init(int Duty);
void PWM3Ch2Init(int Duty);
void PWM3Ch3Init(int Duty);

void Timer12Init(int period);
void PWM12Ch1Init(int Duty);
void PWM12Ch2Init(int Duty);

// LEDy w trybie sterowania PWM
void PWMLEDInit();

// LEDy 1-7 w trybie sterowania programowego PE0-6
void initLEDs();
#define SetLED(LED) 		(GPIOD->BSRRL = (1<<(LED)-1))
#define ResetLED(LED) 		(GPIOD->BSRRH = (1<<(LED)-1))
#define ToggleLED(LED)		(GPIOD->ODR ^= (1<<(LED)-1))
void initServoPins();

void initADC();
void setADCaddr(uint8_t addr);

// przyciski PD9-15
void initButtons(void);
uint8_t ButtonRisingEdge(uint8_t buttonNumber);

// przycisk USER
void initUserButton(void);
uint8_t UserButton(void);
uint8_t toggleUserButton(void);
uint8_t UserButtonRisingEdge(void);

#endif /* PERIPH_H_ */
