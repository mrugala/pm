#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>
#include <misc.h>
#include "comm.h"

#ifndef __IO
#define __IO volatile
#define __I  const volatile
#define __O  volatile
#endif

__IO uint8_t UARTStringReceived; // flaga sygnalizuj¹ca odebranie stringa
__IO uint8_t RotDataReceived;
__IO uint8_t GravStringReceived;
__IO uint8_t AccelDataReceived;
__IO uint8_t GyroDataReceived;
__IO uint8_t bytesReceived;
char UARTdataRx[UART_BUFFER_SIZE+1];
char DMPdataRx[DMP_BUFFER_SIZE];
char GravStringRx[256];

#define DATA_MSG 	1
#define DEBG_MSG 	3
#define ACCEL_DATA 	0
#define GYRO_DATA 	1
#define ROT_DATA 	5

void initUSART1(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	/*
	 * USART1 i 6 - APB2, reszta APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting

	/*
	 * konfiguracja przerwania
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

void initUSART3(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	/*
	 * USART1 i 6 - APB2, reszta APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART3, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting

	/*
	 * konfiguracja przerwania
	 */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART3, ENABLE);
}

void printToUSART(USART_TypeDef* USARTx, __IO char *str)
{
	while(*str)
	{
		// oczekiwanie na opró¿nienie rejestru wysy³ania
		while( !(USARTx->SR & 0x00000080) );

		USART_SendData(USARTx, *str);
		*str++;
	}
}

void clearBuffer(__IO char *str)
{
	while(*str)
	{
		str[0] = 0;
		*str++;
	}
}

/*
 * HANDLERY
 */

void USART1_IRQHandler(void){

	/*
	 * sprawdzenie czy flaga przerwania Rx jest ustawiona
	 */
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){

		static uint8_t msg_type = 0;				// typ wiadomosci
		static uint8_t data_type = 0;				// typ danych
		static uint8_t cnt = 0; 					// pozycja w buforze
		static uint8_t len = 0;						// d³ugosc stringa
		char c = USART_ReceiveData(USART1); 		// bufor znaku odebranego
		
		if(c == '$') 								// znak pocz¹tku ramki
		{
			cnt = 0;
		}
		else if( cnt == 1 ) 								// znak typu wiadomosci
		{
			msg_type = c;
		}
		else
		{
			switch(msg_type)
			{
			case DEBG_MSG:
				if( cnt < (DMP_BUFFER_SIZE - 2) && !GravStringReceived )
				{
					if(c == '\n')
					{
						GravStringRx[len] = '\0';
						GravStringReceived = 1;
						len = 0;
					}
					else
					{
						if(c == '\r')
						{
							GravStringRx[len] = '\0';
						}
						else
						{
							GravStringRx[len] = c;
						}
						++len;
					}
				}
				break;
			case DATA_MSG:
				if(cnt == 2)
					data_type = c;
				else if(data_type == ACCEL_DATA || data_type == GYRO_DATA || data_type == ROT_DATA )
				{
					DMPdataRx[cnt] = c;
					if(cnt == (DMP_BUFFER_SIZE - 1))
					{
						switch(data_type)
						{
						case ACCEL_DATA:
							AccelDataReceived = 1;
							break;
						case GYRO_DATA:
							GyroDataReceived = 1;
							break;
						case ROT_DATA:
							RotDataReceived = 1;
							break;
						}
					}
				}
				break;
			default:
				break;
			}
		}
		cnt++;
	}
}

void USART3_IRQHandler(void){

	/*
	 * sprawdzenie czy flaga przerwania Rx jest ustawiona
	 */
	if( USART_GetITStatus(USART3, USART_IT_RXNE) ){

		static uint8_t cnt = 0; 	// d³ugoœæ stringa
		char c = USART_ReceiveData(USART3); 		// bufor znaku odebranego

		/*
		 * sprawdzenie czy limit znaków nie zosta³ osi¹gniêty
		 */
		if( (c != '\n') && (cnt < UART_BUFFER_SIZE) )
		{
			UARTdataRx[cnt] = c;
			cnt++;
		}
		else
		{
			// w przeciwnym wypadku nastêpuje zerowanie licznika i ustawienie flagi (oraz zapisanie ostatniego znaku)
			if(c != '\n')
				UARTdataRx[cnt] = c;
			bytesReceived = cnt;
			cnt = 0;
			UARTStringReceived = 1;
		}
	}
}
