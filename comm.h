/****************************************************************
 * Komunikacja USART				                            *
 *                                                              *
 * Autor: Jêdrzej Mruga³a                                       *
 *                                                              *
 ****************************************************************/

#ifndef COMM_H_
#define COMM_H_

#include "stdint.h"
#include "stm32f4xx.h"

// USART2
#define UART_BUFFER_SIZE 18 	// 14 bajtów danych + CRC + znaki pocz¹tku/koñca ramki
#define DMP_BUFFER_SIZE 23 	// '$' + typ danuch + dane + "\r\n"
extern __IO uint8_t UARTStringReceived;
extern __IO uint8_t RotDataReceived;
extern __IO uint8_t GravStringReceived;
extern __IO uint8_t AccelDataReceived;
extern __IO uint8_t GyroDataReceived;
extern __IO uint8_t bytesReceived;

#define ROT_DATA_LENGTH		2
#define ACCEL_DATA_LENGTH	4
#define GYRO_DATA_LENGTH	4
#define DATA_OFFSET			3

extern char UARTdataRx[UART_BUFFER_SIZE+1];
extern char DMPdataRx[DMP_BUFFER_SIZE];
extern char GravStringRx[256];

void USART1_IRQHandler(void);		// USART2 interrupt handler

void initUSART1(uint32_t baudrate);
void initUSART3(uint32_t baudrate);
void printToUSART(USART_TypeDef* USARTx, __IO char *str);

void clearBuffer(__IO char *str);

#endif /* COMM_H_ */
