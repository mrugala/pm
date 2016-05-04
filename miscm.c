#include <stdio.h>
#include "tm_stm32f4_hd44780.h"
#include "miscm.h"
#include "periph.h"

//Custom character
uint8_t customChar[] = {
	0x00,    // xxx 00000
	0x0A,    // xxx 01010
	0x0A,    // xxx 01010
	0x00,    // xxx 00000
	0x11,    // xxx 10001
	0x11,    // xxx 10001
	0x0E,    // xxx 01110
	0x00     // xxx 00000
};

//char tmp[256];
//float tmp1;

void HDStartSeq(void)
{
	TM_HD44780_CreateChar(0, &customChar[0]);

	TM_HD44780_Puts(0, 0, "Servo Driver\n\rSTM32F4");

	Delayms(3000);

	TM_HD44780_Clear();

	TM_HD44780_Puts(3, 0, "MANY FUN!");
	TM_HD44780_Puts(6, 1, "WOW!");

	Delayms(1000);

	TM_HD44780_PutCustom(0, 0, 0);
	TM_HD44780_PutCustom(14, 0, 0);
	TM_HD44780_PutCustom(1, 1, 0);
	TM_HD44780_PutCustom(15, 1, 0);

	Delayms(1000);

	TM_HD44780_Clear();
}
