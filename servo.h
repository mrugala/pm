#ifndef SERVO_H_
#define SERVO_H_

#include "stdint.h"

typedef enum SERVO
{ 	FT_S148,
	HS_55,
	HS_645MG,
	HS_485HB,
	HD_1501MG,
	HD_1201MG
} servo;

void initServos();

int AngleToDuty(float angle, servo type);

void Move(uint8_t chan, float angle, servo type);

#endif /* SERVO_H_ */
