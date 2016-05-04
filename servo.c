#include <stm32f4xx_tim.h>
#include "servo.h"
#include "periph.h"

int AngleToDuty (float angle, servo type)
{
	switch(type)
	{
		case FT_S148:
			return -9.8889 * angle + 1440;
		case HS_55:
			return 0.0068 * angle * angle + 10.278 * angle + 1500;
		case HS_645MG:
//			return 0.0068 * angle * angle + 10.278 * angle + 1500;
			return (int)(10 * angle + 1500);
		case HS_485HB:
			return 0.0068 * angle * angle + 10.278 * angle + 1500;
		case HD_1201MG:
			return (int)(7.777778 * angle + 1500);
		case HD_1501MG:
			return 0.0068 * angle * angle + 10.278 * angle + 1500;
		default:
			return 1500;
	}
}

void Move (uint8_t chan, float angle, servo type)
{
	switch(chan)
	{
		case 1:
			TIM_SetCompare1(TIM12, AngleToDuty(angle, type));
			break;

		case 2:
			TIM_SetCompare2(TIM12, AngleToDuty(angle, type));
			break;

		case 3:
			TIM_SetCompare1(TIM3, AngleToDuty(angle, type));
			break;

		case 4:
			TIM_SetCompare2(TIM3, AngleToDuty(angle, type));
			break;

		case 5:
			TIM_SetCompare3(TIM3, AngleToDuty(angle, type));
			break;

		case 6:
			TIM_SetCompare1(TIM4, AngleToDuty(angle, type));
			break;

		case 7:
			TIM_SetCompare2(TIM4, AngleToDuty(angle, type));
			break;

		case 8:
			TIM_SetCompare3(TIM4, AngleToDuty(angle, type));
			break;

		case 9:
			TIM_SetCompare4(TIM4, AngleToDuty(angle, type));
			break;

		case 10:
			TIM_SetCompare1(TIM4, AngleToDuty(angle, type));
			break;

		default:
			break;
	}
}

void initServos()
{
	Timer4Init(20000);
	Timer3Init(20000);
	Timer12Init(20000);

	PWM12Ch1Init(AngleToDuty(0.0, HD_1501MG));  	// servo przy podstawie (obrót wokó³ osi pionowej)
	PWM12Ch2Init(AngleToDuty(0.0, HD_1501MG));  	// servo przy podstawie (obrót wokó³ osi poziomej)
	PWM3Ch1Init(AngleToDuty(90.0, HD_1201MG));  	// serwo na przegubie 3
	PWM3Ch2Init(AngleToDuty(0.0, HD_1201MG));    	// servo na przegubie 4
	PWM3Ch3Init(AngleToDuty(0.0, HS_645MG));		// servo na przegubie 5 (oœ wzd³u¿ ogniwa)
	PWM4Ch1Init(AngleToDuty(0.0, HS_485HB));		// servo na przegubie 6
	PWM4Ch2Init(AngleToDuty(45.0, HS_55));			// chwytak 1
	PWM4Ch3Init(AngleToDuty(-45.0, HS_55));			// chwytak 2
}
