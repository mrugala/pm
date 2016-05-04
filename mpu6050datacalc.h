#ifndef MPUDATACALC_H_
#define MPUDATACALC_H_

#include "stm32f4_mpu6050.h"
#include "comm.h"

void printData(MPU6050_Data_t MPU6050Data) {
	char str[120];

	sprintf(str,
			"Accel\nX:%3.4f\nY:%3.4f\nZ:%3.4f\nGyro\nX:%3.4f\nY:%3.4f\nZ:%3.4f\nTemp\n%3.4f\n\n\n",
			MPU6050Data.AccelX,
			MPU6050Data.AccelY,
			MPU6050Data.AccelZ,
			MPU6050Data.GyroX,
			MPU6050Data.GyroY,
			MPU6050Data.GyroZ,
			MPU6050Data.Temp);

	printToUSART(USART1, str);
}

MPU6050_Data_t zeroData(MPU6050_Data_t data) {
	data.AccelX = 0;
	data.AccelY = 0;
	data.AccelZ = 0;
	data.GyroX = 0;
	data.GyroY = 0;
	data.GyroZ = 0;
	data.Temp = 0;
	return data;
}

MPU6050_Data_t divideData(MPU6050_Data_t data, int denominator) {
	data.AccelX /= denominator;
	data.AccelY /= denominator;
	data.AccelZ /= denominator;
	data.GyroX /= denominator;
	data.GyroY /= denominator;
	data.GyroZ /= denominator;
	data.Temp /= denominator;
	return data;
}

MPU6050_Data_t addData(MPU6050_Data_t data1, MPU6050_Data_t data2) {
	data1.AccelX += data2.AccelX;
	data1.AccelY += data2.AccelY;
	data1.AccelZ += data2.AccelZ;
	data1.GyroX += data2.GyroX;
	data1.GyroY += data2.GyroY;
	data1.GyroZ += data2.GyroZ;
	data1.Temp += data2.Temp;
	return data1;
}

MPU6050_Data_t substractOffset(MPU6050_Data_t data, MPU6050_Data_t offset) {
	data.AccelX -= offset.AccelX;
	data.AccelY -= offset.AccelY;
	data.AccelZ -= offset.AccelZ - 1;
	data.GyroX -= offset.GyroX;
	data.GyroY -= offset.GyroY;
	data.GyroZ -= offset.GyroZ;
	//	data.Temp -= offset.Temp;

	return data;
}

#endif /* MPUDATACALC_H_ */
