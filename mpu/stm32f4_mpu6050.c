#include "stm32f4_mpu6050.h"
#include "comm.h"

MPU6050_Result_t initMPU6050(MPU6050_Config_t config) {
	uint8_t temp;
//	char str[128];
	
	/* Initialize I2C */
	TM_I2C_Init(MPU6050_I2C, MPU6050_I2C_PINSPACK, MPU6050_I2C_CLOCK);
	
	/* Check if device is connected */
	if (!TM_I2C_IsDeviceConnected(MPU6050_I2C, config.Address)) {
		/* Return error */
		return MPU6050_Result_DeviceNotConnected;
	}
	
	/* Check who I am */
	if (TM_I2C_Read(MPU6050_I2C, config.Address, MPU6050_WHO_AM_I) != MPU6050_I_AM) {
		/* Return error */
		return MPU6050_Result_DeviceInvalid;
	}
	
	/* Wakeup MPU6050 */
	TM_I2C_Write(MPU6050_I2C, config.Address, MPU6050_PWR_MGMT_1, 0x00);
	
	/* Config accelerometer */
	temp = TM_I2C_Read(MPU6050_I2C, config.Address, MPU6050_ACCEL_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)(config.accelRange << 3);
	TM_I2C_Write(MPU6050_I2C, config.Address, MPU6050_ACCEL_CONFIG, temp);
//	sprintf(str, "accelconf 0x%x\n", temp);
//	printToUSART(USART3,str);
	
	/* Config gyroscope */
	temp = TM_I2C_Read(MPU6050_I2C, config.Address, MPU6050_GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)(config.gyroRange << 3);
//	sprintf(str, "gyroconf 0x%x\n", temp);
//	printToUSART(USART3,str);
	TM_I2C_Write(MPU6050_I2C, config.Address, MPU6050_GYRO_CONFIG, temp);
	
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Data_t readMPU6050Accelerometer(uint8_t addr) {
	MPU6050_Data_t MPU6050data;
	uint8_t data[6];
	
	TM_I2C_ReadMulti(MPU6050_I2C, addr, MPU6050_ACCEL_XOUT_H, data, 6);
	
	MPU6050data.AccelX = (int16_t)(data[0] << 8 | data[1]) / ACCEL_RANGE;
	MPU6050data.AccelY = (int16_t)(data[2] << 8 | data[3]) / ACCEL_RANGE;
	MPU6050data.AccelZ = (int16_t)(data[4] << 8 | data[5]) / ACCEL_RANGE;
	
	return MPU6050data;
}

MPU6050_Data_t readMPU6050Gyroscope(uint8_t addr) {
	MPU6050_Data_t MPU6050data;
	uint8_t data[6];
	
	TM_I2C_ReadMulti(MPU6050_I2C, addr, MPU6050_GYRO_XOUT_H, data, 6);
	
	MPU6050data.GyroX = (int16_t)(data[0] << 8 | data[1]) / GYRO_RANGE;
	MPU6050data.GyroY = (int16_t)(data[2] << 8 | data[3]) / GYRO_RANGE;
	MPU6050data.GyroZ = (int16_t)(data[4] << 8 | data[5]) / GYRO_RANGE;

	return MPU6050data;
}

MPU6050_Data_t readMPU6050Temperature(uint8_t addr) {
	MPU6050_Data_t MPU6050data;
	uint8_t data[2];
	int16_t temp;
	
	TM_I2C_ReadMulti(MPU6050_I2C, addr, MPU6050_TEMP_OUT_H, data, 2);
	
	temp = (data[0] << 8 | data[1]);
	MPU6050data.Temp = (float)((int16_t)temp / (float)340.0 + (float)35.0);
	
	return MPU6050data;
}

MPU6050_Data_t readMPU6050Data(uint8_t addr) {
	MPU6050_Data_t MPU6050data;
	uint8_t data[14];
	int16_t temp;
	
	TM_I2C_ReadMulti(MPU6050_I2C, addr, MPU6050_ACCEL_XOUT_H, data, 14);
	
	MPU6050data.AccelX = (int16_t)(data[0] << 8 | data[1]) / ACCEL_RANGE *2;
	MPU6050data.AccelY = (int16_t)(data[2] << 8 | data[3]) / ACCEL_RANGE *2;
	MPU6050data.AccelZ = (int16_t)(data[4] << 8 | data[5]) / ACCEL_RANGE *2;

	temp = (data[6] << 8 | data[7]);
	MPU6050data.Temp = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	MPU6050data.GyroX = (int16_t)(data[8] << 8 | data[9]) / GYRO_RANGE *2;
	MPU6050data.GyroY = (int16_t)(data[10] << 8 | data[11]) / GYRO_RANGE *2;
	MPU6050data.GyroZ = (int16_t)(data[12] << 8 | data[13]) / GYRO_RANGE *2;

	return MPU6050data;
}
