#ifndef KALMAN_H_
#define KALMAN_H_

float atan2safe(float x, float y);
float filterFi1(float acc_2, float acc_3, float gyro_1, float * fi1);
float filterFi2(float acc_1, float acc_3, float gyro_2, float * fi2);

#endif /* KALMAN_H_ */
