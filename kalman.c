#include "kalman.h"
#include "mathm.h"
#include <math.h>

#define std_dev_v 5.0
#define std_dev_w 2.0

void matrix_2x2_mul_2x1(float * in1, float * in2, float * out);
void matrix_2x1_mul_1x1(float * in1, float * in2, float * out);
void matrix_2x1_add_2x1(float * in1, float * in2, float * out);
void matrix_2x2_mul_2x2(float * in1, float * in2, float * out);
void matrix_2x2_add_2x2(float * in1, float * in2, float * out);
void matrix_1x2_mul_2x1(float * in1, float * in2, float * out);
void matrix_1x2_mul_2x2(float * in1, float * in2, float * out);
void matrix_2x1_mul_1x2(float * in1, float * in2, float * out);
void matrix_2x2_sub_2x2(float * in1, float * in2, float * out);
void matrix_2x2_trans(float * in, float * out);

float atan2safe(float x, float y) {
	float ret;

	if (x < 0 && y == 0)
		ret = -90.0;
	else if (x > 0 && y == 0)
		ret = 90.0;
	else if (x == 0 && y == 0)
		ret = 0.0;
	else
		ret = atan2(x, y) * 180 / PI;

	return ret;
}

float filterFi1(float acc_2, float acc_3, float gyro_1, float * fi1)
{
	static int i = 0;
	static float A[4] = { 1, -samplesForMean/1000, 0, 1}, B[2] = { samplesForMean/1000, 0 }, C[2] = { 1, 0 };
	static float V[4] = { std_dev_v*std_dev_v*samplesForMean/1000, 0, 0, std_dev_v*std_dev_v*samplesForMean/1000 }, W[1] = { std_dev_w*std_dev_w };
	static float P_pri[4], P_post[4] = { 1, 0, 0, 1 };
	float x_pri[2];
	float eps[1], S[1], K[2];
	float u[1], y[1];
	static float x_post[2] = { 0 };

	float Ax[2], Bu[2];
	float AP[4], AT[4], APAT[4];
	float Cx[1];
	float CP[2], CPCT[1];
	float PCT[2], S1[1];
	float Keps[2];
	float KS[2], KSKT[2];

	/* warunki pocz¹tkowe */
	if(i == 0)
	{
		x_post[0] = atan2safe(acc_2, acc_3);
		x_post[1] = gyro_1;
		i = 1;
	}

	/* x(t+1|t) = Ax(t|t) + Bu(t) */
	u[0] = gyro_1;
	matrix_2x2_mul_2x1(A, x_post, Ax);
	matrix_2x1_mul_1x1(B, u, Bu);
	matrix_2x1_add_2x1(Ax, Bu, x_pri);

	/* P(t+1|t) = AP(t|t)A^T + V */
	matrix_2x2_mul_2x2(A, P_post, AP);
	matrix_2x2_trans(A, AT);
	matrix_2x2_mul_2x2(AP, AT, APAT);
	matrix_2x2_add_2x2(APAT, V, P_pri);

	/* eps(t) = y(t) - Cx(t|t-1) */
	y[0] = atan2safe(acc_2, acc_3);
	*fi1 = y[0];

	matrix_1x2_mul_2x1(C, x_pri, Cx);
	eps[0] = y[0] - Cx[0];

	/* S(t) = CP(t|t-1)C^T + W */
	matrix_1x2_mul_2x2(C, P_pri, CP);
	matrix_1x2_mul_2x1(CP, C, CPCT);
	S[0] = CPCT[0] + W[0];

	/* K(t) = P(t|t-1)C^TS(t)^-1 */
	matrix_2x2_mul_2x1(P_pri, C, PCT);
	S1[0] = 1/S[0];
	matrix_2x1_mul_1x1(PCT, S1, K);

	/* x(t|t) = x(t|t-1) + K(t)eps(t) */
	matrix_2x1_mul_1x1(K, eps, Keps);
	matrix_2x1_add_2x1(x_pri, Keps, x_post);

	/* P(t|t) = P(t|t-1) - K(t)S(t)K(t)^T */
	matrix_2x1_mul_1x1(K, S, KS);
	matrix_2x1_mul_1x2(KS, K, KSKT);
	matrix_2x2_sub_2x2(P_pri, KSKT, P_post);

	return x_post[0];
}

float filterFi2(float acc_1, float acc_3, float gyro_2, float * fi2)
{
	static int i = 0;
	static float A[4] = { 1, -samplesForMean/1000, 0, 1}, B[2] = { samplesForMean/1000, 0 }, C[2] = { 1, 0 };
	static float V[4] = { std_dev_v*std_dev_v*samplesForMean/1000, 0, 0, std_dev_v*std_dev_v*samplesForMean/1000 }, W[1] = { std_dev_w*std_dev_w };
	static float P_pri[4], P_post[4] = { 1, 0, 0, 1 };
	float x_pri[2];
	float eps[1], S[1], K[2];
	float u[1], y[1];
	static float x_post[2] = { 0 };

	float Ax[2], Bu[2];
	float AP[4], AT[4], APAT[4];
	float Cx[1];
	float CP[2], CPCT[1];
	float PCT[2], S1[1];
	float Keps[2];
	float KS[2], KSKT[2];

	/* warunki pocz¹tkowe */
	if(i == 0)
	{
		x_post[0] = atan2safe(acc_1, acc_3);
		x_post[1] = gyro_2;
		i = 1;
	}

	/* x(t+1|t) = Ax(t|t) + Bu(t) */
	u[0] = gyro_2;
	matrix_2x2_mul_2x1(A, x_post, Ax);
	matrix_2x1_mul_1x1(B, u, Bu);
	matrix_2x1_add_2x1(Ax, Bu, x_pri);

	/* P(t+1|t) = AP(t|t)A^T + V */
	matrix_2x2_mul_2x2(A, P_post, AP);
	matrix_2x2_trans(A, AT);
	matrix_2x2_mul_2x2(AP, AT, APAT);
	matrix_2x2_add_2x2(APAT, V, P_pri);

	/* eps(t) = y(t) - Cx(t|t-1) */
	y[0] = atan2safe(acc_1, acc_3);
	*fi2 = y[0];

	matrix_1x2_mul_2x1(C, x_pri, Cx);
	eps[0] = y[0] - Cx[0];

	/* S(t) = CP(t|t-1)C^T + W */
	matrix_1x2_mul_2x2(C, P_pri, CP);
	matrix_1x2_mul_2x1(CP, C, CPCT);
	S[0] = CPCT[0] + W[0];

	/* K(t) = P(t|t-1)C^TS(t)^-1 */
	matrix_2x2_mul_2x1(P_pri, C, PCT);
	S1[0] = 1/S[0];
	matrix_2x1_mul_1x1(PCT, S1, K);

	/* x(t|t) = x(t|t-1) + K(t)eps(t) */
	matrix_2x1_mul_1x1(K, eps, Keps);
	matrix_2x1_add_2x1(x_pri, Keps, x_post);

	/* P(t|t) = P(t|t-1) - K(t)S(t)K(t)^T */
	matrix_2x1_mul_1x1(K, S, KS);
	matrix_2x1_mul_1x2(KS, K, KSKT);
	matrix_2x2_sub_2x2(P_pri, KSKT, P_post);

	return x_post[0];
}

void matrix_2x2_mul_2x1(float * in1, float * in2, float * out)
{
	out[0] = in1[0] * in2[0] + in1[1] * in2[1];
	out[1] = in1[2] * in2[0] + in1[3] * in2[1];
}

void matrix_2x1_mul_1x1(float * in1, float * in2, float * out)
{
	out[0] = in1[0] * in2[0];
	out[1] = in1[1] * in2[0];
}

void matrix_2x1_add_2x1(float * in1, float * in2, float * out)
{
	out[0] = in1[0] + in2[0];
	out[1] = in1[1] + in2[1];
}

void matrix_2x2_mul_2x2(float * in1, float * in2, float * out)
{
	out[0] = in1[0] * in2[0] + in1[1] * in2[2];
	out[1] = in1[0] * in2[1] + in1[1] * in2[3];
	out[2] = in1[2] * in2[0] + in1[3] * in2[2];
	out[3] = in1[2] * in2[1] + in1[3] * in2[3];
}

void matrix_2x2_add_2x2(float * in1, float * in2, float * out)
{
	out[0] = in1[0] + in2[0];
	out[1] = in1[1] + in2[1];
	out[2] = in1[2] + in2[2];
	out[3] = in1[3] + in2[3];
}

void matrix_1x2_mul_2x1(float * in1, float * in2, float * out)
{
	out[0] = in1[0] * in2[0] + in1[1] * in2[1];
}

void matrix_1x2_mul_2x2(float * in1, float * in2, float * out)
{
	out[0] = in1[0] * in2[0] + in1[1] * in2[2];
	out[1] = in1[0] * in2[1] + in1[1] * in2[3];
}

void matrix_2x1_mul_1x2(float * in1, float * in2, float * out)
{
	out[0] = in1[0] * in2[0];
	out[1] = in1[0] * in2[1];
	out[2] = in1[1] * in2[0];
	out[3] = in1[1] * in2[1];
}

void matrix_2x2_sub_2x2(float * in1, float * in2, float * out)
{
	out[0] = in1[0] - in2[0];
	out[1] = in1[1] - in2[1];
	out[2] = in1[2] - in2[2];
	out[3] = in1[3] - in2[3];
}

void matrix_2x2_trans(float * in, float * out)
{
	out[0] = in[0];
	out[1] = in[2];
	out[2] = in[1];
	out[3] = in[3];
}
