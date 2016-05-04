#include "zodm.h"
#include <stdbool.h>
#include <stdio.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define C(x) #x,

static const char* zodm_error_str[] = {
	ZODM_ERROR_CODE(C)
};

#undef C

typedef struct
{
	/*float a1;*/
	float a2;
	/*float a3;*/
	/*float a4;*/
	/*float a5;*/
	/*float d1;*/
	/*float d2;*/
	/*float d3;*/
	float d4;
	/*float d5;*/
} ZDH_t;

static ZDH_t ZDH = { 120, 100 };

bool assert_equal_f(float a, float b, float epsilon)
{
	float diff = a - b;
	return (diff < epsilon) && (-diff < epsilon);
}

zodm_error_t calculate_zodm_5(input_data_t* input, float q_vect[/*static*/ 5])
{
	if (!input)
		return NULL_INPUT_ERROR;

	if (!q_vect)
		return NULL_Q_VECT_ERROR;

	matrix_t rot = { 0 };
	if (get_rotation_matrix(input->alpha_deg, input->beta_deg, input->gamma_deg, &rot))
		return ZODM_ROTATION_ERROR;

	float len = sqrtf(powf(input->x_mm, 2) + powf(input->y_mm, 2) + powf(input->z_mm, 2));

	if (len > ZDH.a2 + ZDH.d4)
	{
		input->x_mm *= (ZDH.a2 + ZDH.d4) / len;
		input->y_mm *= (ZDH.a2 + ZDH.d4) / len;
		input->z_mm *= (ZDH.a2 + ZDH.d4) / len;
		len = sqrtf(powf(input->x_mm, 2) + powf(input->y_mm, 2) + powf(input->z_mm, 2));
	}

	q_vect[0] = atan2f(input->y_mm, input->x_mm);
	float temp = *access_matrix_cell(&rot, 1, 2) * cosf(q_vect[0])
		- *access_matrix_cell(&rot, 0, 2) * sinf(q_vect[0]);
	q_vect[3] = atan2f(sqrtf(1 - powf(temp, 2)), temp);

	if (!assert_equal_f(sinf(q_vect[3]), 0, (float)0.0001))
	{
		float q12 = atan2f(*access_matrix_cell(&rot, 2, 2), *access_matrix_cell(&rot, 1, 0) 
			* cosf(q_vect[0]) + *access_matrix_cell(&rot, 0, 0) * sinf(q_vect[0]));
		temp = -ZDH.d4 * sinf(q12) + input->x_mm * cosf(q_vect[0]) + input->y_mm * sinf(q_vect[0]);
		q_vect[1] = atan2f(ZDH.d4 * cosf(q12) - input->z_mm, temp);
		temp = (powf(input->x_mm, 2) + powf(input->y_mm, 2)
			+ powf(input->z_mm, 2) - powf(ZDH.a2, 2) - powf(ZDH.d4, 2)) / (2 * ZDH.a2 * ZDH.d4);
		q_vect[2] = atan2f(temp, sqrtf(1 - powf(temp, 2)));
		q_vect[4] = atan2f((-*access_matrix_cell(&rot, 2, 0) * cosf(q12) - sinf(q12))
			* (*access_matrix_cell(&rot, 0, 0) * cosf(q_vect[0]) + *access_matrix_cell(&rot, 1, 0) 
			* sinf(q_vect[0])), (-*access_matrix_cell(&rot, 2, 1) * cosf(q12) - sinf(q12))
			* (*access_matrix_cell(&rot, 0, 1) * cosf(q_vect[0]) + *access_matrix_cell(&rot, 1, 1) 
			* sinf(q_vect[0])));
	}
	else
	{
		float q124 = atan2f(*access_matrix_cell(&rot, 2, 0), *access_matrix_cell(&rot, 2, 1));
		temp = (powf(input->x_mm, 2) + powf(input->y_mm, 2) + powf(input->z_mm, 2) - powf(ZDH.a2, 2) 
			- powf(ZDH.d4, 2)) / (2 * ZDH.a2 * ZDH.d4);
		q_vect[2] = atan2f(sqrtf(1 - powf(temp, 2)), temp);
		q_vect[1] = atan2f(input->z_mm, sqrtf(powf(input->x_mm, 2) + powf(input->y_mm, 2)))
			+ atan2f(ZDH.d4 * sinf(q_vect[2]), ZDH.a2 + ZDH.d4 * cosf(q_vect[2]));
		q_vect[4] = atan2f(*access_matrix_cell(&rot, 2, 0), *access_matrix_cell(&rot, 2, 1)) - q_vect[1] 
			- q_vect[2];
		printf("przypadek osobliwy");
	}

	int i;
	for (i = 0; i < 5; i++)
	{
		q_vect[i] *= (180 / (float)M_PI);
	}

	return ZODM_RETURN_OK;
}