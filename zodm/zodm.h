#ifndef __ZODM_H__
#define __ZODM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "matrix.h"
#include "stdbool.h"

typedef struct
{
	float x_mm;
	float y_mm;
	float z_mm;
	float alpha_deg;
	float beta_deg;
	float gamma_deg;
} input_data_t;

#define ZODM_ERROR_CODE(x) \
		x(ZODM_RETURN_OK) \
		x(NULL_INPUT_ERROR) \
		x(NULL_Q_VECT_ERROR) \
		x(ZODM_ROTATION_ERROR) \
		x(ZODM_TOP) \

#define C(x) x,

typedef enum
{
	ZODM_ERROR_CODE(C)
} zodm_error_t;

#undef C

zodm_error_t calculate_zodm_5(input_data_t* input, float q_vect[/*static*/ 5]);

bool assert_equal_f(float a, float b, float epsilon);

#ifdef __cplusplus
}
#endif

#endif /*__ZODM_H__*/
