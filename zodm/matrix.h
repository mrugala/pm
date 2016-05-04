#ifndef __MATRIX_H__
#define __MATRIX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

typedef struct
{
	int a;
	int b;
	float data[256];
} matrix_t;

#define MATRIX_ERROR_CODE(x) \
		x(RETURN_OK) \
		x(GENERAL_DIM_ERROR) \
		x(DIM_A_ERROR) \
		x(DIM_B_ERROR) \
		x(DIM_ERROR_MULT) \
		x(NULL_MATRIX_ERROR) \
		x(ROTATION_ERROR) \
		x(ANGLE_ERROR) \
		x(DET_ERROR) \
		x(COFACTOR_MATRIX_ERROR) \
		x(TRANSPOSE_ERROR) \
		x(GENERAL_ERROR) \
		x(NOT_IMPLEMENTED_ERROR) \
		x(TOP) \

#define C(x) x,

typedef enum
{
	MATRIX_ERROR_CODE(C)
} matrix_error_t;

#undef C

matrix_t diag(size_t size);

matrix_error_t add_matrices(matrix_t* mat1, matrix_t* mat2, matrix_t* result);
matrix_error_t subtract_matrices(matrix_t* mat1, matrix_t* mat2, matrix_t* result);
matrix_error_t multiply_matrix(matrix_t* mat, float factor, matrix_t* result);
matrix_error_t divide_matrix(matrix_t* mat, float divisor, matrix_t* result);
matrix_error_t multiply_matrices(matrix_t* mat1, matrix_t* mat2, matrix_t* result);
matrix_error_t transpose_matrix(matrix_t* mat, matrix_t* result);
matrix_error_t get_determinant(matrix_t* mat, float* det);
matrix_error_t get_inverted_matrix(matrix_t* mat, matrix_t* result);
float* access_matrix_cell(matrix_t* mat, int i, int j);

int matrix_error_to_str(int error_code, char* str, size_t size);
void print_matrix_error(int error_code);

matrix_error_t get_rotation_matrix(float alpha_deg, float beta_deg, float gamma_deg, matrix_t* result);

#ifdef __cplusplus
}
#endif

#endif /*__MATRIX_H__*/