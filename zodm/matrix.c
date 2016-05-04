#include "matrix.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define C(x) #x,

static const char* matrix_error_str[] = {
	MATRIX_ERROR_CODE(C)
};

#undef C

matrix_t diag(size_t size)
{
	matrix_t ret = { size, size, { 0 } };
	memset(ret.data, 0, size);
	int i;
	for (i = 0; i < size; i++)
		*access_matrix_cell(&ret, i, i) = 1.0f;
	return ret;
}

#define MATRIX_CELL(rows, i, j) ((rows)*((i))+((j)))

float* access_matrix_cell(matrix_t* mat, int i, int j)
{
	if (i >= mat->a || j >= mat->b) return NULL;
	return &mat->data[MATRIX_CELL(mat->b, i, j)];
}

static matrix_error_t sub_add_matrices(matrix_t* mat1, matrix_t* mat2, matrix_t* result,
	float (*oper)(matrix_t*, matrix_t*, int, int))
{
	if (!mat1 || !mat2 || !result)
		return NULL_MATRIX_ERROR;

	if (mat1->a != mat2->a)
		return DIM_A_ERROR;

	if (mat1->b != mat2->b)
		return DIM_B_ERROR;

	result->a = mat1->a;
	result->b = mat1->b;

	int i, j;
	for (i = 0; i < mat1->a; ++i)
	{
		for (j = 0; j < mat2->b; ++j)
		{
			*access_matrix_cell(result, i, j) = oper(mat1, mat2, i, j);
		}
	}

	return RETURN_OK;
}

static float subtract_cells(matrix_t* mat1, matrix_t* mat2, int row, int col)
{
	return (*access_matrix_cell(mat1, row, col)) - (*access_matrix_cell(mat2, row, col));
}

static float add_cells(matrix_t* mat1, matrix_t* mat2, int row, int col)
{
	return (*access_matrix_cell(mat1, row, col)) + (*access_matrix_cell(mat2, row, col));
}

static matrix_error_t mult_div_matrices(matrix_t* mat, float factor, matrix_t* result,
	float(*oper)(matrix_t*, float, int, int))
{
	if (!mat || !result)
		return NULL_MATRIX_ERROR;

	result->a = mat->a;
	result->b = mat->b;

	int i, j;
	for (i = 0; i < mat->a; ++i)
	{
		for (j = 0; j < mat->b; ++j)
		{
			*access_matrix_cell(result, i, j) = oper(mat, factor, i, j);
		}
	}

	return RETURN_OK;
}

static float multiply_cell(matrix_t* mat, float factor, int row, int col)
{
	return (*access_matrix_cell(mat, row, col)) * factor;
}

static float divide_cell(matrix_t* mat, float divisor, int row, int col)
{
	return multiply_cell(mat, 1 / divisor, row, col);
}

matrix_error_t add_matrices(matrix_t* mat1, matrix_t* mat2, matrix_t* result)
{
	float(*oper)(matrix_t*, matrix_t*, int, int) = add_cells;

	return sub_add_matrices(mat1, mat2, result, oper);
}

matrix_error_t subtract_matrices(matrix_t* mat1, matrix_t* mat2, matrix_t* result)
{
	float(*oper)(matrix_t*, matrix_t*, int, int) = subtract_cells;

	return sub_add_matrices(mat1, mat2, result, oper);
}

matrix_error_t multiply_matrix(matrix_t* mat, float factor, matrix_t* result)
{
	float(*oper)(matrix_t*, float, int, int) = multiply_cell;

	return mult_div_matrices(mat, factor, result, oper);
}

matrix_error_t divide_matrix(matrix_t* mat, float divisor, matrix_t* result)
{
	float(*oper)(matrix_t*, float, int, int) = divide_cell;

	return mult_div_matrices(mat, divisor, result, oper);
}

matrix_error_t multiply_matrices(matrix_t* mat1, matrix_t* mat2, matrix_t* result)
{
	if (!mat1 || !mat2 || !result)
		return NULL_MATRIX_ERROR;

	if (mat1->b != mat2->a)
		return DIM_ERROR_MULT;

	result->a = mat1->a;
	result->b = mat2->b;

	int i, j, k;
	for (i = 0; i < mat1->a; ++i)
	{
		for (j = 0; j < mat2->b; ++j)
		{
			for (k = 0; k < mat1->b; k++)
			{
				*access_matrix_cell(result, i, j) += (*access_matrix_cell(mat1, i, k)) 
					* (*access_matrix_cell(mat2, k, j));
			}
		}
	}

	return RETURN_OK;
}

matrix_error_t transpose_matrix(matrix_t* mat, matrix_t* result)
{
	if (!mat || !result)
		return NULL_MATRIX_ERROR;

	result->a = mat->b;
	result->b = mat->a;

	int i, j;
	for (i = 0; i < result->a; ++i)
	{
		for (j = 0; j < result->b; ++j)
		{
			*access_matrix_cell(result, i, j) = *access_matrix_cell(mat, j, i);
		}
	}

	return RETURN_OK;
}

matrix_error_t get_determinant(matrix_t* mat, float* det)
{
	if (!mat || !det)
		return NULL_MATRIX_ERROR;

	if ((mat->a != mat->b) || mat->a < 1)
		return GENERAL_DIM_ERROR;

	if (mat->a == 1)
	{
		*det = *access_matrix_cell(mat, 0, 0);
	}
	else if (mat->a == 2)
	{
		*det = (*access_matrix_cell(mat, 0, 0) * *access_matrix_cell(mat, 1, 1)) 
			- (*access_matrix_cell(mat, 0, 1) * *access_matrix_cell(mat, 1, 0));
	}
	else
	{
		
		float temp_det;
		matrix_t temp = { 0 };
		
		*det = 0;

		temp.a = mat->a - 1;
		temp.b = mat->b - 1;

		int col, tcol, i, j;
		for (col = 0; col < mat->a; col++) 
		{
			for (i = 1; i < mat->a; i++)
			{
				tcol = 0;
				for (j = 0; j < mat->b; j++)
				{
					if (j == col)
						continue;
					*access_matrix_cell(&temp, i-1, tcol) = *access_matrix_cell(mat, i, j);
					tcol++;
				}
			}
			if (get_determinant(&temp, &temp_det))
				return DET_ERROR;
			*det += powf(-1.0f, (float)col) * *access_matrix_cell(mat, 0, col) * temp_det;
		}
	}

	return RETURN_OK;
}

static matrix_error_t get_cofactor_matrix(matrix_t* mat, matrix_t* result)
{
	if (!mat || !result)
		return NULL_MATRIX_ERROR;

	if (mat->a != mat->b)
		return GENERAL_DIM_ERROR;

	result->a = mat->a;
	result->b = mat->b;

	matrix_t temp = { mat->a - 1, mat->b - 1, { 0 } };
	float det = 0;

	int i, j, i1, j1, i2, j2;
	for (i = 0; i < result->a; i++)
	{
		for (j = 0; j < result->b; j++)
		{
			i2 = 0;
			for (i1 = 0; i1 < mat->a; i1++)
			{
				if (i1 == i)
					continue;

				j2 = 0;
				for (j1 = 0; j1 < mat->b; j1++)
				{
					if (j1 == j)
						continue;
					*access_matrix_cell(&temp, i2, j2) = *access_matrix_cell(mat, i1, j1);
					j2++;
				}
				i2++;
			}
			if (get_determinant(&temp, &det))
				return DET_ERROR;
			*access_matrix_cell(result, i, j) = powf(-1.0f, (float)(i+j)+2.0f) * det;
		}
	}

	return RETURN_OK;
}

matrix_error_t get_inverted_matrix(matrix_t* mat, matrix_t* result)
{
	matrix_t temp = { 0 }, temp1 = { 0 };
	float det;

	if (get_determinant(mat, &det) || det == 0)
		return DET_ERROR;

	if (get_cofactor_matrix(mat, &temp))
		return COFACTOR_MATRIX_ERROR;

	if (transpose_matrix(&temp, &temp1))
		return TRANSPOSE_ERROR;

	if (divide_matrix(&temp1, det, result))
		return GENERAL_ERROR;

	return RETURN_OK;
}

int matrix_error_to_str(int error_code, char* str, size_t size)
{
	if (error_code >= TOP)
		return -1;

	sprintf_s(str, size, "%s", matrix_error_str[error_code]);

	return 0;
}

void print_matrix_error(int error_code)
{
	char str[64];
	if (!matrix_error_to_str(error_code, str, 64))
	{
		printf("Matrix error: %s", str);
	}
	else
	{
		printf("Invalid error code for matrix error");
	}

}

matrix_error_t get_rotation_matrix(float alpha_deg, float beta_deg, float gamma_deg, matrix_t* result)
{
	if (!result)
		return NULL_MATRIX_ERROR;

	if (fabsf(alpha_deg) > 180.0 || fabsf(beta_deg) > 180.0 || fabsf(gamma_deg) > 180.0)
		return ANGLE_ERROR;

	float alpha = (float)M_PI / 180 * alpha_deg;
	float beta = (float)M_PI / 180 * beta_deg;
	float gamma = (float)M_PI / 180 * gamma_deg;

	matrix_t r = { 0 };
	matrix_t rot_x = { 3, 3, { 1, 0, 0, 0, cosf(alpha), -sinf(alpha), 0, sinf(alpha), cosf(alpha) } };
	matrix_t rot_y = { 3, 3, { cosf(beta), 0, sinf(beta), 0, 1, 0, -sinf(beta), 0, cosf(beta) } };
	matrix_t rot_z = { 3, 3, { cosf(gamma), -sinf(gamma), 0, sinf(gamma), cosf(gamma), 0, 0, 0, 1 } };

	if (multiply_matrices(&rot_x, &rot_y, &r) || multiply_matrices(&r, &rot_x, result))
		return ROTATION_ERROR;

	return RETURN_OK;
}
