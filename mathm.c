#include "mathm.h"
#include "stdint.h"

const float g[3] = { 0, 0, 1 };

void mult3x3matrix(float A[3][3], float B[3][3], float C[3][3])
{
	int i, j, k;

	for(i = 0; i<3; i++)
	{
		for(j = 0; j<3; j++)
		{
			C[i][j] = 0;
			for(k = 0; k<3; k++)
			{
				C[i][j] = C[i][j] + A[i][k] * B[k][j];
			}
		}
	}
}

void mult3x3by3x1matrix(float A[3][3], float B[3], float C[3])
{
	int i, k;

	for(i = 0; i<3; i++)
	{
		C[i] = 0;
		for(k = 0; k<3; k++)
		{
			C[i] = C[i] + A[i][k] * B[k];
		}
	}
}

void rot(float alfa, float beta, float gamma, float Rot[3][3])
{
	alfa = alfa*PI/180;
	beta = beta*PI/180;
	gamma = gamma*PI/180;

	float Rottemp[3][3] = {{ 0, 0, 0 },
	  	 { 0, 0, 0 },
	   	 { 0, 0, 0 }};

	float Rotx[3][3] = {{1, 0,         0		 },
				        {0, cos(alfa), -sin(alfa)},
					    {0, sin(alfa), cos(alfa) }};

	float Roty[3][3] = {{cos(beta),  0, sin(beta)},
    					{0,          1, 0        },
    					{-sin(beta), 0, cos(beta)}};

	float Rotz[3][3] = {{cos(gamma), -sin(gamma), 0},
	        			{sin(gamma), cos(gamma),  0},
	        			{0,          0,           1}};

	mult3x3matrix(Rotz,Roty,Rottemp);
	mult3x3matrix(Rottemp,Rotx,Rot);
}

float korxa(float A[3], float fix, float fiy, float fiz)
{
	float rota[3] = {0,0,0};
	float Rot[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	rot(fix,fiy,fiz, Rot);
	mult3x3by3x1matrix(Rot, A, rota);

	return rota[0];
}

float korya(float A[3], float fix, float fiy, float fiz)
{
	float rota[3] = {0,0,0};
	float Rot[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	rot(fix,fiy,fiz, Rot);
	mult3x3by3x1matrix(Rot, A, rota);

	return rota[1];
}

float korza(float A[3], float fix, float fiy, float fiz)
{
	float rota[3] = {0,0,0};
	float Rot[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	rot(fix,fiy,fiz, Rot);
	mult3x3by3x1matrix(Rot, A, rota);

	return rota[2];
}

float korxg(float fix, float fiy, float fiz)
{
	float rotg[3] = {0,0,0};
	float Rot[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	rot(-fix,-fiy,-fiz, Rot);
	mult3x3by3x1matrix(Rot, (float *)g, rotg);

	return rotg[0];
}

float koryg(float fix, float fiy, float fiz)
{
	float rotg[3] = {0,0,0};
	float Rot[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	rot(-fix,-fiy,-fiz, Rot);
	mult3x3by3x1matrix(Rot, (float *)g, rotg);

	return rotg[1];
}

float korzg(float fix, float fiy, float fiz)
{
	float rotg[3] = {0,0,0};
	float Rot[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	rot(-fix,-fiy,-fiz, Rot);
	mult3x3by3x1matrix(Rot, (float *)g, rotg);

	return rotg[2];
}

float min(float a, float b)
{
	if(a >= b)
		return b;
	else
		return a;
}

float max(float a, float b)
{
	if(a >= b)
		return a;
	else
		return b;
}

bool compareFloats(float a, float b)
{
	return (a >= b-0.001 && a <= b+0.001);
}
