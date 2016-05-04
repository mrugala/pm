/****************************************************************
 * Wszelkie operacje matematyczne.		                        *
 *                                                              *
 * Autor: Jêdrzej Mruga³a                                       *
 *                                                              *
 ****************************************************************/

#ifndef MATHM_H_
#define MATHM_H_

#include <math.h>
#include <stdbool.h>

typedef struct {
	float x;
	float y;
	float z;
	float fiX;
	float fiY;
	float fiZ;
} input_t;

typedef struct {
	float fi1;
	float fi2;
	float fi3;
	float fi4;
	float fi5;
} output_t;

/* definicje sta³ych */
#define JEDN_S		1				// mno¿nik do przeliczania d³ugoœci z jednostek SI (m)
									// 1000 - milimetry
									//  100 - centymetry
									//    1 - metry
#define LEN1 		0.10*JEDN_S		// d³ugoœæ pierwszego (od podstawy) segmentu ramienia [mm]
#define LEN2 		0.10*JEDN_S		// d³ugoœæ drugiego segmentu ramienia [mm]
#define PI 			3.141592653589
#define F_MPU60X0 	1000 			// czêstotliwoœæ próbkowania akcelerometru MPU60X0 [Hz]
#define G			9.8105 			// przyspieszenie ziemskie
#define A2			12
#define D4			10
#define samplesForMean 100			//okres próbkowania [ms]
#define samplesForA 5				//ilosc próbek do usredniania A
#define ALPHA		0.2				//wspolczynnik filtra komplementarbego

bool compareFloats(float a, float b);

/* korekta (odrzucenie) przyspieszenia ziemskiego */
float korxg(float fix, float fiy, float fiz);
float koryg(float fix, float fiy, float fiz);
float korzg(float fix, float fiy, float fiz);

float korxa(float A[3], float fix, float fiy, float fiz);
float korya(float A[3], float fix, float fiy, float fiz);
float korza(float A[3], float fix, float fiy, float fiz);

/* przeliczenie przyspieszenia na wspó³rzêdne i prêdkosci k¹towej na k¹t */
float AtoS(float A);
float OMtoDeg(float OM);

/* min/max dla liczb typu float */
float min(float a, float b);
float max(float a, float b);

/* operacje na macierzach 3x3 */
void mult3x3matrix(float A[3][3], float B[3][3], float C[3][3]);
void mult3x3by3x1matrix(float A[3][3], float B[3], float C[3]);
void rot(float alpha, float beta, float gamma, float Rot[3][3]);

output_t inputToOutput(input_t input);

#endif /* MATHM_H_ */
