#include <stdio.h>
#include <stdlib.h>
#include <OpenBLAS/cblas.h>

#define PRINT_FORMAT "%+10.1lf "

void test1()
{
	double A[6] =
	{
	 1.0, 2.0,  1.0,
	-3.0, 4.0, -1.0
	};
	double B[6] =
	{
	 1.0, 2.0,  1.0,
	-3.0, 4.0, -1.0
	};
	double C[9] =
	{
	.5, .5, .5,
	.5, .5, .5,
	.5, .5, .5
	};
	float alpha = 1.0f;
	float beta = 2.0f;
	cblas_dgemm (CblasColMajor, CblasNoTrans, CblasTrans, 3, 3, 2, alpha, A, 3, B, 3, beta, C, 3);
	for(int i=0; i<9; i++)
	printf(PRINT_FORMAT, C[i]);
	printf("\n");
}

void test2()
{
	double A[8] =
	{
	 1.0, 2.0,  1.0, 1000.0f,
	-3.0, 4.0, -1.0, 1000.0f
	};
	double B[8] =
	{
	 1.0, 2.0,  1.0, 1000.0f,
	-3.0, 4.0, -1.0, 1000.0f
	};
	double C[9] =
	{
	.5, .5, .5,
	.5, .5, .5,
	.5, .5, .5
	};
	float alpha = 1.0f;
	float beta = 2.0f;
	cblas_dgemm (CblasColMajor, CblasNoTrans, CblasTrans, 3, 3, 2, alpha, A, 4, B, 4, beta, C, 3);
	for(int i=0; i<9; i++)
	printf(PRINT_FORMAT, C[i]);
	printf("\n");
}


int main()
{
	test1 ();
	test2 ();
	return 0;
}
