#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <OpenBLAS/cblas.h>

#define PRINT_FORMAT "%+10.6f "

void print_mat (float c[], uint32_t n)
{
	for(uint32_t i=0; i<n; i++)
	{
		for(uint32_t j=0; j<4; j++)
		{
			printf(PRINT_FORMAT, c[i*4+j]);
		}
		printf("\n");
	}
}


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



void test3()
{
	double A[3*3] =
	{
	 1.0, 1.0,  1.0,
	-3.0, 4.0, -1.0,
	-3.0, 4.0, -1.0,
	};
	double B[4*8] =
	{
	 1.0, 2.0,  1.0, 1000.0,
	-3.0, 4.0, -1.0, 1000.0,
	-3.0, 4.0, -1.0, 1000.0,
	-3.0, 4.0, -1.0, 1000.0,
	 1.0, 2.0,  1.0, 1000.0,
	-3.0, 4.0, -1.0, 1000.0,
	-3.0, 4.0, -1.0, 1000.0,
	-3.0, 4.0, -1.0, 1000.0
	};
	double C[4*8] = {0.0f};
	double alpha = 1.0;
	double beta = 0.0;
	cblas_dgemm (CblasColMajor, CblasTrans, CblasNoTrans, 3, 8, 3, alpha, A, 3, B, 4, beta, C, 4);
}


#define COUNT 4
/*
Expected:
-0.403831 2.831014 0.052149 0.000000:
-0.496344 2.911914 0.005367 0.000000:
*/
void test4 ()
{
	float A[3*3] =
	{
	-0.941,     -0.138,     -0.309,
	+0.145,     -0.989,     +0.003,
	-0.306,     -0.042,     +0.951
	};

	float B[4*COUNT] =
	{
	0.773404, -2.747617, 0.181334, 0.000000,
	0.886464, -2.812897, 0.165583, 0.000000
	};

	float C[4*COUNT] = {0.0f};
	float alpha = 1.0;
	float beta = 0.0;
	cblas_sgemm (CblasColMajor, CblasTrans, CblasNoTrans, 3, COUNT, 3, alpha, A, 3, B, 4, beta, C, 4);
	print_mat (C, COUNT);
}



int main()
{
	//test1 ();
	//test2 ();
	//test3 ();
	test4 ();
	return 0;
}
