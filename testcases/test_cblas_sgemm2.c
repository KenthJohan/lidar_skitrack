#include <stdio.h>
#include <stdlib.h>
#include <OpenBLAS/cblas.h>

#include "../detection/points_read.h"
#include "../detection/lidar.h"
#include "csc_math.h"
#include "csc_m3f32.h"

#define TEST_FILENAME "../txtpoints/4/14_17_18_225279.txt"


/*
	+0.102     -0.244     +0.022
	-0.244     +1.739     -0.001
	+0.022     -0.001     +0.008
	*/


int main()
{
	float pos[LIDAR_WH*POINT_STRIDE];
	float c[3*3];
	float mean[3];
	uint32_t count = LIDAR_WH*POINT_STRIDE;
	points_read_filename (TEST_FILENAME, pos, &count);

	//Move the center of all points to origin:
	vf32_move_center_to_zero (DIMENSION (3), pos, POINT_STRIDE, count, mean);
	mf32_get_covariance (DIMENSION (3), pos, POINT_STRIDE, count, c);

	{
		float alpha = 1.0f;
		float beta = 0.0f;
		//cblas_sgemm (CblasColMajor, CblasNoTrans, CblasTrans, 3, 3, count, alpha, pos, 4, pos, 4, beta, c, 3);
		//vsf32_mul (DIMENSION (3)*DIMENSION (3), c, c, 1.0f / ((float)count - 1.0f));
	}


	m3f32_print (c, stdout);


	for (float * p = pos; p < pos + count*POINT_STRIDE; p += POINT_STRIDE)
	{
		//printf ("%+4.4f %+4.4f %+4.4f \n", p[0], p[1], p[2]);
	}
}
