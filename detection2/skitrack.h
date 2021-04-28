#pragma once

#include <stdint.h>
#include "csc/csc_debug.h"
#include "csc/csc_m3f32.h"
#include "csc/csc_m3f32_print.h"
#include "csc/csc_v3f32.h"
#include "csc/csc_v3f32_print.h"

#include "../shared/shared.h"
#include "../shared/log.h"
#include "mathmisc.h"

#define MAXOBJ 10

struct obj_context
{
	uint32_t count;
	struct v3f32 x[MAXOBJ];//Position
	struct v3f32 d[MAXOBJ];//Direction
	struct m3f32 c[MAXOBJ];//Coveriance
	struct m3f32 n[MAXOBJ];//Count
};

void obj_context_print (struct obj_context * obj)
{
	for (uint32_t i = 0; i < obj->count; ++i)
	{
		printf ("%i: N=%i\n", i, obj->n[i]);
	}
}

void obj_context_insert (struct obj_context * obj, struct v3f32 x[], uint32_t n)
{
	int dim = 3;
	int ldx = 3;
	float alpha = 1.0f;
	float beta = 0.0f;
	struct m3f32 c;
	cblas_sgemm (CblasColMajor, CblasNoTrans, CblasTrans, dim, dim, n, alpha, (float const*)x, ldx, (float const*)x, ldx, beta, (float*)&c, dim);
}




struct skitrack
{
	uint32_t framenr;
	//All points of pointcloud (x,y,z,a),(x,y,z,a)
	struct v4f32 x[LIDAR_WH];
};



static void skitrack_process (struct skitrack * ski)
{

}
