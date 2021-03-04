#pragma once

#include <stdint.h>
#include "csc/csc_debug.h"
#include "csc/csc_m3f32.h"
#include "csc/csc_v3f32.h"

#include "../shared/shared.h"
#include "mathmisc.h"



struct skitrack1
{
	uint32_t pc_count;//Number of points in pointcloud
	float pc[LIDAR_WH*POINT_STRIDE];//All points of pointcloud
	float pc1[LIDAR_WH*POINT_STRIDE];//All points of pointcloud
	float w[3];//Eigen values
	float c[3*3];//Covariance matrix first then 3x eigen vectors
	float r[3*3];//Rotation matrix
	float centroid[3];//Center of pointcloud (pc)
};


static void skitrack1_process (struct skitrack1 * s)
{
	ASSERT_PARAM_NOTNULL (s);
	pointcloud_pca (s->pc, s->pc1, &s->pc_count, POINT_STRIDE, s->centroid, s->w, s->c, s->r);


}
