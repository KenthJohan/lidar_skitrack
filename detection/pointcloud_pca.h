#pragma once

#include <OpenBLAS/lapack.h>
#include <OpenBLAS/lapacke.h>
#include <OpenBLAS/cblas.h>

#include <stdint.h>

#include "csc_math.h"


/**
 * @brief Filter out points that is not good for finding the ground plane
 * @param[out]    dst           Pointer to the destination array where the elements is to be copied
 * @param[in]     dst_stride    Specifies the byte offset between consecutive elements
 * @param[in]     src           Pointer to the source of data to be copied
 * @param[out]    src_stride    Specifies the byte offset between consecutive elements
 * @param[in,out] n             Is a pointer to an integer related to the number of elements to copy or how many were copied
 * @param[in]     dim           How many dimension in each element
 * @param[in]     k2
 */
static void pointcloud_filter (float dst[], uint32_t dst_stride, float const src[], uint32_t src_stride, uint32_t *n, uint32_t dim, float k2)
{
	uint32_t j = 0;
	for (uint32_t i = 0; i < (*n); ++i)
	{
		float l2 = vvf32_dot (dim, src, src);
		if (l2 > k2)
		{
			vf32_cpy (dim, dst, src);
			dst += dst_stride;
			j++;
		}
		src += src_stride;
	}
	(*n) = j;
}


/**
 * @brief
 * @param[in,out] x             Pointcloud
 * @param[in,out] x1            Pointcloud extra memory
 * @param[in,out] nx            Is the number of points in pointcloud \param x
 * @param[in]     ldx           Leading dimension of the array specified for \param x and \param x1
 * @param[out]    centroid      v3f32 Center of pointcloud \param x
 * @param[out]    w             v3f32 Eigen values
 * @param[out]    c             m3f32 Eigen vectors
 * @param[out]    r             m3f32 Rotation matrix
 */
static void pointcloud_pca (float x[], float x1[], uint32_t *nx, uint32_t ldx, float centroid[3], float w[3], float c[3*3], float r[3*3])
{
	uint32_t n = (*nx);
	//Remove bad points:
	pointcloud_filter (x, ldx, x, ldx, &n, 3, 1.0f);
	//Move the center of all points to origin:
	vf32_move_center_to_zero (3, x, ldx, x1, ldx, n, centroid);
	//Calculate the covariance matrix of the points which can be used to get the orientation of the points:
	//matrix(c) := scalar(alpha) * matrix(pc1) * matrix(pc1)^T
	float alpha = 1.0f / ((float)n - 1.0f);
	cblas_sgemm (CblasColMajor, CblasNoTrans, CblasTrans, 3, 3, n, alpha, x1, ldx, x1, ldx, 0.0f, c, 3);
	//Calculate the eigen vectors (c) and eigen values (w) from covariance matrix (c) which will get the orientation of the points:
	//https://software.intel.com/sites/products/documentation/doclib/mkl_sa/11/mkl_lapack_examples/dsyev.htm
	LAPACKE_ssyev (LAPACK_COL_MAJOR, 'V', 'U', 3, c, 3, w);
	//Rectify every point by this rotation matrix which is the current orientation of the points:
	r[0] = c[3];
	r[1] = c[6];
	r[2] = c[0];
	r[3] = c[4];
	r[4] = c[7];
	r[5] = c[1];
	r[6] = c[5];
	r[7] = c[8];
	r[8] = c[2];
	//matrix(pc) := matrix(r) * matrix(pc1)
	cblas_sgemm (CblasColMajor, CblasNoTrans, CblasNoTrans, 3, n, 3, 1.0f, r, 3, x1, ldx, 0.0f, x, ldx);
	(*nx) = n;
}
