#pragma once


#ifdef __MINGW32__
//pacman -S mingw64/mingw-w64-x86_64-openblas
//-lopenblas
#include <OpenBLAS/lapack.h>
#include <OpenBLAS/lapacke.h>
#include <OpenBLAS/cblas.h>
#else
//sudo apt-get install libblas-dev
//sudo apt-get install libopenblas-dev
//sudo apt-get install liblapacke-dev
#include <lapacke.h>
#include <cblas.h>
#endif

#include <stdint.h>

#include "csc/csc_math.h"
#include "csc/csc_v4f32.h"
#include "../shared/shared.h"



/*
In 2D computer graphics, a pixel represents a value on a regular grid in two-dimensional space.
In 3D computer graphics, a voxel represents a value on a regular grid in three-dimensional space.
*/


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


static uint32_t pointcloud_filter1 (v4f32 p[], float k2, uint32_t x0, uint32_t x1)
{
	ASSERT_PARAM_NOTNULL (p);
	ASSERT (x0 >= 0);
	ASSERT (x1 < LIDAR_W);
	uint32_t j = 0;
	for (uint32_t x = x0; x <= x1; ++x)
	{
		for (uint32_t y = 0; y < LIDAR_H; ++y)
		{
			uint32_t i = LIDAR_INDEX(x,y);
			float l2 = v4f32_norm2 (p[i]);
			if (l2 < k2) {continue;}
			v4f32_cpy (p[j], p[i]);
			j ++;
		}
	}
	return j;
}




static void flipper()
{

}


/**
 * @brief
 * @param[in,out] x             Pointcloud
 * @param[in,out] x1            Pointcloud extra memory
 * @param[in]     n             Number of points in pointcloud \param x and \param x1
 * @param[in]     ldx           Leading dimension of the array specified for \param x and \param x1
 * @param[out]    centroid      v3f32 Center of pointcloud \param x
 * @param[out]    w             v3f32 Eigen values
 * @param[out]    c             m3f32 Eigen vectors
 * @param[out]    r             m3f32 Rotation matrix
 */
static void pointcloud_pca (float x[], float x1[], uint32_t n, uint32_t ldx, float centroid[3], float w[3], float c[3*3], float e[3*3], float r[3*3], float k)
{
	//Number of dimensions:
	uint32_t dim = 3;
	//Move the center of all points to origin:
	vf32_move_center_to_zero (dim, x, ldx, x1, ldx, n, centroid);
	//Calculate the covariance matrix of the points which can be used to get the orientation of the points:
	//matrix(c) := scalar(alpha) * matrix(pc1) * matrix(pc1)^T
	float alpha = 1.0f / ((float)n - 1.0f);
	cblas_sgemm (CblasColMajor, CblasNoTrans, CblasTrans, dim, dim, n, alpha*k, x1, ldx, x1, ldx, (1.0f - k), c, dim);
	//Calculate the eigen vectors (c) and eigen values (w) from covariance matrix (c) which will get the orientation of the points:
	//https://software.intel.com/sites/products/documentation/doclib/mkl_sa/11/mkl_lapack_examples/dsyev.htm
	memcpy (e, c, sizeof (float)*3*3);
	LAPACKE_ssyev (LAPACK_COL_MAJOR, 'V', 'U', dim, e, dim, w);
	//Prevent pointcloud flipping:
	//((0,1,0) dot (c[6], c[7], c[8]) < 0) = (c[7] < 0)
	if (e[7] < 0.0f)
	{
		//Flip Y vector of pointcloud
		e[3] *= -1.0f;
		e[4] *= -1.0f;
		e[5] *= -1.0f;
		//Flip Z vector of pointcloud
		e[6] *= -1.0f;//x
		e[7] *= -1.0f;//y
		e[8] *= -1.0f;//z
	}


	//Assemble a rotation matrix from eigen vectors which are used to rotate pointcloud:
	//After rotation the pointcloud should be aligned to standard basis:
	//Eigen column vectors in (c) are sorted by eigen values, shortest vector first:
	r[0] = e[3];//Medium length PCA basis to x standard basis
	r[1] = e[4];//Medium length PCA basis to x standard basis
	r[2] = e[5];//Medium length PCA basis to x standard basis
	r[3] = e[6];//Farthest length PCA basis to y standard basis
	r[4] = e[7];//Farthest length PCA basis to y standard basis
	r[5] = e[8];//Farthest length PCA basis to y standard basis
	r[6] = e[0];//Shortest length PCA basis to z standard basis
	r[7] = e[1];//Shortess length PCA basis to z standard basis
	r[8] = e[2];//Shortset length PCA basis to z standard basis
	//x : (3 * n) matrix
	//r : (3 * 3) matrix
	//x := r^T * x + 0*x
	cblas_sgemm (CblasColMajor, CblasTrans, CblasNoTrans, dim, n, dim, 1.0f, r, dim, x1, ldx, 0.0f, x, ldx);


	/*
	r[0] = c[3];// x <=> y
	r[1] = c[6];// y <=> z
	r[2] = c[0];// z <=> x
	r[3] = c[4];
	r[4] = c[7];
	r[5] = c[1];
	r[6] = c[5];
	r[7] = c[8];
	r[8] = c[2];
	//x : (3 * n) matrix
	//r : (3 * 3) matrix
	//x := r^T * x + 0*x
	cblas_sgemm (CblasColMajor, CblasNoTrans, CblasNoTrans, dim, n, dim, 1.0f, r, dim, x1, ldx, 0.0f, x, ldx);
	*/
}


/**
 * @brief Projects pixels from a 2D image to a 1D image in direction \p k
 * @param xn[in] Width of 2D image
 * @param yn[in] Height  of 2D image
 * @param k[in]  Direction. P(x,y) = img2D(x, y + x*k)
 * @param q[out] The 1D image.
 */
void vf32_project_2d_to_1d (float p[], uint32_t xn, uint32_t yn, float k, float q[])
{
	for (uint32_t y = 0; y < yn; ++y)
	{
		float sum = 0.0f;
		for (uint32_t x = 0; x < xn; ++x)
		{
			float yy = (float)y + (float)x*k;
			if (yy < 0.0f){continue;}
			if (yy >= (float)yn){continue;}
			ASSERT (yy >= 0.0f);
			ASSERT (yy < (float)yn);
			uint32_t index = (uint32_t)yy*xn + x;
			ASSERT (index < xn*yn);
			sum += p[index];
		}
		float val = sum;
		q[y] = val;
	}
}



/**
 * @brief Projects pixels from a 2D image to a 1D image in direction \p k
 * @param xn[in] Width of 2D image
 * @param yn[in] Height  of 2D image
 * @param k[in]  Direction. P(x,y) = img2D(x, y + x*k)
 * @param q[out] The 1D image.
 */
void vf32_project_2d_to_1d_pn (float const p[], uint32_t xn, uint32_t yn, float k, float q[])
{
	for (uint32_t y = 0; y < yn; ++y)
	{
		float sump = 0.0f;
		float sumn = 0.0f;
		for (uint32_t x = 0; x < xn; ++x)
		{
			float yy = (float)y + (float)x*k;
			if (yy < 0.0f){continue;}
			if (yy >= (float)yn){continue;}
			ASSERT (yy >= 0.0f);
			ASSERT (yy < (float)yn);
			uint32_t index = (uint32_t)yy*xn + x;
			ASSERT (index < xn*yn);
			if (p[index] > 0.0f)
			{
				sump += 1;
			}
			if (p[index] < 0.0f)
			{
				sumn += 1;
			}
		}
		float val = (sump - sumn) / xn;
		q[y] = val;
	}
}



float vf32_most_common_line (float const p[], uint32_t xn, uint32_t yn, uint32_t yp)
{
	float highscore = 0.0f;
	float k1 = 0.0f;
	float const delta = 0.1f;
	for (float k = -1.0f; k < 1.0f; k += delta)
	{
		float score = 0.0f;
		for (uint32_t y = yp; y < yn-yp; ++y)
		{
			float sum = 0.0f;
			for (uint32_t x = 0; x < xn; ++x)
			{
				//skew in the y-direction by (k) amount:
				float yy = y + x*k;
				if (yy < 0.0f || yy >= yn)
				{
					continue;
				}
				ASSERT (yy >= 0.0f);
				ASSERT (yy < (float)yn);
				uint32_t index = (uint32_t)yy*xn + x;
				ASSERT (index < xn*yn);
				//Sum of noisy pixel will become close to zero:
				//Sum of similiar pixel will become large positive or negative value:
				//TODO: Do not count undefined pixels:
				sum += p[index];
			}
			score += sum*sum;
		}
		if (score > highscore)
		{
			highscore = score;
			k1 = k;
		}
		printf ("sum:  %+f : %f\n", k, score);
	}
	printf ("best: %+f : %f\n", k1, highscore);
	return k1;
}



float vf32_most_common_line2 (float const p[], uint32_t xn, uint32_t yn, float q[])
{
	float max = 0.0f;
	float k1 = 0.0f;
	float const delta = 0.1f;
	for (float k = -1.0f; k < 1.0f; k += delta)
	{
		vf32_project_2d_to_1d_pn (p, xn, yn, k, q);
		float sum = 0.0f;
		for (uint32_t i = 0; i < yn; ++i)
		{
			sum += q[i]*q[i];
		}
		if (sum > max)
		{
			max = sum;
			k1 = k;
		}
	}
	//printf ("max: %+f : %f\n", k1, max);
	return k1;
}



void point_select (uint32_t pointcol[LIDAR_WH], int x, int y, uint32_t color)
{
	int index = LIDAR_INDEX(x,y);
	ASSERT (index < LIDAR_WH);
	printf ("index %i\n", index);
	pointcol[index] = color;
}




void point_to_pixel (float const p[4], uint32_t xn, uint32_t yn, float * pixel, float * x, float * y)
{
	float const sx = 1.0f/IMG_SCALE;
	float const sy = 1.0f/IMG_SCALE;
	(*x) = p[0]*sx + xn/2.0f;
	(*y) = p[1]*sy + yn/2.0f;
	//z-value becomes the pixel value:
	(*pixel) = p[2];
}


void pixel_to_point (float p[4], uint32_t xn, uint32_t yn, float pixel, float x, float y)
{
	//x = p*sx + xn/2
	//x - xn/2 = p*sx
	//(x - xn/2)/sx = p
	p[0] = (x - xn/2) * IMG_SCALE;
	p[1] = (y - yn/2) * IMG_SCALE;
	p[2] = pixel;
}




uint32_t point_project (float pix[], float imgf[], uint32_t xn, uint32_t yn, float v[], uint32_t v_stride, uint32_t x_count)
{
	memset (pix, 0, sizeof(float) * xn * yn);
	memset (imgf, 0, sizeof(float) * xn * yn);
	uint32_t count = 0;
	for (uint32_t i = 0; i < x_count; ++i, v += v_stride)
	{
		//v[2] += 1.0f;
		//(x,y) becomes the pixel position:
		//Set origin in the middle of the image and 20 pixels becomes 1 meter:
		//z-value becomes the pixel value:
		float x;
		float y;
		float z;
		point_to_pixel (v, xn, yn, &z, &x, &y);
		//Crop the pointcloud to the size of the image;
		if (x >= xn){continue;}
		if (y >= yn){continue;}
		if (x < 0){continue;}
		if (y < 0){continue;}
		//Convert (x,y) to index row-major:
		uint32_t index = ((uint32_t)y * xn) + (uint32_t)x;
		//z += 10.0f;
		//If multiple points land on one pixel then it will be accumalted but it will also be normalized later on:
		pix[index] += z;
		imgf[index] += 1.0f;
		count++;
		//pix[index] = 0.5f*pix[index] + 0.5f*z;
	}

	//Normalize every non zero pixel:
	for (uint32_t i = 0; i < IMG_XN*IMG_YN; ++i)
	{
		if (imgf[i] > 0.0f)
		{
			pix[i] /= imgf[i];
		}
	}

	for (uint32_t i = 0; i < IMG_XN*IMG_YN; ++i)
	{
		//Gradient convolution could be applied later so this statement will have no effect:
		//It is important that this statement does not affect the end result:
		//This statement test scenories where average pointcloud z-position is far of origin:
		//pix[i] += 10.0f;
	}
	return count;
}


static void random_points (float v[], unsigned n)
{
	while (n--)
	{
		v[0] = (float)rand() / (float)RAND_MAX;
		v[1] = (float)rand() / (float)RAND_MAX;
		v[2] = ((float)rand() / (float)RAND_MAX) * 10.0f;
		v[3] = 1.0f;
		v += 4;
	}
}



lapack_int m3f32_lapacke_inverse (float *A, unsigned n)
{
	int ipiv[3*3];
	lapack_int ret;
	ret =  LAPACKE_sgetrf (LAPACK_COL_MAJOR, n, n, A, n, ipiv);
	if (ret !=0)
	{
		return ret;
	}
	ret = LAPACKE_sgetri (LAPACK_COL_MAJOR, n, A, n, ipiv);
	return ret;
}



static void points_test_sinus_slope (float points[])
{
	//Test sinus slopes:
	for (float * p = points; p < points + LIDAR_WH*POINT_STRIDE; p += POINT_STRIDE)
	{
		p[2] += 0.2f * sin (4.0f*p[1]);
	}
}






/**
 * @brief main_vox_neighbor
 * @param v 3D array of id
 * @param x Origin coordinate
 * @param y Origin coordinate
 * @param z Origin coordinate
 */
static void main_vox_neighbor (uint8_t *id, uint8_t voxel[], uint8_t x, uint8_t y, uint8_t z)
{
	//This must be true to do an convolution:
	ASSERT (x > 0);
	ASSERT (y > 0);
	ASSERT (z > 0);
	ASSERT (x < (VOXEL_XN-1));
	ASSERT (y < (VOXEL_YN-1));
	ASSERT (z < (VOXEL_ZN-1));

	//(3x3x3) convolution comparison where (x,y,z) is the origin voxel and (a,b,c) is the neighbor voxels:
	for (uint8_t a = x - 1; a <= x + 1; ++a)
	{
		for (uint8_t b = y - 1; b <= y + 1; ++b)
		{
			for (uint8_t c = z - 1; c <= z + 1; ++c)
			{
				//Don't compare it selft:
				if (VOXEL_INDEX(a, b, c) == VOXEL_INDEX(x, y, z)) {continue;}
				//If a neigbor exist then copy the class:
				if (voxel[VOXEL_INDEX(a, b, c)] != 0)
				{
					voxel[VOXEL_INDEX(x, y, z)] = voxel[VOXEL_INDEX(a, b, c)];
					goto loop_break;
				}
			}
		}
	}
loop_break:
	//If no neigbor were found then generate a new classid for the origin voxel:
	if (voxel[VOXEL_INDEX(x, y, z)] == 0)
	{
		(*id)++;
		voxel[VOXEL_INDEX(x, y, z)] = (*id);
	}
}


/**
 * @brief main_test_voxels
 * @param voxel 3D array of ids
 * @param p Pointcloud, array of 4D point (x,y,z,w), w is not used yet.
 * @param n Number of points in pointcloud
 */
static void main_test_voxels
(
uint8_t voxel[VOXEL_XN*VOXEL_YN*VOXEL_ZN],
uint8_t img2d[VOXEL_XN*VOXEL_YN],
float const points[],//Stride=4
unsigned points_count
)
{
	//Reset each voxel:
	memset (voxel, 0, VOXEL_XN*VOXEL_YN*VOXEL_ZN);
	//Reset each pixel:
	memset (img2d, 0, VOXEL_XN*VOXEL_YN*sizeof(uint32_t));
	//Each voxel will be given an ID:
	uint8_t id = 0;

	//Iterate each point in pointcloud:
	for (unsigned i = 0; i < points_count; ++i, points+=4)
	{
		//Map 3d points to a index in the 3D array:
		float fx = (points[0])/VOXEL_SCALE;//Downscale the LIDAR points to lower resolution.
		float fy = (points[1])/VOXEL_SCALE;//Downscale the LIDAR points to lower resolution.
		float fz = (points[2])/VOXEL_SCALE;//Downscale the LIDAR points to lower resolution.
		uint8_t ux = fx; //This will be the direction the LIDAR is pointing at, (fx) will never be negative.
		uint8_t uy = fy+VOXEL_YN/2; //LIDAR (fy)=0 coordinate is moved to middle of the 3D image.
		uint8_t uz = fz+VOXEL_ZN/2; //LIDAR (fz)=0 coordinate is moved to middle of the 3D image.
		//Ignore edges because those can not be proccessed with convolution:
		if (ux >= (VOXEL_XN-1)){continue;}
		if (uy >= (VOXEL_YN-1)){continue;}
		if (uz >= (VOXEL_ZN-1)){continue;}
		if (ux <= 0){continue;}
		if (uy <= 0){continue;}
		if (uz <= 0){continue;}
		//if (voxel1[VOX_I(ux, uy, uz)]){continue;}
		//Project point to the 2D image, the pixel value represent (uz):
		//img2d[PIXEL_INDEX(ux, uy)] = (0xFF << 0) | (uz << 8) | (0xFF << 24);
		img2d[PIXEL_INDEX(ux, uy)] = img2d[PIXEL_INDEX(ux, uy)]/2 + uz*4;
		//printf ("%x\n", img2d[PIXEL_INDEX(ux, uy)]);
		main_vox_neighbor (&id, voxel, ux, uy, uz);
	}
}





uint32_t rgba_value (float value, float kr, float kg, float kb)
{
	uint32_t r = CLAMP (value*kr, 0.0f, 255.0f);
	uint32_t g = CLAMP (value*kg, 0.0f, 255.0f);
	uint32_t b = CLAMP (value*kb, 0.0f, 255.0f);
	return RGBA (r, g, b, 0xAA);
}



static uint32_t pointcloud_subset (float const img[], float pc[], uint32_t y0, uint32_t y1)
{
	uint32_t count = 0;
	for (uint32_t x = 0; x < IMG_XN; ++x)
	{
		for (uint32_t y = y0; y < y1; ++y)
		{
			uint32_t index = ((uint32_t)y * IMG_XN) + (uint32_t)x;
			float * p = pc + POINT_STRIDE * count;
			pixel_to_point (p, IMG_XN, IMG_YN, img[index]*2.0f, x, y);
			count++;
		}
	}
	return count;
}







