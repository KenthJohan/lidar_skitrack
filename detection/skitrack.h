#pragma once

#include <stdint.h>
#include "csc/csc_debug.h"
#include "csc/csc_m3f32.h"

#include "../shared/shared.h"
#include "mathmisc.h"

#define SKITRACK2_PEAKS_COUNT 1



/*
	 1: Read filename                   : (Filename) -> (text 3D points)
	 2: Convert text points to f32      : (text 3D points) -> (3D points)
	 3: Filter out bad points           : (3D points) -> (3D points)
	 4: (PCA) Move center to origin     : (3D points) -> (3D points)
	 5: (PCA) Get covariance matrix     : (3D points) -> (3x3 matrix)
	 6: (PCA) Get eigen vectors         : (3x3 matrix) -> (3x3 rotation matrix)
	 7: (PCA) Rectify points            : ((3x3 rotation matrix), (3D points)) -> (3D points)
	 8: Project 3D points to 2D image   : (3D points)) -> (2D image)
	 9: (Conv) Amplify skitrack         : (2D image) -> (2D image)
	10: Remove low values               : (2D image) -> (2D image)
	11: (Conv) Smooth                   : (2D image) -> (2D image)
	12: Find most common line direction : (2D image) -> (direction)
	13: Project 2D image to 1D image    : ((2D image), (direction)) -> (1D image)
	14: Remove low values               : (1D image) -> (1D image)
	15: (Conv) Amplify 1D skitracks     : (1D image) -> (1D image)
	16: Find all peaks                  : (1D image) -> ((position), (strength))
	17: Output of skitrack position     : ((position), (strength))
*/
struct skitrack
{
	uint32_t pc_count;//Number of points in pointcloud
	float pc1[LIDAR_WH*POINT_STRIDE];//All points of pointcloud (x,y,z,a),(x,y,z,a)
	uint32_t pc2_count;//Number of points in pointcloud
	float pc2[LIDAR_WH*POINT_STRIDE];//All points of pointcloud (x,y,z,a),(x,y,z,a)
	float w[3];//Eigen values
	float c[3*3];//Covariance matrix first then 3x eigen vectors
	float r[3*3];//Rotation matrix
	float centroid[3];//Center point of pointcloud

	//2D images:
	float imgf[IMG_XN*IMG_YN];//Used for normalizing pixel
	float img1[IMG_XN*IMG_YN];//From projected points
	float img2[IMG_XN*IMG_YN];//Convolution from img1
	float img3[IMG_XN*IMG_YN];//Convolution from img2

	//1D images:
	float q1[IMG_YN];
	float q2[IMG_YN];
	float q3[IMG_YN];
	float qmem[IMG_YN];

	//Peaks index locations:
	float peak[SKITRACK2_PEAKS_COUNT];

	//Skitrack direction (x, x + k*y):
	float k;
};






static void skitrack_process (struct skitrack * s)
{
	ASSERT_PARAM_NOTNULL (s);

	//Remove bad points:
	pointcloud_filter (s->pc1, POINT_STRIDE, s->pc1, POINT_STRIDE, &s->pc_count, POINT_DIM, 1.0f);

	//Move pointcloud to the origin and rotate it:
	//pointcloud_pca requires an extra memory buffer for storing a temporary pointcloud due to nature of matrix-matrix-multiplcation:
	{
		float aux[LIDAR_WH*POINT_STRIDE];
		pointcloud_pca (s->pc1, aux, s->pc_count, POINT_STRIDE, s->centroid, s->w, s->c, s->r);
	}

	/*
	//Clamp points:
	for (uint32_t i = 0; i < LIDAR_WH; ++i)
	{
		s->pc1[i*POINT_STRIDE+2] = CLAMP (s->pc1[i*POINT_STRIDE+2], -0.05f, 0.03f);
	}
	*/

	//Project 3D points to a 2D image:
	//The center of the image is put ontop of the origin where all points are located:
	point_project (s->img1, s->imgf, IMG_XN, IMG_YN, s->pc1, POINT_STRIDE, s->pc_count);



	//Amplify skitrack patterns in the 2D image:
	//https://en.wikipedia.org/wiki/Sobel_operator
	{
		int32_t kxn = 1;
		int32_t kyn = 6;
		//Kernel must sum up to zero!
		float kernel[1*6] =
		{
		-8.0f,
		2.0f,
		6.0f,
		6.0f,
		2.0f,
		-8.0f
		};
		vf32_normalize (kxn*kyn, kernel, kernel);
		vf32_convolution2d (s->img2, s->img1, IMG_XN, IMG_YN, kernel, kxn, kyn);
	}

	//Noise must not get larger than height of skitracks:
	for (uint32_t i = 0; i < IMG_XN*IMG_YN; ++i)
	{
		s->img2[i] = CLAMP (s->img2[i], -0.05f, 0.03f);
	}


	//Smooth filter, do we really need this?:
	{
		int32_t kxn = 3;
		int32_t kyn = 3;
		float kernel[3*3] =
		{
		1.0f, 1.0f, 1.0f,
		1.0f, 2.0f, 1.0f,
		1.0f, 1.0f, 1.0f,
		};
		vf32_normalize (kxn*kyn, kernel, kernel);
		vf32_convolution2d (s->img3, s->img2, IMG_XN, IMG_YN, kernel, kxn, kyn);
	}

	//Find the most common direction in the image which hopefully is parallel to skitracks:
	s->k = vf32_most_common_line2 (s->img3, IMG_XN, IMG_YN, s->q1);

	//Project 2D image to a 1D image in the the most common direction (k):
	vf32_project_2d_to_1d (s->img3, IMG_XN, IMG_YN, s->k, s->q1);

	vf32_remove_low_values (s->q1, IMG_YN);

	//
	for (uint32_t i = 0; i < IMG_YN; ++i)
	{
		s->q2[i] = fabs(s->q1[i]);
	}

	//Slowly change the skitrack memory for new information:
	for (uint32_t i = 0; i < IMG_YN; ++i)
	{
		s->qmem[i] += fabs(s->q1[i]) * 0.15f;
	}

	//Try to create a peak where the skitrack is located:
	{
		float kernel[] = {1.0f,  1.0f,  2.0f, 2.0f,  2.0f,  1.0f,  1.0f};
		vf32_convolution1d (s->q2, IMG_YN, s->q3, kernel, countof (kernel));
	}

	//Apply skitrack memory:
	for (uint32_t i = 3; i < IMG_YN-3; ++i)
	{
		s->q3[i] += s->qmem[i];
	}

	//Remove feet tracks which makes the peak finder ignore feet tracks completly:
	for (uint32_t i = 3; i < IMG_YN-3; ++i)
	{
		if (s->q1[i] < 0.0f)
		{
			s->q3[i] = 0.0f;
		}
	}

	//Find local peaks which is estimated skitracks position:
	{
		float q[IMG_YN] = {0.0f};
		memcpy (q, s->q3, sizeof (q));
		uint32_t peak_u32[SKITRACK2_PEAKS_COUNT];
		vf32_find_peaks (q, IMG_YN, peak_u32, SKITRACK2_PEAKS_COUNT, 16, 20);
		for (uint32_t i = 0; i < SKITRACK2_PEAKS_COUNT; ++i)
		{
			s->peak[i] = CLAMP (peak_u32[i], 10, IMG_YN-10);
		}
	}


	//Memory functionality of the skitrack:
	{
		int32_t o = s->peak[0];//Get index location of the peak 0
		int32_t w = 12;//Memory radius around skitrack
		int32_t a = MAX (o-w, 0);//Start index
		int32_t b = MIN (o+w, IMG_YN);//Stop index
		//Decrease kitrack memory:
		for (int32_t i = 0; i < IMG_YN; ++i)
		{
			s->qmem[i] += -0.03f;
		}
		//Reset memory around skitrack:
		for (int32_t i = a; i < b; ++i)
		{
			if (s->qmem[i] < 0.0f) {s->qmem[i] = 0.0f;}
			s->qmem[i] += 0.001f;
		}
		//Increase skitrack memory:
		for (int32_t i = a; i < b; ++i)
		{
			if (s->qmem[i] < 0.0f) {s->qmem[i] = 0.0f;}
			s->qmem[i] += 0.001f;
		}
		//Clamp skitrack memory:
		for (int32_t i = 0; i < IMG_YN; ++i)
		{
			s->qmem[i] = CLAMP(s->qmem[i], -1.0f, 1.0f);
		}
	}



	//Experiment of subset point to make a local PCA plane:
	{
		int32_t o = s->peak[0];//Skitrack position
		int32_t w = 12;
		int32_t a = MAX (o-w, 0);
		int32_t b = MIN (o+w, IMG_YN);
		s->pc2_count = pointcloud_subset (s->img1, s->pc2, a, b);
	}


	//Exponential Moving Average (EMA) filter:
	/*
	for (uint32_t i = 0; i < SKITRACK2_PEAKS_COUNT; ++i)
	{
		float k = 0.7f;
		s->g1[i] = k * s->g1[i] + (1.0f - k) * (float)s->g[i];
	}
	*/

}
