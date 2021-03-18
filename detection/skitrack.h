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


#define SKITRACK2_PEAKS_COUNT 1
#define SKITRACK_STRENGHT_THRESHOLD 0.8f
#define SKITRACK_POINTCLOUD_COUNT_THRESHOLD (LIDAR_WH*2/3)
#define SKITRACK_POINTPLANE_COUNT_THRESHOLD (LIDAR_WH*2/3)
#define SKITRACK_POINTCLOUD_THICKNESS_THRESHOLD  0.001f
#define SKITRACK_POINTCLOUD_THICKNESS_THRESHOLD2  0.010f
#define SKITRACK_NEAR_THRESHOLD 2.3f
#define SKITRACK_NEARCOUNT_THRESHOLD 10
#define SKITRACK_MIN_Z -0.1f
#define SKITRACK_MAX_Z 0.1f






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
	v4f32 pc1[LIDAR_WH];//All points of pointcloud (x,y,z,a),(x,y,z,a)
	float w[3];//Eigen values
	float covk;
	float c[3*3];//Covariance matrix
	float c1[3*3];//Covariance matrix
	float e[3*3];//3x eigen vectors
	float r[3*3];//Rotation matrix

	float centroid_k;
	float centroid[3];//Center point of pointcloud

	//2D images:
	float imgf[IMG_XN*IMG_YN];//Used for normalizing pixel
	float imgm[IMG_XN*IMG_YN];//Used for normalizing pixel
	float img1[IMG_XN*IMG_YN];//From projected points
	float img2[IMG_XN*IMG_YN];//Convolution from img1
	float img3[IMG_XN*IMG_YN];//Convolution from img2

	//1D images:
	float q1[IMG_YN];
	float q2[IMG_YN];
	float q3[IMG_YN];
	float qmem[IMG_YN];

	//Peaks index locations:
	uint32_t peak_u32[SKITRACK2_PEAKS_COUNT];
	uint32_t peakg_u32[SKITRACK2_PEAKS_COUNT];
	float trackpos[SKITRACK2_PEAKS_COUNT];

	//Skitrack direction (x, x + k*y):
	float k;
	float kg;

	float strength;

	uint32_t pointplanecount;
	uint32_t nearcount;
};



static void skitrack_rectify (struct skitrack * s)
{
	ASSERT_PARAM_NOTNULL (s);


	{
		v4f32 x[LIDAR_WH];
		v3f32 centroid;
		m3f32 c;
		m3f32 e;
		v3f32 w;
		float k = 1.0f;
		memcpy(x, s->pc1, sizeof (x));
		skitrack_centering (x, x, s->pc_count, k, centroid);
		skitrack_cov (x, s->pc_count, c, k);
		skitrack_eigen (c, e, w);
		skitrack_conditional_basis_flip (e);
		skitrack_reorder_eigen (e, w);
		csc_v3f32_print_rgb (stdout, w);
		csc_v3f32_print_rgb (stdout, centroid);
		//printf ("%f %f\n", w[2], 0.0006f);
		if (w[2] < SKITRACK_POINTCLOUD_THICKNESS_THRESHOLD)
		{
			s->covk = 1.0f;
			s->centroid_k = 1.0f;
		}
		else if (w[2] < SKITRACK_POINTCLOUD_THICKNESS_THRESHOLD2)
		{
			s->covk = 0.01f;
			s->centroid_k = 0.01f;
		}
		else
		{
			s->covk = 0.0f;
			s->centroid_k = 0.0f;
		}


		if (s->pc_count < SKITRACK_POINTCLOUD_COUNT_THRESHOLD)
		{
			s->covk = 0.0f;
			s->centroid_k = 0.0f;
		}
	}

	//Move pointcloud to the origin and rotate it:
	//pointcloud_pca requires an extra memory buffer for storing a temporary pointcloud due to nature of matrix-matrix-multiplcation:
	{
		float aux[LIDAR_WH*POINT_STRIDE];
		pointcloud_pca (s->pc1, aux, s->pc_count, POINT_STRIDE, s->centroid, s->centroid_k, s->w, s->c, s->e, s->covk);
		//csc_m3f32_print_rgb (stdout, s->c1);
		//csc_v3f32_print_rgb (stdout, s->centroid1);
	}
}





static void skitrack_project (struct skitrack * s)
{
	memset (s->img1, 0, sizeof(float) * IMG_XN * IMG_YN);
	memset (s->imgf, 0, sizeof(float) * IMG_XN * IMG_YN);
	s->pointplanecount = 0;
	for (uint32_t i = 0; i < s->pc_count; ++i)
	{
		//v[2] += 1.0f;
		//(x,y) becomes the pixel position:
		//Set origin in the middle of the image and 20 pixels becomes 1 meter:
		//z-value becomes the pixel value:
		float x;
		float y;
		float z;
		point_to_pixel (s->pc1[i], IMG_XN, IMG_YN, &z, &x, &y);
		//Crop the pointcloud to the size of the image;
		if (x >= IMG_XN){continue;}
		if (y >= IMG_YN){continue;}
		if (x < 0){continue;}
		if (y < 0){continue;}
		if ((z > (SKITRACK_MIN_Z*2)) && (z < (SKITRACK_MAX_Z*2)))
		{
			s->pointplanecount++;
		}
		if (z < SKITRACK_MIN_Z){continue;}
		if (z > SKITRACK_MAX_Z){continue;}
		//Convert (x,y) to index row-major:
		uint32_t index = ((uint32_t)y * IMG_XN) + (uint32_t)x;
		//z += 10.0f;
		//If multiple points land on one pixel then it will be accumalted but it will also be normalized later on:
		s->img1[index] += z;
		s->imgf[index] += 1.0f;
		//pix[index] = 0.5f*pix[index] + 0.5f*z;
	}

	//Normalize every non zero pixel:
	for (uint32_t i = 0; i < IMG_XN*IMG_YN; ++i)
	{
		if (s->imgf[i] > 0.0f)
		{
			s->img1[i] /= s->imgf[i];
		}
	}

	for (uint32_t i = 0; i < IMG_XN*IMG_YN; ++i)
	{
		//Gradient convolution could be applied later so this statement will have no effect:
		//It is important that this statement does not affect the end result:
		//This statement test scenories where average pointcloud z-position is far of origin:
		//pix[i] += 10.0f;
	}
}





static void skitrack_process (struct skitrack * s)
{
	ASSERT_PARAM_NOTNULL (s);

	//Remove bad points:
	{
		s->pc_count = pointcloud_filter (s->pc1, POINT_STRIDE, s->pc1, POINT_STRIDE, s->pc_count, POINT_DIM, 0.1f);
		uint32_t count = pointcloud_filter (s->pc1, POINT_STRIDE, s->pc1, POINT_STRIDE, s->pc_count, POINT_DIM, SKITRACK_NEAR_THRESHOLD*SKITRACK_NEAR_THRESHOLD);
		s->nearcount = s->pc_count - count;
		s->pc_count = count;
	}


	if (s->pc_count > 100)
	{
		skitrack_rectify (s);
	}

	//Project 3D points to a 2D image:
	//The center of the image is put ontop of the origin where all points are located:
	skitrack_project (s);



	{
		int32_t kxn = 1;
		int32_t kyn = 15;
		for (uint32_t i = 0; i < IMG_XN*IMG_YN; ++i)
		{
			s->imgf[i] = CLAMP (s->imgf[i], 0.0f, 1.0f);
		}
		vf32_convolution2d_clean (s->imgm, s->imgf, IMG_XN, IMG_YN, kxn, kyn);
	}


	//Amplify skitrack patterns in the 2D image:
	//https://en.wikipedia.org/wiki/Sobel_operator
	{
		int32_t kxn = 1;
		int32_t kyn = 15;
		//Kernel must sum up to zero!
		float kernel[1*15] =
		{
		0.5f,
		1.0f,
		0.5f,
		-0.5f,
		-2.0f,
		-0.5f,
		0.5f,
		1.0f,
		0.5f,
		-0.5f,
		-2.0f,
		-0.5f,
		0.5f,
		1.0f,
		0.5f,
		};
		vf32_normalize (kxn*kyn, kernel, kernel);
		//vf32_convolution2d (s->img2, s->img1, IMG_XN, IMG_YN, kernel, kxn, kyn);
		vf32_convolution2d_masked (s->img2, s->img1, s->imgm, IMG_XN, IMG_YN, kernel, kxn, kyn);
	}

	//Noise must not get larger than height of skitracks:
	for (uint32_t i = 0; i < IMG_XN*IMG_YN; ++i)
	{
		//s->img2[i] = CLAMP (s->img2[i], -0.05f, 0.03f);
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
		//vf32_convolution2d_masked (s->img3, s->img2, s->imgm, IMG_XN, IMG_YN, kernel, kxn, kyn);
	}

	//Find the most common direction in the image which hopefully is parallel to skitracks:
	{
		float q[IMG_YN] = {0.0f};
		s->k = vf32_most_common_line2 (s->img3, IMG_XN, IMG_YN, q);
	}

	//Project 2D image to a 1D image in the the most common direction (k):
	vf32_project_2d_to_1d (s->img3, IMG_XN, IMG_YN, s->k, s->q1);

	vf32_remove_low_values (s->q1, IMG_YN);

	//Absolute
	for (uint32_t i = 0; i < IMG_YN; ++i)
	{
		s->q2[i] = fabs(s->q1[i]);
	}


	//Try to create a peak where the skitrack is located:
	{
		float kernel[] = {1.0f,  1.0f,  1.0f, 1.0f,  1.0f,  1.0f,  1.0f};
		vf32_normalize (countof (kernel), kernel, kernel);
		vf32_convolution1d (s->q2, IMG_YN, s->q3, kernel, countof (kernel));
		memcpy (s->q2, s->q3, sizeof (float) * IMG_YN);
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
		vf32_find_peaks (q, IMG_YN, s->peak_u32, SKITRACK2_PEAKS_COUNT, 16, 20);
		for (uint32_t i = 0; i < SKITRACK2_PEAKS_COUNT; ++i)
		{
			s->peak_u32[i] = CLAMP (s->peak_u32[i], 10, IMG_YN-10);
			s->strength = s->q2[s->peak_u32[i]];
			s->trackpos[i] = ((float)s->peak_u32[i] - (float)IMG_YN/2.0f) * (float)IMG_SCALE;
		}
	}



	//Decrease skitrack memory:
	for (int32_t i = 0; i < IMG_YN; ++i)
	{
		s->qmem[i] *= 0.9f;
	}

	//Memory functionality of the skitrack:
	if ((s->covk > 0.0f) && (s->strength > SKITRACK_STRENGHT_THRESHOLD))
	{
		vu32_cpy (SKITRACK2_PEAKS_COUNT, s->peakg_u32, s->peak_u32);
		s->kg = s->k;


		int32_t o = s->peak_u32[0];//Get index location of the peak 0
		int32_t w = 12;//Memory radius around skitrack
		int32_t a = MAX (o-w, 0);//Start index
		int32_t b = MIN (o+w, IMG_YN);//Stop index

		//Reset memory around skitrack:
		for (int32_t i = a; i < b; ++i)
		{
			s->qmem[i] = 2.0f;
		}

		//Slowly change the skitrack memory for new information:
		for (uint32_t i = 0; i < IMG_YN; ++i)
		{
			s->qmem[i] += s->q2[i] * 0.15f;
		}
	}





	for (uint32_t i = 0; i < SKITRACK2_PEAKS_COUNT; ++i)
	{
		s->trackpos[i] = ((float)s->peakg_u32[i] - (float)IMG_YN/2.0f) * (float)IMG_SCALE;
	}

}


static void skitrack_print_info (struct skitrack * s)
{
	printf ("[INFO] Pointcount: %i of %i (th=%i)\n", s->pc_count, LIDAR_WH, SKITRACK_POINTCLOUD_COUNT_THRESHOLD);
	printf ("[INFO] Pointplanecount: %i of %i (th=%i)\n", s->pointplanecount, s->pc_count, SKITRACK_POINTPLANE_COUNT_THRESHOLD);
	printf ("[INFO] Nearcount: %i of %i (th=%i)\n", s->nearcount, s->pc_count, SKITRACK_NEARCOUNT_THRESHOLD);
	printf ("[INFO] Trackpos: %fm\n", s->trackpos[0]);
	printf ("[INFO] covk: %3.0f%%\n", s->covk*100.0f);
	printf ("[INFO] Strength: [%i] %f (th=%3.3f)\n", s->peak_u32[0], s->strength, SKITRACK_STRENGHT_THRESHOLD);
}
