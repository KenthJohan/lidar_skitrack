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
struct skitrack2
{
	float img1[IMG_XN*IMG_YN];//Projected points
	float img2[IMG_XN*IMG_YN];//Convolution from img1
	float img3[IMG_XN*IMG_YN];//Convolution from img2
	float imgf[IMG_XN*IMG_YN];//Used for normalizing pixel
	float q1[IMG_YN];
	float q2[IMG_YN];
	float q3[IMG_YN];
	float q4[IMG_YN];
	uint32_t g[SKITRACK2_PEAKS_COUNT];
	float g1[SKITRACK2_PEAKS_COUNT];
	float k;
};



static void skitrack2_process (struct skitrack2 * s, float pc[], uint32_t pc_count)
{
	ASSERT_PARAM_NOTNULL (s);

	//Project 3D points to a 2D image:
	//The center of the image is put ontop of the origin where all points are:
	point_project (s->img1, s->imgf, IMG_XN, IMG_YN, pc, POINT_STRIDE, pc_count);


	//Amplify skitrack pattern in the 2D image:
	{
		int32_t kxn = 1;
		int32_t kyn = 5;
		float kernel[1*5] =
		{
		-5.0f,
		2.0f,
		6.0f,
		2.0f,
		-5.0f
		};
		vf32_normalize (kxn*kyn, kernel, kernel);
		vf32_convolution2d (s->img2, s->img1, IMG_XN, IMG_YN, kernel, kxn, kyn);
	}



	//vf32_remove_low_values (img2, IMG_XN*IMG_YN);


	//Smooth filter:
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


	//vf32_remove_low_values (img3, IMG_XN*IMG_YN);
	//memcpy (img3, img2, sizeof(img3));



	//Find the most common lines direction in the image which hopefully matches the direction of the skitrack:
	//Project 2D image to a 1D image in the the most common direction (k):

	//float k = vf32_most_common_line (img3, IMG_XN, IMG_YN, 20);
	s->k = vf32_most_common_line2 (s->img3, IMG_XN, IMG_YN, s->q1);
	//vf32_project_2d_to_1d (img3, IMG_XN, IMG_YN, k, q1);
	vf32_project_2d_to_1d_pn (s->img3, IMG_XN, IMG_YN, s->k, s->q1);
	vf32_remove_low_values (s->q1, IMG_YN);
	for (uint32_t i = 0; i < IMG_YN; ++i)
	{
		s->q1[i] = s->q1[i] * s->q1[i] * 1.0f;
	}
	float skitrack_kernel1d_a[] =
	{
	 1.0f,  1.0f,  2.0f, 2.0f,  2.0f,  1.0f,  1.0f
	};
	vf32_convolution1d (s->q1, IMG_YN, s->q2, skitrack_kernel1d_a, countof (skitrack_kernel1d_a));

	/*
	//Amplify skitrack pattern in the 1D image:
	float skitrack_kernel1d[] =
	{
	 1.0f,  3.0f,  1.0f,
	-3.0f, -9.0f, -3.0f,
	 1.0f,  14.0f,  1.0f,
	-3.0f, -9.0f, -3.0f,
	 1.0f,  3.0f,  1.0f
	};
	vf32_convolution1d (s->q1, IMG_YN, s->q2, skitrack_kernel1d, countof (skitrack_kernel1d));
	*/






	//vf32_weight_ab (IMG_YN, s->q3, s->q3, s->q2, 0.5f);


	//Find the peaks which should be where the skitrack is positioned:
	{
		float q[IMG_YN] = {0.0f};
		memcpy (q, s->q2, sizeof (q));
		vf32_find_peaks (q, IMG_YN, s->g, SKITRACK2_PEAKS_COUNT, 16, 20);
	}



	for (uint32_t i = 0; i < SKITRACK2_PEAKS_COUNT; ++i)
	{
		float k = 0.7f;
		s->g1[i] = k * s->g1[i] + (1.0f - k) * (float)s->g[i];
	}



	//vf32_normalize (countof (q1), q1, q1);
	//vf32_normalize (countof (q2), q2, q2);
}
