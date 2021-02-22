#pragma once

#include <stdint.h>
#include "csc/csc_debug.h"
#include "csc/csc_m3f32.h"

#include "../shared/shared.h"
#include "mathmisc.h"

#define SKITRACK2_PEAKS_COUNT 2


struct skitrack2
{
	float img1[IMG_XN*IMG_YN];//Projected points
	float img2[IMG_XN*IMG_YN];//Convolution from img1
	float img3[IMG_XN*IMG_YN];//Convolution from img2
	float imgf[IMG_XN*IMG_YN];//Used for normalizing pixel
	float q1[IMG_YN];
	float q2[IMG_YN];
	uint32_t g[SKITRACK2_PEAKS_COUNT];
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


	//Amplify skitrack pattern in the 1D image:
	float skitrack_kernel1d[] =
	{
	 1.0f,  3.0f,  1.0f,
	-3.0f, -9.0f, -3.0f,
	 1.0f,  7.0f,  1.0f,
	-3.0f, -9.0f, -3.0f,
	 1.0f,  3.0f,  1.0f
	};
	vf32_convolution1d (s->q1, IMG_YN, s->q2, skitrack_kernel1d, countof (skitrack_kernel1d));


	//Find the peaks which should be where the skitrack is positioned:
	{
		float q[IMG_YN] = {0.0f};
		memcpy (q, s->q2, sizeof (q));
		vf32_find_peaks (q, IMG_YN, s->g, SKITRACK2_PEAKS_COUNT, 16, 20);
	}


	//vf32_normalize (countof (q1), q1, q1);
	//vf32_normalize (countof (q2), q2, q2);
}
