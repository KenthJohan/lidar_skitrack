#pragma once

#include <stdio.h>

#include "csc/csc_debug_nng.h"
#include "csc/csc_math.h"
#include "csc/csc_linmat.h"
#include "csc/csc_m3f32.h"

#include "../shared/shared.h"
#include "points_read.h"
#include "mathmisc.h"

#include "mg_attr.h"
#include "mg_comp.h"
#include "mg_send.h"
#include "myent.h"

#include "skitrack1.h"
#include "skitrack2.h"




struct gobj_line
{
	nng_socket sock;
	uint32_t cap;
	uint32_t last;
	float * lines;//Stride=4
};

void gobj_line_push (struct gobj_line * obj, float x, float y, float z)
{
	float * lines = obj->lines + obj->last * 4;
	lines[0] = x;
	lines[1] = y;
	lines[2] = z;
	lines[3] = 1.0f;
	obj->last++;
}


void gobj_line_send (struct gobj_line * obj)
{
	int r;
	r = nng_send (obj->sock, obj->lines, obj->last*4*sizeof(float), 0);
	if (r)
	{
		perror (nng_strerror (r));
	}
}





uint32_t rgba_value (float value, float kr, float kg, float kb)
{
	uint32_t r = CLAMP (value*kr, 0.0f, 255.0f);
	uint32_t g = CLAMP (value*kg, 0.0f, 255.0f);
	uint32_t b = CLAMP (value*kb, 0.0f, 255.0f);
	return RGBA (r, g, b, 0xFF);
}



/**
 * @brief Create RGBA image visualisation
 * @param[out] img  RGBA image visual
 * @param[in]  pix  Grayscale image
 * @param[in]  w    Width of the image
 * @param[in]  h    Height of the image
 */
static void image_visual (uint32_t img[], float pix[], uint32_t xn, uint32_t yn, float q1[], float q2[], uint32_t g[], uint32_t m, float k)
{
	//Negatives becomes red and positives becomes greeen:
	for (uint32_t i = 0; i < xn*yn; ++i)
	{
		img[i] = rgba_value (pix[i], -3000.0f, 3000.0f, 0.0f);
	}


	for (uint32_t y = 0; y < yn; ++y)
	{
		img[y*xn+1] = rgba_value (q1[y], -100.0f, 100.0f, 0.0f);
		img[y*xn+0] = rgba_value (q2[y], -100.0f, 100.0f, 0.0f);
	}


	for (uint32_t y = 0; y < yn; ++y)
	{
		if (q1[y] == 0)
		{
			img[y*xn+1] = RGBA (0xAA, 0xAA, 0x00, 0xFF);
		}
		if (q2[y] == 0)
		{
			img[y*xn+0] = RGBA (0xAA, 0xAA, 0x00, 0xFF);
		}
	}


	for (uint32_t i = 0; i < m; ++i)
	{
		if (g[i] < yn)
		{
			uint32_t y = g[i];
			for (uint32_t x = 0; x < xn; ++x)
			{
				float yy = (float)y + (float)x*k;
				if (yy < 0.0f){continue;}
				if (yy >= (float)yn){continue;}
				ASSERT (yy >= 0);
				ASSERT (yy < (float)yn);
				uint32_t index = (uint32_t)yy * xn + x;
				ASSERT (index < xn*yn);
				img[index] |= RGBA (0x00, 0x00, 0xFF, 0xFF);
			}
		}
	}
}








enum visual_line
{
	VISUAL_LINE_ORIGIN_0,
	VISUAL_LINE_ORIGIN_1,
	VISUAL_LINE_ORIGIN_2,
	VISUAL_LINE_PCA_0,
	VISUAL_LINE_PCA_1,
	VISUAL_LINE_PCA_2,
	VISUAL_LINE_SKITRACK,
	VISUAL_LINE_SKITRACK_END = VISUAL_LINE_SKITRACK + SKITRACK2_PEAKS_COUNT - 1,
	VISUAL_LINE_COUNT
};




#define VISUAL_MODE_IMG_MASK UINT32_C(0x0000000F)
#define VISUAL_MODE_IMG1     UINT32_C(0x00000001)
#define VISUAL_MODE_IMG2     UINT32_C(0x00000002)
#define VISUAL_MODE_IMG3     UINT32_C(0x00000003)



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
void show (const char * filename, nng_socket sock, uint32_t visual_mode)
{
	struct skitrack1 s1 = {0};
	struct skitrack2 s2 = {0};

	float pointpos[LIDAR_WH*POINT_STRIDE*2];
	uint32_t pointcol[LIDAR_WH*2] = {RGBA (0xFF, 0xFF, 0xFF, 0xFF)};//The color of each point. This is only used for visualization.
	uint32_t imgv[IMG_XN*IMG_YN] = {0};//Used for visual confirmation that the algorithm works


	points_read_filename (filename, s1.pc, &s1.pc_count);
	//points_test_sinus_slope (s1.pc);

	memcpy (pointpos, s1.pc, LIDAR_WH*POINT_STRIDE*sizeof(float));
	skitrack1_process (&s1);
	skitrack2_process (&s2, s1.pc, s1.pc_count);
	memcpy (pointpos + LIDAR_WH*POINT_STRIDE, s1.pc, LIDAR_WH*POINT_STRIDE*sizeof(float));

	//Visualize the skitrack and more information:
	switch (visual_mode & VISUAL_MODE_IMG_MASK)
	{
	case VISUAL_MODE_IMG1:
		image_visual (imgv, s2.img1, IMG_XN, IMG_YN, s2.q1, s2.q2, s2.g, SKITRACK2_PEAKS_COUNT, s2.k);
		break;
	case VISUAL_MODE_IMG2:
		image_visual (imgv, s2.img2, IMG_XN, IMG_YN, s2.q1, s2.q2, s2.g, SKITRACK2_PEAKS_COUNT, s2.k);
		break;
	case VISUAL_MODE_IMG3:
		image_visual (imgv, s2.img3, IMG_XN, IMG_YN, s2.q1, s2.q2, s2.g, SKITRACK2_PEAKS_COUNT, s2.k);
		break;
	}



	//pix_rgba[105*IMG_XN + 12] |= RGBA(0x00, 0x66, 0x00, 0x00);
	//pix_rgba[0*IMG_XN + 1] |= RGBA(0x00, 0xFF, 0x00, 0xFF);
	//pix_rgba[2*IMG_XN + 0] |= RGBA(0x00, 0xFF, 0xff, 0xFF);
	//pix_rgba[2*IMG_XN + 1] |= RGBA(0x00, 0xFF, 0xff, 0xFF);

	struct v4f32_line linepos1[VISUAL_LINE_COUNT];
	struct u32_line linecol1[VISUAL_LINE_COUNT];

	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_0].a, 0.0f, 0.0f, 0.0f);
	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_0].b, 1.0f, 0.0f, 0.0f);
	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_1].a, 0.0f, 0.0f, 0.0f);
	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_1].b, 0.0f, 1.0f, 0.0f);
	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_2].a, 0.0f, 0.0f, 0.0f);
	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_2].b, 0.0f, 0.0f, 1.0f);

	vf32_set3 (linepos1[VISUAL_LINE_PCA_0].a, 0.0f   , 0.0f   , 0.0f   );
	vf32_set3 (linepos1[VISUAL_LINE_PCA_0].b, s1.c[0], s1.c[1], s1.c[2]);
	vf32_set3 (linepos1[VISUAL_LINE_PCA_1].a, 0.0f   , 0.0f   , 0.0f   );
	vf32_set3 (linepos1[VISUAL_LINE_PCA_1].b, s1.c[3], s1.c[4], s1.c[5]);
	vf32_set3 (linepos1[VISUAL_LINE_PCA_2].a, 0.0f   , 0.0f   , 0.0f   );
	vf32_set3 (linepos1[VISUAL_LINE_PCA_2].b, s1.c[6], s1.c[7], s1.c[8]);




	linecol1[VISUAL_LINE_ORIGIN_0].a = RGBA(0xFF, 0x00, 0x00, 0xFF);
	linecol1[VISUAL_LINE_ORIGIN_0].b = RGBA(0xFF, 0x00, 0x00, 0xFF);
	linecol1[VISUAL_LINE_ORIGIN_1].a = RGBA(0x00, 0xFF, 0x00, 0xFF);
	linecol1[VISUAL_LINE_ORIGIN_1].b = RGBA(0x00, 0xFF, 0x00, 0xFF);
	linecol1[VISUAL_LINE_ORIGIN_2].a = RGBA(0x00, 0x00, 0xFF, 0xFF);
	linecol1[VISUAL_LINE_ORIGIN_2].b = RGBA(0x00, 0x00, 0xFF, 0xFF);

	linecol1[VISUAL_LINE_PCA_0].a = RGBA(0xFF, 0x00, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
	linecol1[VISUAL_LINE_PCA_0].b = RGBA(0xFF, 0x00, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
	linecol1[VISUAL_LINE_PCA_1].a = RGBA(0x00, 0xFF, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
	linecol1[VISUAL_LINE_PCA_1].b = RGBA(0x00, 0xFF, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
	linecol1[VISUAL_LINE_PCA_2].a = RGBA(0x00, 0x00, 0xFF, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
	linecol1[VISUAL_LINE_PCA_2].b = RGBA(0x00, 0x00, 0xFF, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);

	for (int i = VISUAL_LINE_SKITRACK; i <= VISUAL_LINE_SKITRACK_END; ++i)
	{
		linecol1[i].a = RGBA(0x77, 0xFF, 0x11, 0xFF);
		linecol1[i].b = RGBA(0x77, 0xFF, 0x11, 0xFF);
	}

	{
		pixel_to_point (linepos1[VISUAL_LINE_SKITRACK+0].a, IMG_XN, IMG_YN, 0.0f, -10.0f, s2.g[0] - 10.0f * s2.k);
		pixel_to_point (linepos1[VISUAL_LINE_SKITRACK+0].b, IMG_XN, IMG_YN, 0.0f,  30.0f, s2.g[0] + 30.0f * s2.k);
		pixel_to_point (linepos1[VISUAL_LINE_SKITRACK+1].a, IMG_XN, IMG_YN, 0.0f, -10.0f, s2.g[1] - 10.0f * s2.k);
		pixel_to_point (linepos1[VISUAL_LINE_SKITRACK+1].b, IMG_XN, IMG_YN, 0.0f,  30.0f, s2.g[1] + 30.0f * s2.k);
		float rot[3*3];
		memcpy (rot, s1.r, sizeof (rot));
		m3f32_lapacke_inverse (rot, 3);
		for (float * i = linepos1[VISUAL_LINE_SKITRACK+0].a; i <= linepos1[VISUAL_LINE_SKITRACK_END].b; i += POINT_STRIDE)
		{
			mv3f32_mul (i, rot, i);
			vvf32_add (4, i, i, s1.centroid);
		}
	}



	//Send visual information to the graphic server:
	{
		for (int i = 0; i < LIDAR_WH*2; ++i)
		{
			//pointpos[i + 0] = 10.0f;
			//pointpos[i + 1] = 10.0f;
			//pointpos[i + 2] = 10.0f;
			pointpos[i*POINT_STRIDE + 3] = 10.0f;
		}

		mg_send_add (sock, MYENT_MESH_RECTANGLE, MG_MESH);
		mg_send_add (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD);

		mg_send_set (sock, MYENT_TEXTURE1, MG_TEXTURE, &(component_texture){0, IMG_XN, IMG_YN, 1}, sizeof(component_texture));
		mg_send_set (sock, MYENT_TEXTURE2, MG_TEXTURE, &(component_texture){0, IMG_XN, IMG_YN, 1}, sizeof(component_texture));
		mg_send_set (sock, MYENT_TEXTURE1, MG_TEXTURE_CONTENT, imgv, IMG_XN*IMG_YN*sizeof(uint32_t));

		mg_send_set (sock, MYENT_MESH_RECTANGLE, MG_COUNT, &(component_count){6}, sizeof(component_count));
		mg_send_set (sock, MYENT_MESH_RECTANGLE, MG_RECTANGLE, &(component_rectangle){IMG_XN*IMG_SCALE, IMG_YN*IMG_SCALE}, sizeof(component_rectangle));

		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_COUNT, &(component_count){LIDAR_WH*2}, sizeof(component_count));
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_POS, pointpos, LIDAR_WH*POINT_STRIDE*sizeof(float)*2);


		{
			float m[4*4] = {0.0f};
			//m4f32_identity (m);
			m4f32_translation (m, s1.centroid);
			m[M4_02] = s1.c[0];//Column 2
			m[M4_12] = s1.c[1];//Column 2
			m[M4_22] = s1.c[2];//Column 2
			m[M4_00] = s1.c[3];//Column 0
			m[M4_10] = s1.c[4];//Column 0
			m[M4_20] = s1.c[5];//Column 0
			m[M4_01] = s1.c[6];//Column 1
			m[M4_11] = s1.c[7];//Column 1
			m[M4_21] = s1.c[8];//Column 1
			m[M4_33] = 1.0f;
			m4f32_print(m, stdout);
			mg_send_set (sock, MYENT_DRAW_IMG1, MG_TRANSFORM, m, sizeof (component_transform));
			mg_send_set (sock, MYENT_DRAW_IMG1, MG_ADD_INSTANCEOF, &(uint32_t){MYENT_MESH_RECTANGLE}, sizeof (uint32_t));
			mg_send_set (sock, MYENT_DRAW_IMG1, MG_ADD_INSTANCEOF, &(uint32_t){MYENT_TEXTURE1}, sizeof (uint32_t));
		}


		{
			mg_send_add (sock, MYENT_DRAW_IMG2, MG_TRANSFORM);
			mg_send_set (sock, MYENT_DRAW_IMG2, MG_POSITION,&(component_position){0.0f, 0.0f, 0.0f, 1.0f}, sizeof (component_position));
			mg_send_set (sock, MYENT_DRAW_IMG2, MG_SCALE, &(component_position){1.0f, 1.0f, 0.0f, 1.0f}, sizeof (component_position));
			mg_send_set (sock, MYENT_DRAW_IMG2, MG_QUATERNION, &(component_position){0.0f, 0.0f, 0.0f, 1.0f}, sizeof (component_position));
			mg_send_set (sock, MYENT_DRAW_IMG2, MG_ADD_INSTANCEOF, &(uint32_t){MYENT_MESH_RECTANGLE}, sizeof (uint32_t));
			mg_send_set (sock, MYENT_DRAW_IMG2, MG_ADD_INSTANCEOF, &(uint32_t){MYENT_TEXTURE1}, sizeof (uint32_t));
		}

		/*
		r = nng_send (socks[MAIN_NNGSOCK_LINE_POS], linepos1, VISUAL_LINE_COUNT*POINT_STRIDE*2*sizeof(float), 0);
		if (r) {perror (nng_strerror (r));}
		r = nng_send (socks[MAIN_NNGSOCK_LINE_COL], linecol1, VISUAL_LINE_COUNT*2*sizeof(uint32_t), 0);
		if (r) {perror (nng_strerror (r));}
		r = nng_send (socks[MAIN_NNGSOCK_POINTCLOUD_POS], pointpos, LIDAR_WH*POINT_STRIDE*sizeof(float)*2, 0);
		if (r) {perror (nng_strerror (r));}
		r = nng_send (socks[MAIN_NNGSOCK_POINTCLOUD_COL], pointcol, LIDAR_WH*sizeof(uint32_t)*2, 0);
		if (r) {perror (nng_strerror (r));}
		r = nng_send (socks[MAIN_NNGSOCK_GROUNDPROJECTION], imgv, IMG_XN*IMG_YN*sizeof(uint32_t), 0);
		if (r) {perror (nng_strerror (r));}
		*/
	}

}
