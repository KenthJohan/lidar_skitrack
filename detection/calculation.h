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


#define VISUAL_MODE_IMG_MASK          UINT32_C(0x0000000F)
#define VISUAL_MODE_IMG1              UINT32_C(0x00000001)
#define VISUAL_MODE_IMG2              UINT32_C(0x00000002)
#define VISUAL_MODE_IMG3              UINT32_C(0x00000003)
#define VISUAL_MODE_VERBOOSE_MASK     UINT32_C(0x000000F0)
#define VISUAL_MODE_VERBOOSE1         UINT32_C(0x00000010)

#define IMG3_XN 4


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

}


static void draw_skitrack(struct skitrack2 * s, uint32_t img[])
{
	for (uint32_t i = 0; i < SKITRACK2_PEAKS_COUNT; ++i)
	{
		if (s->g1[i] < IMG_YN)
		{
			uint32_t y = s->g3[i];
			for (uint32_t x = 0; x < IMG_XN; ++x)
			{
				float yy = (float)y + (float)x * s->k;
				if (yy < 0.0f){continue;}
				if (yy >= (float)IMG_YN){continue;}
				ASSERT (yy >= 0);
				ASSERT (yy < (float)IMG_YN);
				uint32_t index = (uint32_t)yy * IMG_XN + x;
				ASSERT (index < IMG_XN*IMG_YN);
				img[index] |= RGBA (0x00, 0x00, 0xFF, 0xFF);
			}
		}
	}
}


static void show_img2 (nng_socket sock, struct skitrack1 * s1, uint32_t flags)
{
	float m[4*4] = {0.0f};
	//m4f32_identity (m);
	m4f32_translation (m, s1->centroid);
	m[M4_02] = s1->c[0];//Column 2
	m[M4_12] = s1->c[1];//Column 2
	m[M4_22] = s1->c[2];//Column 2
	m[M4_00] = s1->c[3];//Column 0
	m[M4_10] = s1->c[4];//Column 0
	m[M4_20] = s1->c[5];//Column 0
	m[M4_01] = s1->c[6];//Column 1
	m[M4_11] = s1->c[7];//Column 1
	m[M4_21] = s1->c[8];//Column 1
	m[M4_33] = 1.0f;
	if (flags & VISUAL_MODE_VERBOOSE1)
	{
		printf ("[INFO] Affine matrix of img2:\n");
		m4f32_print (m, stdout);
	}
	mg_send_set (sock, MYENT_DRAW_IMG2, MG_TRANSFORM, m, sizeof (component_transform));
}


static void show_img4 (nng_socket sock, struct skitrack2 * s2)
{
	uint32_t imgv[IMG3_XN*IMG_YN] = {0};//Used for visual confirmation that the algorithm works

	for (uint32_t y = 0; y < IMG_YN; ++y)
	{
		imgv[y+IMG_YN*0] = rgba_value (s2->q1[y], -100.0f, 100.0f, 0.0f);
		imgv[y+IMG_YN*1] = rgba_value (s2->q2[y], -100.0f, 100.0f, 0.0f);
		imgv[y+IMG_YN*2] = rgba_value (s2->q3[y], -100.0f, 100.0f, 0.0f);
		imgv[y+IMG_YN*3] = rgba_value (s2->q4[y], -100.0f, 100.0f, 0.0f);
		/*
		if (s2->q1[y] == 0)
		{
			imgv[y+IMG_YN*0] = RGBA (0xAA, 0xAA, 0x00, 0xFF);
		}
		if (s2->q2[y] == 0)
		{
			imgv[y+IMG_YN*1] = RGBA (0xAA, 0xAA, 0x00, 0xFF);
		}
		*/
	}


	mg_send_set (sock, MYENT_TEXTURE2, MG_TEXTURE_CONTENT, imgv, IMG3_XN*IMG_YN*sizeof(uint32_t));
}


static void show_init (nng_socket sock)
{

	mg_send_add (sock, MYENT_MESH_RECTANGLE, MG_MESH);
	mg_send_add (sock, MYENT_MESH_RECTANGLE2, MG_MESH);
	mg_send_add (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD);
	mg_send_add (sock, MYENT_DRAW_LINES, MG_LINES);


	{
		component_texture t1 = {0, IMG_XN, IMG_YN, 1};
		component_texture t2 = {0, IMG_YN, IMG3_XN, 1};
		mg_send_set (sock, MYENT_TEXTURE1, MG_TEXTURE, &t1, sizeof(component_texture));
		mg_send_set (sock, MYENT_TEXTURE2, MG_TEXTURE, &t2, sizeof(component_texture));
	}

	{
		component_count c = 6;
		component_rectangle r = {IMG_XN*IMG_SCALE, IMG_YN*IMG_SCALE};
		mg_send_set (sock, MYENT_MESH_RECTANGLE, MG_COUNT, &c, sizeof(component_count));
		mg_send_set (sock, MYENT_MESH_RECTANGLE, MG_RECTANGLE, &r, sizeof(component_rectangle));
	}

	{
		component_count c = 6;
		component_rectangle r = {IMG_YN*IMG_SCALE, IMG3_XN*IMG_SCALE};
		mg_send_set (sock, MYENT_MESH_RECTANGLE2, MG_COUNT, &c, sizeof(component_count));
		mg_send_set (sock, MYENT_MESH_RECTANGLE2, MG_RECTANGLE, &r, sizeof(component_rectangle));
	}

	{
		component_count c = LIDAR_WH*2;
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_COUNT, &c, sizeof(component_count));
	}

	{
		component_position p = {0.0f, 0.0f, 0.0f, 1.0f};
		component_position s = {1.0f, 1.0f, 0.0f, 1.0f};
		component_position q = {0.0f, 0.0f, 0.0f, 1.0f};
		uint32_t mesh = MYENT_MESH_RECTANGLE;
		uint32_t texture = MYENT_TEXTURE1;
		mg_send_add (sock, MYENT_DRAW_IMG1, MG_TRANSFORM);
		mg_send_set (sock, MYENT_DRAW_IMG1, MG_POSITION, p, sizeof (component_position));
		mg_send_set (sock, MYENT_DRAW_IMG1, MG_SCALE, s, sizeof (component_position));
		mg_send_set (sock, MYENT_DRAW_IMG1, MG_QUATERNION, q, sizeof (component_position));

		mg_send_set (sock, MYENT_DRAW_IMG1, MG_ADD_INSTANCEOF, &mesh, sizeof (uint32_t));
		mg_send_set (sock, MYENT_DRAW_IMG1, MG_ADD_INSTANCEOF, &texture, sizeof (uint32_t));
		//mg_send_set (sock, MYENT_DRAW_IMG2, MG_ADD_INSTANCEOF, &mesh, sizeof (uint32_t));
		//mg_send_set (sock, MYENT_DRAW_IMG2, MG_ADD_INSTANCEOF, &texture, sizeof (uint32_t));
	}

	{
		uint32_t mesh = MYENT_MESH_RECTANGLE2;
		uint32_t texture = MYENT_TEXTURE2;
		component_position p = {-0.8f, 0.0f, 0.0f, 1.0f};
		component_position s = {1.0f, 1.0f, 0.0f, 1.0f};
		component_position q = {0.0f, 0.0f, 0.0f, 1.0f};
		qf32_xyza (q, 0.0f, 0.0f, 1.0f, M_PI/2.0f);
		mg_send_add (sock, MYENT_DRAW_IMG4, MG_TRANSFORM);
		mg_send_set (sock, MYENT_DRAW_IMG4, MG_POSITION, p, sizeof (component_position));
		mg_send_set (sock, MYENT_DRAW_IMG4, MG_SCALE, s, sizeof (component_position));
		mg_send_set (sock, MYENT_DRAW_IMG4, MG_QUATERNION, q, sizeof (component_position));
		mg_send_set (sock, MYENT_DRAW_IMG4, MG_ADD_INSTANCEOF, &mesh, sizeof (uint32_t));
		mg_send_set (sock, MYENT_DRAW_IMG4, MG_ADD_INSTANCEOF, &texture, sizeof (uint32_t));
	}

	{
		//The color of each point. This is only used for visualization.
		uint32_t pointcol[LIDAR_WH*2];
		vu32_set1 (LIDAR_WH*2, pointcol, 0xFFFFFFFF);
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_COL, pointcol, LIDAR_WH*sizeof(uint32_t)*2);
	}

	{
		//The color of each point. This is only used for visualization.
		v4f32 lines[6];
		uint32_t col[6];
		component_count c = 6;
		vu32_set1 (6, col, 0xFFFFFFFF);

		mg_send_set (sock, MYENT_DRAW_LINES, MG_COUNT, &c, sizeof(c));
		mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_COL, col, sizeof(col));
		mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_POS, lines, sizeof(lines));
	}

}








static void show (struct skitrack1 * s1, struct skitrack2 * s2, nng_socket sock, uint32_t flags)
{
	float pointpos[LIDAR_WH*POINT_STRIDE*2];
	uint32_t imgv[IMG_XN*IMG_YN] = {0};//Used for visual confirmation that the algorithm works

	//points_test_sinus_slope (s1->pc);

	memcpy (pointpos, s1->pc, LIDAR_WH*POINT_STRIDE*sizeof(float));
	skitrack1_process (s1);
	skitrack2_process (s2, s1->pc, s1->pc_count);
	memcpy (pointpos + LIDAR_WH*POINT_STRIDE, s1->pc, LIDAR_WH*POINT_STRIDE*sizeof(float));


	if (flags & VISUAL_MODE_VERBOOSE1)
	{
		printf ("[INFO] Eigen values: %f %f %f\n", s1->w[0], s1->w[1], s1->w[2]);
		printf ("[INFO] Eigen column vectors:\n");
		m3f32_print (s1->c, stdout);
	}

	//Visualize the skitrack and more information:
	switch (flags & VISUAL_MODE_IMG_MASK)
	{
	case VISUAL_MODE_IMG1:
		image_visual (imgv, s2->img1, IMG_XN, IMG_YN, s2->q1, s2->q2, s2->g, SKITRACK2_PEAKS_COUNT, s2->k);
		break;
	case VISUAL_MODE_IMG2:
		image_visual (imgv, s2->img2, IMG_XN, IMG_YN, s2->q1, s2->q2, s2->g, SKITRACK2_PEAKS_COUNT, s2->k);
		break;
	case VISUAL_MODE_IMG3:
		image_visual (imgv, s2->img3, IMG_XN, IMG_YN, s2->q1, s2->q2, s2->g, SKITRACK2_PEAKS_COUNT, s2->k);
		break;
	}

	draw_skitrack (s2, imgv);



	//pix_rgba[105*IMG_XN + 12] |= RGBA(0x00, 0x66, 0x00, 0x00);
	//pix_rgba[0*IMG_XN + 1] |= RGBA(0x00, 0xFF, 0x00, 0xFF);
	//pix_rgba[2*IMG_XN + 0] |= RGBA(0x00, 0xFF, 0xff, 0xFF);
	//pix_rgba[2*IMG_XN + 1] |= RGBA(0x00, 0xFF, 0xff, 0xFF);



	/*
	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_0].a, 0.0f, 0.0f, 0.0f);
	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_0].b, 1.0f, 0.0f, 0.0f);
	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_1].a, 0.0f, 0.0f, 0.0f);
	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_1].b, 0.0f, 1.0f, 0.0f);
	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_2].a, 0.0f, 0.0f, 0.0f);
	vf32_set3 (linepos1[VISUAL_LINE_ORIGIN_2].b, 0.0f, 0.0f, 1.0f);
	linecol1[VISUAL_LINE_ORIGIN_0].a = RGBA(0xFF, 0x00, 0x00, 0xFF);
	linecol1[VISUAL_LINE_ORIGIN_0].b = RGBA(0xFF, 0x00, 0x00, 0xFF);
	linecol1[VISUAL_LINE_ORIGIN_1].a = RGBA(0x00, 0xFF, 0x00, 0xFF);
	linecol1[VISUAL_LINE_ORIGIN_1].b = RGBA(0x00, 0xFF, 0x00, 0xFF);
	linecol1[VISUAL_LINE_ORIGIN_2].a = RGBA(0x00, 0x00, 0xFF, 0xFF);
	linecol1[VISUAL_LINE_ORIGIN_2].b = RGBA(0x00, 0x00, 0xFF, 0xFF);
	for (int i = VISUAL_LINE_SKITRACK; i <= VISUAL_LINE_SKITRACK_END; ++i)
	{
		linecol1[i].a = RGBA(0x77, 0xFF, 0x11, 0xFF);
		linecol1[i].b = RGBA(0x77, 0xFF, 0x11, 0xFF);
	}
	{
		pixel_to_point (linepos1[VISUAL_LINE_SKITRACK+0].a, IMG_XN, IMG_YN, 0.0f, -10.0f, s2->g[0] - 10.0f * s2->k);
		pixel_to_point (linepos1[VISUAL_LINE_SKITRACK+0].b, IMG_XN, IMG_YN, 0.0f,  30.0f, s2->g[0] + 30.0f * s2->k);
		pixel_to_point (linepos1[VISUAL_LINE_SKITRACK+1].a, IMG_XN, IMG_YN, 0.0f, -10.0f, s2->g[1] - 10.0f * s2->k);
		pixel_to_point (linepos1[VISUAL_LINE_SKITRACK+1].b, IMG_XN, IMG_YN, 0.0f,  30.0f, s2->g[1] + 30.0f * s2->k);
		float rot[3*3];
		memcpy (rot, s1->r, sizeof (rot));
		m3f32_lapacke_inverse (rot, 3);
		for (float * i = linepos1[VISUAL_LINE_SKITRACK+0].a; i <= linepos1[VISUAL_LINE_SKITRACK_END].b; i += POINT_STRIDE)
		{
			mv3f32_mul (i, rot, i);
			vvf32_add (4, i, i, s1->centroid);
		}
	}

	*/


	{
		struct v4f32_line pos[6];
		struct u32_line col[6];
		vf32_set3 (pos[0].a, 0.0f   , 0.0f   , 0.0f   );
		vf32_set3 (pos[0].b, s1->c[0], s1->c[1], s1->c[2]);
		vf32_set3 (pos[1].a, 0.0f   , 0.0f   , 0.0f   );
		vf32_set3 (pos[1].b, s1->c[3], s1->c[4], s1->c[5]);
		vf32_set3 (pos[2].a, 0.0f   , 0.0f   , 0.0f   );
		vf32_set3 (pos[2].b, s1->c[6], s1->c[7], s1->c[8]);
		vsf32_mul (4, pos[0].b, pos[0].b, sqrt(s1->w[0])*2.0f);
		vsf32_mul (4, pos[1].b, pos[1].b, sqrt(s1->w[1])*2.0f);
		vsf32_mul (4, pos[2].b, pos[2].b, sqrt(s1->w[2])*2.0f);
		vf32_addv (4, pos, 4, pos, 4, s1->centroid, 0, 12);
		col[0].a = RGBA(0xFF, 0x00, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
		col[0].b = RGBA(0xFF, 0x00, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
		col[1].a = RGBA(0x00, 0xFF, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
		col[1].b = RGBA(0x00, 0xFF, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
		col[2].a = RGBA(0x00, 0x00, 0xFF, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
		col[2].b = RGBA(0x00, 0x00, 0xFF, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
		mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_POS, pos, sizeof(pos));
		mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_COL, col, sizeof(col));
	}



	show_img2 (sock, s1, flags);
	show_img4 (sock, s2);

	//Send visual information to the graphic server:
	{
		for (int i = 0; i < LIDAR_WH*2; ++i)
		{
			//pointpos[i + 0] = 10.0f;
			//pointpos[i + 1] = 10.0f;
			//pointpos[i + 2] = 10.0f;
			pointpos[i*POINT_STRIDE + 3] = 10.0f;
		}
		//points_test_sinus_slope(pointpos);
		mg_send_set (sock, MYENT_TEXTURE1, MG_TEXTURE_CONTENT, imgv, IMG_XN*IMG_YN*sizeof(uint32_t));
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_POS, pointpos, LIDAR_WH*POINT_STRIDE*sizeof(float)*2);
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
