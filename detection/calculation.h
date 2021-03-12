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

#include "skitrack.h"




#define VISUAL_MODE_IMG_MASK          UINT32_C(0x0000000F)
#define VISUAL_MODE_IMG1              UINT32_C(0x00000001)
#define VISUAL_MODE_IMG2              UINT32_C(0x00000002)
#define VISUAL_MODE_IMG3              UINT32_C(0x00000003)
#define VISUAL_MODE_VERBOOSE_MASK     UINT32_C(0x000000F0)
#define VISUAL_MODE_VERBOOSE1         UINT32_C(0x00000010)

#define IMG3_XN 4



static void convert_float_to_rgba (uint32_t img[], float pix[], uint32_t n)
{
	//Negatives becomes red and positives becomes greeen:
	for (uint32_t i = 0; i < n; ++i)
	{
		img[i] = rgba_value (pix[i], -3000.0f, 3000.0f, 0.0f);
	}
}


static void draw_skitrack (nng_socket sock, struct skitrack * s2, uint32_t img[], uint32_t flags)
{
	//Visualize the skitrack and more information:
	switch (flags & VISUAL_MODE_IMG_MASK)
	{
	case VISUAL_MODE_IMG1:
		convert_float_to_rgba (img, s2->img1, IMG_XN*IMG_YN);
		break;
	case VISUAL_MODE_IMG2:
		convert_float_to_rgba (img, s2->img2, IMG_XN*IMG_YN);
		break;
	case VISUAL_MODE_IMG3:
		convert_float_to_rgba (img, s2->img3, IMG_XN*IMG_YN);
		break;
	}
	for (uint32_t i = 0; i < SKITRACK2_PEAKS_COUNT; ++i)
	{
		if (s2->peak[i] < IMG_YN)
		{
			uint32_t y = s2->peak[i];
			for (uint32_t x = 0; x < IMG_XN; ++x)
			{
				float yy = (float)y + (float)x * s2->k;
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
	mg_send_set (sock, MYENT_TEXTURE1, MG_TEXTURE_CONTENT, img, IMG_XN*IMG_YN*sizeof(uint32_t));
}


static void draw_img2 (nng_socket sock, struct skitrack * s1, uint32_t flags)
{
	float m[4*4] = {0.0f};
	//m4f32_identity (m);
	m4f32_translation (m, s1->centroid);
	m[M4_02] = s1->e[0];//Column 2
	m[M4_12] = s1->e[1];//Column 2
	m[M4_22] = s1->e[2];//Column 2
	m[M4_00] = s1->e[3];//Column 0
	m[M4_10] = s1->e[4];//Column 0
	m[M4_20] = s1->e[5];//Column 0
	m[M4_01] = s1->e[6];//Column 1
	m[M4_11] = s1->e[7];//Column 1
	m[M4_21] = s1->e[8];//Column 1
	m[M4_33] = 1.0f;
	if (flags & VISUAL_MODE_VERBOOSE1)
	{
		//printf ("[INFO] Affine matrix of img2:\n");
		//m4f32_print (m, stdout);
	}
	//mg_send_set (sock, MYENT_DRAW_IMG2, MG_TRANSFORM, m, sizeof (component_transform));
}


static void draw_img4 (nng_socket sock, struct skitrack * s2)
{
	uint32_t imgv[IMG3_XN*IMG_YN] = {0};//Used for visual confirmation that the algorithm works

	for (uint32_t y = 0; y < IMG_YN; ++y)
	{
		imgv[y+IMG_YN*0] = rgba_value (s2->q1[y], -100.0f, 100.0f, 0.0f);
		imgv[y+IMG_YN*1] = rgba_value (s2->q2[y], -100.0f, 100.0f, 0.0f);
		imgv[y+IMG_YN*2] = rgba_value (s2->q3[y], -100.0f, 100.0f, 0.0f);
		imgv[y+IMG_YN*3] = rgba_value (s2->qmem[y], -100.0f, 100.0f, 0.0f);
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


static void draw_tracklines()
{
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
}


static void draw_pca (nng_socket sock, struct skitrack * s1)
{
	struct v4f32_line pos[3];
	struct u32_line col[3];
	vf32_set3 (pos[0].a, 0.0f   , 0.0f   , 0.0f   );
	vf32_set3 (pos[0].b, s1->e[0], s1->e[1], s1->e[2]);
	vf32_set3 (pos[1].a, 0.0f   , 0.0f   , 0.0f   );
	vf32_set3 (pos[1].b, s1->e[3], s1->e[4], s1->e[5]);
	vf32_set3 (pos[2].a, 0.0f   , 0.0f   , 0.0f   );
	vf32_set3 (pos[2].b, s1->e[6], s1->e[7], s1->e[8]);
	vsf32_mul (4, pos[0].b, pos[0].b, sqrt(s1->w[0])*2.0f);
	vsf32_mul (4, pos[1].b, pos[1].b, sqrt(s1->w[1])*2.0f);
	vsf32_mul (4, pos[2].b, pos[2].b, sqrt(s1->w[2])*2.0f);
	vf32_addv (4, (float*)pos, 4, (float*)pos, 4, s1->centroid, 0, 3*2);
	col[0].a = RGBA(0xFF, 0x00, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
	col[0].b = RGBA(0xFF, 0x00, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
	col[1].a = RGBA(0x00, 0xFF, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
	col[1].b = RGBA(0x00, 0xFF, 0x00, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
	col[2].a = RGBA(0x00, 0x00, 0xFF, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
	col[2].b = RGBA(0x00, 0x00, 0xFF, 0xFF) | RGBA(0xAA, 0xAA, 0xAA, 0xFF);
	mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_POS, pos, sizeof(pos));
	mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_COL, col, sizeof(col));
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
		component_count c = LIDAR_WH*3;
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
		uint32_t pointcol[LIDAR_WH*3];
		vu32_set1 (LIDAR_WH*1, pointcol+LIDAR_WH*0, 0xFFFFFFFF);
		vu32_set1 (LIDAR_WH*1, pointcol+LIDAR_WH*1, 0xFFFFFF88);
		vu32_set1 (LIDAR_WH*1, pointcol+LIDAR_WH*2, 0xFF88FFFF);
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_COL, pointcol, LIDAR_WH*sizeof(uint32_t)*3);
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


static void show (struct skitrack * s2, nng_socket sock, uint32_t flags)
{
	//Used for visual confirmation of the algorithm internal workings:
	struct
	{
		float cloud1[LIDAR_WH*POINT_STRIDE];
		float cloud2[LIDAR_WH*POINT_STRIDE];
		float cloud3[LIDAR_WH*POINT_STRIDE];
	} points;
	memset (&points, 0, sizeof(points));
	uint32_t imgv[IMG_XN*IMG_YN] = {0};


	//points_test_sinus_slope (s1->pc);

	//Copy unrectified pointcloud:
	memcpy (points.cloud1, s2->pc1, LIDAR_WH*POINT_STRIDE*sizeof(float));
	//skitrack_firstpass (s2);
	skitrack_rectify (s2);
	skitrack_process (s2);
	//skitrack_subset (s2);

	//Copy rectified pointcloud:
	memcpy (points.cloud2, s2->pc1, LIDAR_WH*POINT_STRIDE*sizeof(float));
	memcpy (points.cloud3, s2->pc2, LIDAR_WH*POINT_STRIDE*sizeof(float));

	for (int i = 0; i < LIDAR_WH; ++i)
	{
		points.cloud1[i*POINT_STRIDE + 3] = 10.0f; //Set the size of the points
	}
	for (int i = 0; i < LIDAR_WH; ++i)
	{
		points.cloud2[i*POINT_STRIDE + 3] = 10.0f; //Set the size of the points
	}
	for (uint32_t i = 0; i < LIDAR_WH; ++i)
	{
		points.cloud3[i*POINT_STRIDE + 2] += 0.5f; //Move pointcloud up a little
		points.cloud3[i*POINT_STRIDE + 3] = 20.0f; //Set the size of the points
	}



	/*
	pointcloud_pca1 (s1->pc, s1->pc1, &s1->pc_count, POINT_STRIDE, s1->centroid, s1->w, s1->c, s1->r);
	skitrack2_process (s2, s1->pc, s1->pc_count);

	//2n iteration
	printf("2n iteration pc_count :%i\n", s1->pc_count);
	memset (pointpos + PC2, 0, LIDAR_WH*POINT_STRIDE*sizeof(float));
	skitrack1_process (s1);
	skitrack2_process (s2, s1->pc, s1->pc_count);
	memcpy (pointpos + PC2, s1->pc, LIDAR_WH*POINT_STRIDE*sizeof(float));
	*/




	//points_test_sinus_slope(pointpos);
	mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_POS, &points, sizeof(points));


	if (flags & VISUAL_MODE_VERBOOSE1)
	{
		printf ("[INFO] Centroid: %f %f %f\n", s2->centroid[0], s2->centroid[1], s2->centroid[2]);
		printf ("[INFO] Eigen values: %f %f %f\n", s2->w[0], s2->w[1], s2->w[2]);
		printf ("[INFO] Eigen column vectors:\n");
		m3f32_print (s2->e, stdout);
	}

	draw_skitrack (sock, s2, imgv, flags);
	draw_pca (sock, s2);
	draw_img2 (sock, s2, flags);
	draw_img4 (sock, s2);
}
