#pragma once

#include <stdio.h>

#include "csc/csc_debug_nng.h"
#include "csc/csc_math.h"
#include "csc/csc_linmat.h"
#include "csc/csc_m3f32.h"
#include "csc/csc_m3f32_print.h"
#include "csc/csc_v3f32_print.h"
#include "csc/csc_vu32.h"
#include "csc/csc_rgb.h"

#include "../shared/shared.h"
#include "points_read.h"
#include "mathmisc.h"

#include "mg_attr.h"
#include "mg_comp.h"
#include "mg_send.h"
#include "myent.h"

#include "skitrack.h"

static void show_init (nng_socket sock)
{
	mg_send_add (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD);
	mg_send_add (sock, MYENT_DRAW_LINES, MG_LINES);



	{
		component_count c = LIDAR_WH*1;
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_COUNT, &c, sizeof(component_count));
	}


	{
		//The color of each point. This is only used for visualization.
		uint32_t pointcol[LIDAR_WH*1];
		vu32_set1 (LIDAR_WH*1, pointcol+LIDAR_WH*0, 0xFFFFFF00);
		//vu32_set1 (LIDAR_WH*1, pointcol+LIDAR_WH*1, 0xFFFFFF88);
		//vu32_set1 (LIDAR_WH*1, pointcol+LIDAR_WH*2, 0xFF88FFFF);
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_COL, pointcol, LIDAR_WH*sizeof(uint32_t)*1);
	}

	{
		//The color of each point. This is only used for visualization.
		struct v4f32 lines[6];
		uint32_t col[6];
		component_count c = 6;
		vu32_set1 (6, col, 0xFFFFFFFF);
		mg_send_set (sock, MYENT_DRAW_LINES, MG_COUNT, &c, sizeof(c));
		mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_COL, col, sizeof(col));
		mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_POS, lines, sizeof(lines));
	}

}



static void skitrack_show (struct skitrack * ski, nng_socket sock)
{
	/*
	struct rgba pointcol[LIDAR_WH*1];
	for (uint32_t i = 0; i < LIDAR_WH; ++i)
	{
		float w = ski->x[i].w;
		ski->x[i].w = 20.0f;
		//pointcol[i] = 0xFF000000;
		w = CLAMP(w * 4.0f, 0.0f, 255.0f);
		pointcol[i].r = (uint8_t)(w);
		pointcol[i].g = (uint8_t)(w);
		pointcol[i].b = (uint8_t)(w);
		pointcol[i].a = 0xFF;
	}
	*/
	mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_POS, ski->x, sizeof(struct v4f32)*LIDAR_WH);
	//mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_COL, pointcol, LIDAR_WH*sizeof(uint32_t));
}












