#pragma once

#include <stdio.h>
#include "csc/csc_math.h"
#include "csc/csc_tcol.h"
#include "csc/csc_m3f32_print.h"
#include "csc/csc_v3f32_print.h"

static FILE * main_log_file;

static void log_pca (m3f32 e, v3f32 w)
{
	//printf ("[INFO] Centroid: %f %f %f\n", s2->centroid[0], s2->centroid[1], s2->centroid[2]);
	printf ("[INFO] Eigen values:\n");
	csc_v3f32_print_rgb (main_log_file, w);
	printf ("[INFO] Eigen column vectors:\n");
	csc_m3f32_print_rgb (main_log_file, e);
}
