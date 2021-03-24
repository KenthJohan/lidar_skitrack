#pragma once



/*
#define POINTC_W 320
#define POINTC_H 20

#define VOX_XN 60
#define VOX_YN 30
#define VOX_ZN 10
#define VOX_I(x,y,z) ((z)*VOX_XN*VOX_YN + (y)*VOX_XN + (x))
#define VOX_SCALE 0.15f
*/


#define RGBA(r,g,b,a) (((r) << 0) | ((g) << 8) | ((b) << 16) | ((a) << 24))

#define POINT_STRIDE 4
#define POINT_DIM 3

#define LIDAR_W 320
#define LIDAR_H 20
#define LIDAR_WH (LIDAR_W*LIDAR_H)
#define LIDAR_FPS 30
#define LIDAR_FOV_W 60
#define LIDAR_FOV_H 4
#define LIDAR_INDEX(x,y) ((x)*LIDAR_H + (y))

#define VOXEL_XN 60
#define VOXEL_YN 30
#define VOXEL_ZN 10
#define VOXEL_INDEX(x,y,z) ((z)*VOXEL_XN*VOXEL_YN + (y)*VOXEL_XN + (x))
#define VOXEL_SCALE 0.15f
#define PIXEL_INDEX(x,y) ((y)*VOXEL_XN + (x))

#define IMG_XN 30
#define IMG_YN 140
#define IMG_CN 4
#define IMG_SCALE (1.0f/25.0f)
