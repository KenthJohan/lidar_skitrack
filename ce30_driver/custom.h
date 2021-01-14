#pragma once

#include <iostream>
#include <ce30_driver/ce30_driver.h>
#include <unistd.h>
#include <time.h>

#include <ce30_driver/custom.h>

#include "../csc/csc_debug.h"
#include "../csc/csc_argv.h"

#define CE30_WIDTH 320
#define CE30_HIEGHT 20
#define CE30_XY_INDEX(x,y) ((x)*CE30_HIEGHT + (y))

enum ce30_outformat
{
	CE30_OUTFORMAT_TEXT_XYZA,
	CE30_OUTFORMAT_BINARY_XYZA
};






void ce30_scan_to_frame (ce30_driver::Scan scan, float frame[CE30_WIDTH*CE30_HIEGHT*4])
{
	for (int x = 0; x < CE30_WIDTH; ++x)
	{
		for (int y = 0; y < CE30_HIEGHT; ++y)
		{
			ce30_driver::Channel channel = scan.at(x, y);
			frame[CE30_XY_INDEX(x,y)*4 + 0] = channel.point().x;
			frame[CE30_XY_INDEX(x,y)*4 + 1] = channel.point().y;
			frame[CE30_XY_INDEX(x,y)*4 + 2] = channel.point().z;
			frame[CE30_XY_INDEX(x,y)*4 + 3] = channel.amplitude;
		}
	}
}


double ce30_scan_frame_amplitude (ce30_driver::Scan scan)
{
	float sum = 0;
	for (int x = 0; x < CE30_WIDTH; ++x)
	{
		for (int y = 0; y < CE30_HIEGHT; ++y)
		{
			ce30_driver::Channel channel = scan.at(x, y);
			sum += channel.amplitude;
		}
	}
	return sum / (CE30_WIDTH*CE30_HIEGHT);
}
