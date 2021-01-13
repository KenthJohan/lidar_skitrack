#include <iostream>
#include <ce30_driver/ce30_driver.h>
#include <chrono>
#include <unistd.h>
#include <time.h>
#include "../csc/csc_debug.h"
#include "../csc/csc_crossos.h"

using namespace std;
using namespace ce30_driver;




uint32_t iterations = 0;
double time_sum = 0;
struct timespec ts0;
struct timespec ts1;


void DataReceiveCB(shared_ptr<PointCloud> cloud)
{
	for (Point& point : cloud->points)
	{
		//printf ("%f %f %f\n", point.x, point.y, point.z);
	}
	{
		clock_gettime (CLOCK_REALTIME, &ts1);
		double d = ((ts1.tv_nsec - ts0.tv_nsec) / 1000000000.0) + (ts1.tv_sec  - ts0.tv_sec);
		iterations++;
		time_sum += d;
		printf ("delta: %lf10.7\n", d);
		printf ("d avg: %lf10.7\n", time_sum / iterations);
		clock_gettime (CLOCK_REALTIME, &ts0);
	}
}


int main()
{
	UDPServer server;
	server.RegisterCallback(DataReceiveCB);
	if (!server.Start())
	{
		return -1;
	}
	clock_gettime (CLOCK_REALTIME, &ts0);
	while (true)
	{
		server.SpinOnce();
	}
}
