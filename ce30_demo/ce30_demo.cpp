#include <iostream>
#include <ce30_driver/ce30_driver.h>
#include <chrono>

using namespace std;
using namespace ce30_driver;


uint32_t iterations = 0;
double time_sum = 0;
clock_t time_delta = 0;
clock_t time0 = 0;


void DataReceiveCB(shared_ptr<PointCloud> cloud)
{
	iterations++;
	time_delta = clock() - time0;
	time0 = clock();
	for (Point& point : cloud->points)
	{
		//cout << point.x << " " << point.y << " " << point.z << endl;
	}
	double d = (double)(time_delta) / CLOCKS_PER_SEC;
	time_sum += d;
	printf ("delta: %lf10.7\n", d);
	printf ("d avg: %lf10.7\n", time_sum / iterations);
	printf ("count: %i\n", cloud->points.size());
}

int main()
{
	UDPServer server;
	server.RegisterCallback(DataReceiveCB);
	if (!server.Start())
	{
		return -1;
	}
	while (true)
	{
		server.SpinOnce();
	}
}
