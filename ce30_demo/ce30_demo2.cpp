#include <iostream>
#include <ce30_driver/ce30_driver.h>
#include <unistd.h>
#include <time.h>
#include "../csc/csc_debug.h"

using namespace std;
using namespace ce30_driver;


int main()
{

	{
		struct timespec begin, end;
		clock_gettime (CLOCK_MONOTONIC_RAW, &begin);
		sleep(1);
		clock_gettime (CLOCK_MONOTONIC_RAW, &end);
		double d = ((end.tv_nsec - begin.tv_nsec) / 1000000000.0) + (end.tv_sec  - begin.tv_sec);
		ASSERTF ((d < 1.1) && (d > 0.9), "clock() and CLOCKS_PER_SEC not right. d = %lf, CLOCKS_PER_SEC=%i", d, CLOCKS_PER_SEC);
	}


	UDPSocket socket;
	if (!Connect(socket))
	{
		return -1;
	}
	VersionRequestPacket version_request;
	if (!SendPacket(version_request, socket))
	{
		return -1;
	}
	VersionResponsePacket version_response;
	if (!GetPacket(version_response, socket))
	{
		return -1;
	}
	cout << "CE30-D Version: " << version_response.GetVersionString() << endl;
	StartRequestPacket start_request;
	if (!SendPacket(start_request, socket))
	{
		return -1;
	}

	// Now it's ready to receive measurement data
	Packet packet;
	Scan scan;

	uint32_t iterations = 0;
	double time_sum = 0;
	struct timespec ts0;
	struct timespec ts1;

	clock_gettime (CLOCK_MONOTONIC_RAW, &ts0);
	while (true)
	{
		if (!GetPacket(packet, socket))
		{
			continue;
		}
		unique_ptr<ParsedPacket> parsed = packet.Parse();
		if (parsed)
		{
			scan.AddColumnsFromPacket(*parsed);
			if (!scan.Ready())
			{
				continue;
			}
			for (int x = 0; x < scan.Width(); ++x)
			{
				for (int y = 0; y < scan.Height(); ++y)
				{
					//Channel channel = scan.at(x, y);
					//printf ("%f %f %f\n", channel.point().x, channel.point().y, channel.point().z);
				}
			}
			scan.Reset();
			
			{
				clock_gettime (CLOCK_MONOTONIC_RAW, &ts1);
				double d = ((ts1.tv_nsec - ts0.tv_nsec) / 1000000000.0) + (ts1.tv_sec  - ts0.tv_sec);
				iterations++;
				time_sum += d;
				printf ("delta: %lf10.7\n", d);
				printf ("d avg: %lf10.7\n", time_sum / iterations);
				clock_gettime (CLOCK_MONOTONIC_RAW, &ts0);
			}
			
		}
	}
	StopRequestPacket stop_request;
	SendPacket(stop_request, socket);
}
