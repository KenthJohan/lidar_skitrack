#include <iostream>
#include <ce30_driver/ce30_driver.h>
#include <unistd.h>
#include "../csc/csc_debug.h"

using namespace std;
using namespace ce30_driver;


int main()
{

	{
		clock_t t0 = clock();
		sleep(1);
		clock_t t1 = clock();
		double d = (t1-t0) / CLOCKS_PER_SEC;
		printf ("d: %lf\n", d);
		ASSERTF ((d < 1.0001) && (d > 0.9999), "clock() and CLOCKS_PER_SEC not right");
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
	clock_t time_delta = 0;
	clock_t time0 = 0;

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

			iterations++;
			time_delta = clock() - time0;
			time0 = clock();
			double d = (double)(time_delta) / CLOCKS_PER_SEC;
			time_sum += d;
			printf ("delta: %lf10.7\n", d);
			printf ("d avg: %lf10.7\n", time_sum / iterations);

			scan.Reset();
		}
	}
	StopRequestPacket stop_request;
	SendPacket(stop_request, socket);
}
