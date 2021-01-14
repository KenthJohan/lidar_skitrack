#include <iostream>
#include <ce30_driver/ce30_driver.h>
#include <unistd.h>
#include <time.h>
#include "../csc/csc_debug.h"
#include "../csc/csc_argv.h"

using namespace std;
using namespace ce30_driver;

#define ARG_HELP 0x00000001

int main (int argc, char const * argv[])
{
	char const * arg_filename = "ce30_pointcloud.out";
	double arg_duration = 0;
	int arg_flags = 0;
	struct csc_argv_option option[] =
	{
	{'d', "duration", CSC_ARGV_TYPE_DOUBLE,    &arg_duration,   {.val_umax = 0}, "How long to record"},
	{'f', "filename", CSC_ARGV_TYPE_STRING,    &arg_filename,   {.val_umax = 0}, "The filename"},
	{'h', "help",     CSC_ARGV_TYPE_INT,       &arg_flags,      {.val_int = ARG_HELP}, "Show help"},
	{.type = CSC_ARGV_TYPE_END}};

	csc_argv_parsev (option, argv+1);

	if (arg_flags & ARG_HELP)
	{
		csc_argv_print_description (option);
		csc_argv_print_value (option);
		return 0;
	}

	UDPSocket socket;
	if (!Connect(socket))
	{
		fprintf (stderr, "Could not connect to socket\n");
		return -1;
	}
	VersionRequestPacket version_request;
	if (!SendPacket(version_request, socket))
	{
		fprintf (stderr, "Could not send VersionRequestPacket\n");
		return -1;
	}
	VersionResponsePacket version_response;
	if (!GetPacket(version_response, socket))
	{
		fprintf (stderr, "Could not get VersionResponsePacket\n");
		return -1;
	}

	printf ("CE30-D Version: %s\n", version_response.GetVersionString().c_str());

	StartRequestPacket start_request;
	if (!SendPacket(start_request, socket))
	{
		fprintf (stderr, "Could not send StartRequestPacket\n");
		return -1;
	}

	// Now it's ready to receive measurement data
	Packet packet;
	Scan scan;


	//Timestamps to measure duration of recording
	struct timespec ts0;
	struct timespec ts1;

	//ASSERT_NOTNULL (arg_filename);
	//FILE * f = fopen (arg_filename, "r");
	//ASSERT_NOTNULL (f);

	clock_gettime (CLOCK_REALTIME, &ts0);
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
					Channel channel = scan.at(x, y);
					//printf ("%f %f %f %02x\n", channel.point().x, channel.point().y, channel.point().z, channel.amp_raw);
					printf ("%+4.2f %+4.2f %+4.2f %+4.2f %02x\n", channel.point().x, channel.point().y, channel.point().z, channel.amplitude, channel.amp_raw);
					//fwrite ();
				}
			}
			scan.Reset();


			if (arg_duration > 0.0)
			{
				clock_gettime (CLOCK_REALTIME, &ts1);
				double d = ((ts1.tv_nsec - ts0.tv_nsec) / 1000000000.0) + (ts1.tv_sec  - ts0.tv_sec);
				if (d > arg_duration)
				{
					printf ("The recording has ended because it had limited recording duration\n");
					break;
				}
			}
		}
	}
	StopRequestPacket stop_request;
	SendPacket (stop_request, socket);
}
