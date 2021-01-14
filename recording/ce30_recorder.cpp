#include <iostream>
#include <ce30_driver/ce30_driver.h>
#include <unistd.h>
#include <time.h>

#include <ce30_driver/custom.h>

#include "../csc/csc_debug.h"
#include "../csc/csc_argv.h"

using namespace std;
using namespace ce30_driver;

#define ARG_HELP    UINT32_C(0x00000001)
#define ARG_VERBOSE UINT32_C(0x00000002)

int main (int argc, char const * argv[])
{
	char const * arg_filename = "ce30_pointcloud.out";
	double arg_duration = 0;
	uint32_t arg_flags = 0;
	FILE * file_out = NULL;
	struct csc_argv_option option[] =
	{
	{'d', "duration", CSC_ARGV_TYPE_DOUBLE,    &arg_duration,   {.val_umax = 0}, "How long to record"},
	{'f', "filename", CSC_ARGV_TYPE_STRING,    &arg_filename,   {.val_umax = 0}, "The filename"},
	{'h', "help",     CSC_ARGV_TYPE_U32,       &arg_flags,      {.val_u32 = ARG_HELP}, "Show help"},
	{'v', "verbose",  CSC_ARGV_TYPE_U32,       &arg_flags,      {.val_u32 = ARG_VERBOSE}, "Show verbose"},
	{.type = CSC_ARGV_TYPE_END}};

	csc_argv_parsev (option, argv+1);

	if (arg_flags & ARG_HELP)
	{
		csc_argv_print_description (option);
		csc_argv_print_value (option);
		return 0;
	}


	if (arg_filename)
	{
		printf ("Opening binary file %s to write LiDAR frames.\n", arg_filename);
		file_out = fopen (arg_filename, "wb");
	}

	ASSERT (file_out);


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

	//Timestamps to measure duration of recording
	struct timespec ts0;
	struct timespec ts1;
	double d10;
	double d21;
	uint32_t counter = 0;
	Packet packet;
	Scan scan;

	if (arg_flags & ARG_VERBOSE)
	{
		printf ("%10s %10s %10s %10s %10s\n", "frame", "duration", "spf", "fps", "avg_fps");
	}
	clock_gettime (CLOCK_REALTIME, &ts0);
	clock_gettime (CLOCK_REALTIME, &ts1);
	while (true)
	{
		if (!GetPacket(packet, socket))
		{
			continue;
		}
		std::unique_ptr<ParsedPacket> parsed = packet.Parse();
		if (parsed == NULL)
		{
			continue;
		}
		scan.AddColumnsFromPacket(*parsed);
		if (!scan.Ready())
		{
			continue;
		}

		//fprintf (f, "%+Ef %+Ef %+Ef %+Ef %02x\n", channel.point().x, channel.point().y, channel.point().z, channel.amplitude, channel.amp_raw);

		float frame[CE30_WIDTH*CE30_HIEGHT*4] = {0};
		ce30_scan_to_frame (scan, frame);
		int r = fwrite (frame, CE30_WIDTH*CE30_HIEGHT*4, 1, file_out);
		ASSERT (r == 1);
		scan.Reset();



		struct timespec ts2;
		clock_gettime (CLOCK_REALTIME, &ts2);
		d21 = ((ts2.tv_nsec - ts1.tv_nsec) / 1000000000.0) + (ts2.tv_sec  - ts1.tv_sec);
		ts1 = ts2;
		d10 = ((ts1.tv_nsec - ts0.tv_nsec) / 1000000000.0) + (ts1.tv_sec  - ts0.tv_sec);
		counter++;


		if (arg_flags & ARG_VERBOSE)
		{
			printf ("%10u %10.2lf %10.8lf %10.6lf %10.6lf\n", counter, d10, d21, 1.0 / d21, (double)counter / d10);
		}
		if (arg_duration > 0.0 && d10 > arg_duration)
		{
			if (arg_flags & ARG_VERBOSE)
			{
				printf ("%10s %10s %10s %10s %10s\n", "frame", "duration", "spf", "fps", "avg_fps");
			}
			printf ("The recording has ended because it had limited recording duration\n");
			break;
		}
	}
	StopRequestPacket stop_request;
	SendPacket (stop_request, socket);
}
