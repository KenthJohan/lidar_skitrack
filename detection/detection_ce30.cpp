#include <iostream>
#include <unistd.h>
#include <time.h>

#include <ce30_driver/ce30_driver.h>
#include "ce30_driver/custom.hpp"
#include "ce30_driver/ce30.h"

#include "csc/csc_debug.h"
#include "csc/csc_argv.h"

#include "mg_send.h"
#include "calculation.h"

using namespace std;
using namespace ce30_driver;

#define ARG_HELP    UINT32_C(0x00000001)
#define ARG_VERBOSE UINT32_C(0x00000002)




int main (int argc, char const * argv[])
{
	char const * arg_address = "ce30_pointcloud.out";
	double arg_duration = 0;
	uint32_t arg_flags = 0;
	struct csc_argv_option option[] =
	{
	{'d', "duration", CSC_ARGV_TYPE_DOUBLE,    &arg_duration,   {.val_umax = 0}, "How long to send"},
	{'f', "filename", CSC_ARGV_TYPE_STRING,    &arg_address,    {.val_umax = 0}, "Address"},
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

	nng_socket sock;
	mg_pairdial (&sock, arg_address);
	show_init (sock);

	if (arg_flags & ARG_VERBOSE)
	{
		printf ("%10s %10s %10s %10s %10s %10s\n", "frame", "duration", "spf", "fps", "avg_fps", "frame_amp");
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

		struct skitrack1 s1 = {0};
		struct skitrack2 s2 = {0};
		show (&s1, &s2, sock, 1);



		struct timespec ts2;
		clock_gettime (CLOCK_REALTIME, &ts2);
		d21 = ((ts2.tv_nsec - ts1.tv_nsec) / 1000000000.0) + (ts2.tv_sec  - ts1.tv_sec);
		ts1 = ts2;
		d10 = ((ts1.tv_nsec - ts0.tv_nsec) / 1000000000.0) + (ts1.tv_sec  - ts0.tv_sec);
		counter++;


		if (arg_flags & ARG_VERBOSE)
		{
			float s = ce30_scan_frame_amplitude (scan);
			printf ("%10u %10.2lf %10.8lf %10.6lf %10.6lf %10.5f\n", counter, d10, d21, 1.0 / d21, (double)counter / d10, s);
		}
		if (arg_duration > 0.0 && d10 > arg_duration)
		{
			if (arg_flags & ARG_VERBOSE)
			{
				printf ("%10s %10s %10s %10s %10s %10s\n", "frame", "duration", "spf", "fps", "avg_fps", "frame_amp");
			}
			printf ("The recording ended after %ds. Recording duration was set to %d\n", d10, arg_duration);
			break;
		}



		scan.Reset();
	}
	StopRequestPacket stop_request;
	SendPacket (stop_request, socket);

	nng_close (sock);
}
