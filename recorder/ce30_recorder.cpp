#include <iostream>
#include <unistd.h>
#include <time.h>


#include <ce30_driver/ce30_driver.h>
#include "ce30_driver/ce30.h"
#include "ce30_driver/custom.hpp"

#include "csc/csc_debug.h"
#include "csc/csc_argv.h"

using namespace std;
using namespace ce30_driver;

#define ARG_HELP    UINT32_C(0x00000001)
#define ARG_VERBOSE UINT32_C(0x00000002)
#define ARG_STDOUT  UINT32_C(0x00000004)

int main (int argc, char const * argv[])
{
	UNUSED (argc);
	char const * arg_filename = "ce30_pointcloud.out";
	char const * arg_logfile = "ce30_pointcloud.log";
	double arg_duration = 0;
	uint32_t arg_flags = 0;
	FILE * file_out = NULL;
	FILE * file_log = stdout;

	struct csc_argv_option option[] =
	{
	{'d', "duration", CSC_TYPE_DOUBLE,    &arg_duration,   0, "How long to record"},
	{'f', "filename", CSC_TYPE_STRING,    &arg_filename,   0, "The filename"},
	{'l', "logfile",  CSC_TYPE_STRING,    &arg_logfile,    0, "The log filename"},
	{'h', "help",     CSC_TYPE_U32,       &arg_flags,      ARG_HELP, "Show help"},
	{'v', "verbose",  CSC_TYPE_U32,       &arg_flags,      ARG_VERBOSE, "Show verbose"},
	{'s', "stdout",   CSC_TYPE_U32,        &arg_flags,      ARG_STDOUT, "Outputs pointdata to stdout"},
	{CSC_ARGV_END}};

	csc_argv_parseall (argv+1, option);

	if (arg_flags & ARG_HELP)
	{
		csc_argv_description0 (option, stdout);
		csc_argv_description1 (option, stdout);
		return 0;
	}

	if (arg_flags & ARG_STDOUT)
	{
		arg_filename = NULL;
		file_log = fopen (arg_logfile, "w");
		file_out = stdout;
	}

	if (arg_filename)
	{
		fprintf (file_log, "Opening binary file %s to write LiDAR frames.\n", arg_filename);
		file_out = fopen (arg_filename, "wb");
	}


	ASSERT (file_out);


	UDPSocket socket;
	if (!Connect(socket))
	{
		fprintf (file_log, "Could not connect to socket\n");
		return -1;
	}
	VersionRequestPacket version_request;
	if (!SendPacket(version_request, socket))
	{
		fprintf (file_log, "Could not send VersionRequestPacket\n");
		return -1;
	}
	VersionResponsePacket version_response;
	if (!GetPacket(version_response, socket))
	{
		fprintf (file_log, "Could not get VersionResponsePacket\n");
		return -1;
	}

	fprintf (file_log, "CE30-D Version: %s\n", version_response.GetVersionString().c_str());

	StartRequestPacket start_request;
	if (!SendPacket(start_request, socket))
	{
		fprintf (file_log, "Could not send StartRequestPacket\n");
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
		fprintf (file_log, "%10s %10s %10s %10s %10s %10s\n", "frame", "duration", "spf", "fps", "avg_fps", "frame_amp");
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

		float frame[CE30_WIDTH*CE30_HEIGHT*4] = {0};
		ce30_scan_to_frame (scan, frame);
		/*
		for (int i = 0; i < CE30_WIDTH*CE30_HEIGHT*4; ++i)
		{
			float r = (float)rand() / (float)RAND_MAX;
			frame[i] += r * 100.0f;
		}
		*/
		int r = fwrite (frame, sizeof (frame), 1, file_out);
		ASSERT (r == 1);

		struct timespec ts2;
		clock_gettime (CLOCK_REALTIME, &ts2);
		d21 = ((ts2.tv_nsec - ts1.tv_nsec) / 1000000000.0) + (ts2.tv_sec  - ts1.tv_sec);
		ts1 = ts2;
		d10 = ((ts1.tv_nsec - ts0.tv_nsec) / 1000000000.0) + (ts1.tv_sec  - ts0.tv_sec);
		counter++;


		if (arg_flags & ARG_VERBOSE)
		{
			float s = ce30_scan_frame_amplitude (scan);
			fprintf (file_log, "%10u %10.2lf %10.8lf %10.6lf %10.6lf %10.5f\n", counter, d10, d21, 1.0 / d21, (double)counter / d10, s);
		}
		if (arg_duration > 0.0 && d10 > arg_duration)
		{
			if (arg_flags & ARG_VERBOSE)
			{
				fprintf (file_log, "%10s %10s %10s %10s %10s %10s\n", "frame", "duration", "spf", "fps", "avg_fps", "frame_amp");
			}
			fprintf (file_log, "The recording ended after %lfs. Recording duration was set to %lf\n", d10, arg_duration);
			break;
		}



		scan.Reset();
	}
	StopRequestPacket stop_request;
	SendPacket (stop_request, socket);
}
