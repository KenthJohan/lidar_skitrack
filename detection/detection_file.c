#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>

//git clone https://github.com/nanomsg/nng
//cd nng && mkdir build && cd build
//cmake -G"MSYS Makefiles" .. -DCMAKE_INSTALL_PREFIX="C:\msys64\mingw64"
//pacman -R cmake
//pacman -S mingw-w64-x86_64-cmake
//mingw32-make -j4
//mingw32-make test
//mingw32-make install
//-lnng
#include <nng/nng.h>
#include <nng/protocol/pair0/pair.h>
#include <nng/supplemental/util/platform.h>

#include "csc/csc_debug_nng.h"
#include "csc/csc_crossos.h"
#include "csc/csc_malloc_file.h"
#include "csc/csc_math.h"
#include "csc/csc_linmat.h"
#include "csc/csc_m3f32.h"
#include "csc/csc_m4f32.h"
#include "csc/csc_v3f32.h"
#include "csc/csc_v4f32.h"
#include "csc/csc_qf32.h"
#include "csc/csc_filecopy.h"
#include "csc/csc_argv.h"
#include "csc/argparse.h"

#include "../shared/shared.h"
#include "calculation.h"
#include "mg_send.h"

#define ARG_HELP    UINT32_C(0x00000001)
#define ARG_VERBOSE UINT32_C(0x00000002)
#define ARG_STDIN   UINT32_C(0x00000010)
#define ARG_LEGACY_FILENAME  UINT32_C(0x00000100)


//../txtpoints/4/14_17_18_225279.txt -m1
int main (int argc, char const * argv[])
{
	csc_crossos_enable_ansi_color();
	char const * arg_filename = "../txtpoints/4/14_17_18_225279.txt";
	char const * arg_address = "tcp://localhost:9002";
	uint32_t arg_flags = 0;
	uint32_t arg_visualmode = 1;
	struct csc_argv_option option[] =
	{
	{'a', "address",         CSC_ARGV_TYPE_STRING, &arg_address,    {.val_umax = 0}, "The address to send to"},
	{'f', "legacy_filename", CSC_ARGV_TYPE_STRING, &arg_filename,   {.val_umax = 0}, "The filename to load legacy pointcloud"},
	{'h', "help",            CSC_ARGV_TYPE_U32,    &arg_flags,      {.val_u32 = ARG_HELP}, "Show help"},
	{'v', "verbose",         CSC_ARGV_TYPE_U32,    &arg_flags,      {.val_u32 = ARG_VERBOSE}, "Show verbose"},
	{'i', "input",           CSC_ARGV_TYPE_U32,    &arg_flags,      {.val_u32 = ARG_STDIN}, "Get pointcloud from stdin"},
	{'L', "legacy_filename", CSC_ARGV_TYPE_U32,    &arg_flags,      {.val_u32 = ARG_LEGACY_FILENAME}, "ARG_LEGACY_FILENAME"},
	{'m', "mode",            CSC_ARGV_TYPE_U32,    &arg_visualmode, {.val_umax = 0}, "The visual mode"},
	{.type = CSC_ARGV_TYPE_END}};
	csc_argv_parsev (option, argv+1);
	if (arg_flags & ARG_HELP)
	{
		csc_argv_print_description (option);
		csc_argv_print_value (option);
		return 0;
	}

	nng_socket sock;
	mg_pairdial (&sock, arg_address);
	show_init (sock);


	struct skitrack1 s1 = {0};
	struct skitrack2 s2 = {0};
	if (arg_flags & ARG_STDIN)
	{
		while (1)
		{
			int r = fread (s1.pc, sizeof (float) * LIDAR_WH * POINT_STRIDE, 1, stdin);
			ASSERTF (r == 1, "%i", r);
			s1.pc_count = LIDAR_WH;
			show (&s1, &s2, sock, arg_visualmode);
		}
	}
	else if ((arg_flags & ARG_LEGACY_FILENAME) && arg_filename)
	{
		legacy_points_read_filename (arg_filename, s1.pc, &s1.pc_count);
		printf ("pc_count %i\n", s1.pc_count);
		show (&s1, &s2, sock, arg_visualmode);
	}
	else if (arg_filename)
	{
		printf ("Opening binary file %s to read LiDAR frames.\n", arg_filename);
		FILE * f = fopen (arg_filename, "rb");
		ASSERT_NOTNULL (f);
		while (1)
		{
			int r = fread (s1.pc, sizeof (float) * LIDAR_WH * POINT_STRIDE, 1, f);
			ASSERTF (r == 1, "%i", r);
			s1.pc_count = LIDAR_WH;
			show (&s1, &s2, sock, arg_visualmode);
			usleep(10000);
		}
	}


	nng_close (sock);
	return 0;
}
