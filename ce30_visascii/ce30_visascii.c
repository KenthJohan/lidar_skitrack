#include "csc_crossos.h"
#include "csc_basic.h"
#include "csc_debug.h"
#include "csc_malloc_file.h"
#include "csc_gl.h"
#include "csc_math.h"
#include "csc_argv.h"
#include "csc_debug_nng.h"

#include <SDL2/SDL.h>
#include <GL/glew.h>

#include <stdio.h>
#include <unistd.h>//chdir()

#include <nng/nng.h>
#include <nng/protocol/pair0/pair.h>
#include <nng/supplemental/util/platform.h>

#include "../ce30_driver/ce30.h"

#define ARG_HELP    UINT32_C(0x00000001)
#define ARG_VERBOSE UINT32_C(0x00000002)


int main (int argc, char * argv[])
{
	csc_crossos_enable_ansi_color ();
	ASSERT (argc);
	ASSERT (argv);
	setbuf (stdout, NULL);

	char const * arg_filename = NULL;
	uint32_t arg_flags = 0;
	struct csc_argv_option option[] =
	{
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

	FILE * f = stdin;
	if (arg_filename != NULL)
	{
		f = fopen (arg_filename, "rb");
		ASSERT_NOTNULL (f);
	}

	float frame[CE30_WIDTH*CE30_HEIGHT*4];
	size_t r;
	while(1)
	{
		size_t r = fread (frame, sizeof (frame), 1, f);
		if (r != 1) {break;}
		//printf ("r %i\n", r);
		for (uint32_t x = 0; x < CE30_WIDTH; ++x)
		{
			for (uint32_t y = 0; y < CE30_HEIGHT; ++y)
			{
				float * p = frame + CE30_XY_INDEX (x,y);
				printf ("%10.5f %10.5f %10.5f %10.5f\n", p[0], p[1], p[2], p[3]);
			}
		}
	}


	return 0;
}
