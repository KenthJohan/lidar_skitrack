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

//pacman -S mingw64/mingw-w64-x86_64-openblas
//-lopenblas
#include <OpenBLAS/lapack.h>
#include <OpenBLAS/lapacke.h>
#include <OpenBLAS/cblas.h>

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

#include "../shared/shared.h"
#include "calculation.h"
#include "mg_send.h"


//../txtpoints/4/14_17_18_225279.txt
int main (int argc, char const * argv[])
{
	ASSERT (argc);
	ASSERT (argv);
	csc_crossos_enable_ansi_color();
	nng_socket sock;
	mg_pairdial (&sock,"tcp://localhost:9002");
	ASSERT_PARAM_NOTNULL (argv[1]);
	uintmax_t visual_mode = VISUAL_MODE_IMG1;
	if (argc >= 3 && argv[2])
	{
		char * e;
		visual_mode = strtoumax (argv[2], &e, 0);
		ASSERT (visual_mode <= UINT32_MAX);
	}
	show (argv[1], sock, (uint32_t)visual_mode);
	nng_close (sock);
	return 0;
}
