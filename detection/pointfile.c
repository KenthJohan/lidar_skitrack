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

#include "csc_debug_nng.h"
#include "csc_crossos.h"
#include "csc_malloc_file.h"
#include "csc_math.h"
#include "csc_linmat.h"
#include "csc_m3f32.h"
#include "csc_m4f32.h"
#include "csc_v3f32.h"
#include "csc_v4f32.h"
#include "csc_qf32.h"
#include "csc_filecopy.h"

#include "calculation.h"



int main (int argc, char const * argv[])
{
	ASSERT (argc);
	ASSERT (argv);
	csc_crossos_enable_ansi_color();
	nng_socket socks[MAIN_NNGSOCK_COUNT] = {{0}};
	main_nng_pairdial (socks + MAIN_NNGSOCK_POINTCLOUD_POS, "tcp://localhost:9002");
	main_nng_pairdial (socks + MAIN_NNGSOCK_POINTCLOUD_COL, "tcp://localhost:9003");
	main_nng_pairdial (socks + MAIN_NNGSOCK_TEX,            "tcp://localhost:9004");
	main_nng_pairdial (socks + MAIN_NNGSOCK_VOXEL,          "tcp://localhost:9005");
	main_nng_pairdial (socks + MAIN_NNGSOCK_LINE_POS,       "tcp://localhost:9006");
	main_nng_pairdial (socks + MAIN_NNGSOCK_LINE_COL,       "tcp://localhost:9007");
	ASSERT_PARAM_NOTNULL (argv[1]);
	uintmax_t visual_mode = VISUAL_MODE_IMG1;
	if (argc >= 3 && argv[2])
	{
		char * e;
		visual_mode = strtoumax (argv[2], &e, 0);
		ASSERT (visual_mode <= UINT32_MAX);
	}
	show (argv[1], socks, (uint32_t)visual_mode);
	nng_close (socks[MAIN_NNGSOCK_POINTCLOUD_POS]);
	nng_close (socks[MAIN_NNGSOCK_POINTCLOUD_COL]);
	nng_close (socks[MAIN_NNGSOCK_TEX]);
	nng_close (socks[MAIN_NNGSOCK_VOXEL]);
	nng_close (socks[MAIN_NNGSOCK_LINE_POS]);
	nng_close (socks[MAIN_NNGSOCK_LINE_COL]);
	return 0;
}
