#include <unistd.h>
#include <stdio.h>

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

#ifdef USING_QT_CREATOR
	//chdir ("../detection/");
#endif

	csc_crossos_enable_ansi_color();

	nng_socket socks[MAIN_NNGSOCK_COUNT] = {{0}};
	main_nng_pairdial (socks + MAIN_NNGSOCK_POINTCLOUD_POS, "tcp://localhost:9002");
	main_nng_pairdial (socks + MAIN_NNGSOCK_POINTCLOUD_COL, "tcp://localhost:9003");
	main_nng_pairdial (socks + MAIN_NNGSOCK_TEX,            "tcp://localhost:9004");
	main_nng_pairdial (socks + MAIN_NNGSOCK_VOXEL,          "tcp://localhost:9005");
	main_nng_pairdial (socks + MAIN_NNGSOCK_LINE_POS,       "tcp://localhost:9006");
	main_nng_pairdial (socks + MAIN_NNGSOCK_LINE_COL,       "tcp://localhost:9007");

	chdir ("../txtpoints/4");
	//show ("14_13_57_24145.txt", socks);
	//show ("14_13_55_22538.txt", socks);
	//show ("14_13_53_20565.txt", socks);
	//show ("14_13_52_19801.txt", socks);
	//show ("14_13_53_20906.txt", socks);
	//show ("14_13_55_22978.txt", socks);
	//show ("14_13_58_25517.txt", socks);
	//show ("14_13_53_20783.txt", socks);
	//show ("14_13_55_22978.txt", socks);
	//show ("14_13_54_21339.txt", socks);
	//show ("14_13_59_26063.txt", socks);
	//show ("14_16_57_204577.txt", socks);
	//return 0;


	FILE * f = popen ("ls", "r");
	ASSERT (f);
	char buf[200] = {'\0'};
	while (1)
	{
		int c = getchar();
		switch (c)
		{
		case 'q':
			goto exit_while;
			break;
		case '\n':
		case 'n':
			if (fgets (buf, sizeof (buf), f) == NULL) {goto exit_while;};
			buf[strcspn(buf, "\r\n")] = 0;
			printf ("Examining LiDAR point file: %s\n", buf);
			show (buf, socks, VISUAL_MODE_IMG1);
			break;
		case 'c':
			//copy_file (buf, "../txtpoints2");
			break;
		}
	}
exit_while:
	pclose (f);

	nng_close (socks[MAIN_NNGSOCK_POINTCLOUD_POS]);
	nng_close (socks[MAIN_NNGSOCK_POINTCLOUD_COL]);
	nng_close (socks[MAIN_NNGSOCK_TEX]);
	nng_close (socks[MAIN_NNGSOCK_VOXEL]);
	nng_close (socks[MAIN_NNGSOCK_LINE_POS]);
	nng_close (socks[MAIN_NNGSOCK_LINE_COL]);

	return 0;
}
