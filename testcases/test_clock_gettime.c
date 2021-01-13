#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include "../csc/csc_debug.h"
#include "../csc/csc_crossos.h"


int main()
{
	csc_crossos_enable_ansi_color();

	{
		struct timespec begin, end;
		clock_gettime (CLOCK_REALTIME, &begin);
		sleep(1);
		clock_gettime (CLOCK_REALTIME, &end);
		double d = ((end.tv_nsec - begin.tv_nsec) / 1000000000.0) + (end.tv_sec  - begin.tv_sec);
		ASSERTF ((d < 1.1) && (d > 0.9), "clock_gettime not right. d = %lf", d);
	}
}
