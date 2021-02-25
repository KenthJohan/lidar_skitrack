#pragma once

#include <stdint.h>
#include <stdio.h>

#include "csc/csc_malloc_file.h"

void points_read (char const s[], float p[], uint32_t *n)
{
	uint32_t i = 0;
	float v[4] = {0.0f};
	while (s[0] != '\0')
	{
		char * e;//Used for endptr of float token
		v[0] = strtof (s, &e);//Convert string to float starting from (s)
		if (e == s) {s++; continue;}//If parse fails then try again
		s = e;//Parse success goto to next token
		v[1] = strtof (s, &e);//Convert string to float starting from (s)
		if (e == s) {s++; continue;}//If parse fails then try again
		s = e;//Parse success goto to next token
		v[2] = strtof (s, &e);//Convert string to float starting from (s)
		if (e == s) {s++; continue;}//If parse fails then try again
		s = e;//Parse success goto to next token
		memcpy (p, v, sizeof(v));//If a entire point (v) got successfully parsed then copy this point into the point array (p)
		p += 4;//The point array (p) consist of 4 dim points
		i++;//Keep track of how many points got parsed
	}
	(*n) = i;
}


void legacy_points_read_filename (char const * filename, float p[], uint32_t *n)
{
	ASSERT_PARAM_NOTNULL (filename);
	ASSERT_PARAM_NOTNULL (p);
	ASSERT_PARAM_NOTNULL (n);
	char const * text = csc_malloc_file (filename);
	points_read (text, p, n);
	free ((void*)text);
}


void points_print (float p[], uint32_t n)
{
	for (uint32_t i = 0; i < n; ++i)
	{
		printf ("%f %f %f\n", p[0], p[1], p[2]);
		p += 4;
	}
}
