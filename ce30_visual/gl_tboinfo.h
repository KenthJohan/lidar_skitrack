#pragma once

#include "csc_crossos.h"
#include "csc_basic.h"
#include "csc_debug.h"
#include "csc_malloc_file.h"
#include "csc_gl.h"
#include "csc_math.h"
#include "csc_debug_nng.h"

#include <SDL2/SDL.h>
#include <GL/glew.h>
#include <stdio.h>

#include <nng/nng.h>
#include <nng/protocol/pair0/pair.h>
#include <nng/supplemental/util/platform.h>

#include "demo1.h"


#define GL_TEXINFO_MAX 10
struct gl_texinfo
{
	struct dd_img2d imginfo[GL_TEXINFO_MAX];
	uint32_t tex[GL_TEXINFO_MAX];
};


void gl_texinfo_init (struct gl_texinfo * info)
{
	glGenTextures (GL_TEXINFO_MAX, info->tex);
}


void gl_texinfo_allocate (struct gl_texinfo * info, uint32_t index, uint32_t w, uint32_t h, uint32_t format, uint32_t type, void * data)
{
	ASSERT (index < GL_TEXINFO_MAX);
	uint32_t tbo = info->tex[index];
	if (index >= GL_TEXINFO_MAX)
	{
		printf ("gl_tboinfo_push max number of texture buffer object reached!");
		return;
	}
	glBindTexture (GL_TEXTURE_2D, tbo);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, format, type, data);
	glGenerateMipmap (GL_TEXTURE_2D);
	info->imginfo[index].w = w;
	info->imginfo[index].h = h;
	info->imginfo[index].format = format;
	info->imginfo[index].type = type;
}


void gl_texinfo_cpy (struct gl_texinfo * info, uint32_t index, uint32_t pbo)
{
	ASSERT (index < GL_TEXINFO_MAX);
	uint32_t tex    = info->tex[index];
	uint32_t w      = info->imginfo[index].w;
	uint32_t h      = info->imginfo[index].h;
	uint32_t format = info->imginfo[index].format;
	uint32_t type   = info->imginfo[index].type;
	//Select texture destination (tex) and PBO source (pbo).
	glBindTexture (GL_TEXTURE_2D, tex);
	glBindBuffer (GL_PIXEL_UNPACK_BUFFER, pbo);
	//Copy pixels from the bound pixel buffer object (pbo) to the bound texture object (tex)
	glTexSubImage2D (GL_TEXTURE_2D, 0, 0, 0, w, h, format, type, 0);
	glBindTexture (GL_TEXTURE_2D, 0);
}

