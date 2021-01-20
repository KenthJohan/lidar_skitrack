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


#define GL_MESHINFO_MAX 10
#define GL_MEHSINFO_ATTRIBUTE_POS 0
#define GL_MEHSINFO_ATTRIBUTE_COL 1
struct gl_meshinfo
{
	uint32_t cap[GL_MESHINFO_MAX];
	uint32_t vao[GL_MESHINFO_MAX];
	uint32_t vbop[GL_MESHINFO_MAX];
	uint32_t vboc[GL_MESHINFO_MAX];
	uint32_t program[GL_MESHINFO_MAX];
	uint32_t uniform_mvp[GL_MESHINFO_MAX];
	uint32_t drawmode[GL_MESHINFO_MAX];
	uint32_t texture[GL_MESHINFO_MAX];
	float model[4*4*GL_MESHINFO_MAX];
};


void gl_meshinfo_init (struct gl_meshinfo * info)
{
	glGenVertexArrays (GL_MESHINFO_MAX, info->vao);
	glGenBuffers (GL_MESHINFO_MAX, info->vbop);
	glGenBuffers (GL_MESHINFO_MAX, info->vboc);
	memset (info->cap, 0, GL_MESHINFO_MAX * sizeof (uint32_t));
	for (float * m = info->model; m < info->model+4*4*GL_MESHINFO_MAX; m += 4*4)
	{
		m4f32_identity (m);
	}
}



void gl_meshinfo_allocate (struct gl_meshinfo * info, uint32_t index, uint32_t cap)
{
	ASSERT (index < GL_MESHINFO_MAX);
	uint32_t vbop = info->vbop[index];
	uint32_t vboc = info->vboc[index];
	uint32_t vao = info->vao[index];

	glBindVertexArray (vao);

	glBindBuffer (GL_ARRAY_BUFFER, vbop);
	glBufferData (GL_ARRAY_BUFFER, cap * sizeof(float) * 4, NULL, GL_STATIC_DRAW);
	glVertexAttribPointer (GL_MEHSINFO_ATTRIBUTE_POS, 4, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray (GL_MEHSINFO_ATTRIBUTE_POS);

	glBindBuffer (GL_ARRAY_BUFFER, vboc);
	glBufferData (GL_ARRAY_BUFFER, cap * sizeof(uint32_t), NULL, GL_STATIC_DRAW);
	glVertexAttribPointer (GL_MEHSINFO_ATTRIBUTE_COL, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, (void*)0);
	glEnableVertexAttribArray (GL_MEHSINFO_ATTRIBUTE_COL);

	info->cap[index] = cap;
}


void gl_meshinfo_example (struct gl_meshinfo * info, uint32_t index)
{
	ASSERT (index < GL_MESHINFO_MAX);
	uint32_t vbop = info->vbop[index];
	uint32_t vboc = info->vboc[index];
	uint32_t cap = info->cap[index];
	ASSERT (cap > 0);

	glBindBuffer (GL_ARRAY_BUFFER, vbop);
	float * v = (float*)glMapBufferRange (GL_ARRAY_BUFFER, 0, cap * sizeof(float) * 4, GL_MAP_WRITE_BIT);
	for (uint32_t i = 0; i < cap; ++i)
	{
		v[0] = ((float)rand() / (float)RAND_MAX) - 0.5f;
		v[1] = ((float)rand() / (float)RAND_MAX) - 0.5f;
		v[2] = ((float)rand() / (float)RAND_MAX) - 0.5f;
		v[3] = 1.0f;
		v += 4;
	}
	glUnmapBuffer (GL_ARRAY_BUFFER);

	glBindBuffer (GL_ARRAY_BUFFER, vboc);
	uint32_t * c = (uint32_t*)glMapBufferRange (GL_ARRAY_BUFFER, 0, cap * sizeof(uint32_t), GL_MAP_WRITE_BIT);
	for (uint32_t i = 0; i < cap; ++i)
	{
		//   = 0xAABBGGRR
		c[i] = 0xFFFFFFFF;
	}
	glUnmapBuffer (GL_ARRAY_BUFFER);
}


void gl_meshinfo_draw (struct gl_meshinfo * info, uint32_t index, float const mvp[4*4])
{
	ASSERT (index < GL_MESHINFO_MAX);
	uint32_t vao         = info->vao[index];
	uint32_t program     = info->program[index];
	uint32_t drawmode    = info->drawmode[index];
	uint32_t uniform_mvp = info->uniform_mvp[index];
	uint32_t cap         = info->cap[index];
	float * model        = info->model + index*4*4;
	glUseProgram (program);
	glUniformMatrix4fv (uniform_mvp, 1, GL_FALSE, (GLfloat const *) mvp);
	glBindVertexArray (vao);
	glDrawArrays (drawmode, 0, cap);
}


void gl_meshinfo_set_drawmode (struct gl_meshinfo * info, uint32_t index, GLuint drawmode)
{
	ASSERT (index < GL_MESHINFO_MAX);
	info->drawmode[index] = drawmode;
}


void gl_meshinfo_set_program (struct gl_meshinfo * info, uint32_t index, GLuint program)
{
	ASSERT (index < GL_MESHINFO_MAX);
	info->program[index] = program;
}


void gl_meshinfo_set_uniform_mvp (struct gl_meshinfo * info, uint32_t index, GLuint uniform_mvp)
{
	ASSERT (index < GL_MESHINFO_MAX);
	info->uniform_mvp[index] = uniform_mvp;
}


void gl_meshinfo_set_texture (struct gl_meshinfo * info, uint32_t index, GLuint texture)
{
	ASSERT (index < GL_MESHINFO_MAX);
	info->texture[index] = texture;
}

