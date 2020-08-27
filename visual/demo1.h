#pragma once

#include "csc_crossos.h"
#include "csc_basic.h"
#include "csc_debug.h"
#include "csc_malloc_file.h"
#include "csc_sdlcam.h"
#include "csc_gl.h"
#include "csc_math.h"
#include "csc_debug_nng.h"

#include <SDL2/SDL.h>
#include <GL/glew.h>
#include <stdio.h>

#include <nng/nng.h>
#include <nng/protocol/pair0/pair.h>
#include <nng/supplemental/util/platform.h>



void pair_listen (nng_socket * sock, char const * address)
{
	int r;
	r = nng_pair0_open (sock);
	NNG_EXIT_ON_ERROR (r);
	r = nng_listen (*sock, address, NULL, 0);
	NNG_EXIT_ON_ERROR (r);
}


#define NET_RECV_DISCARD 0x01

void net_recv (nng_socket sock, GLenum target, GLuint vbo, unsigned size8, int flag)
{
	int rv;
	size_t sz;
	float * val = NULL;
	rv = nng_recv (sock, &val, &sz, NNG_FLAG_ALLOC | NNG_FLAG_NONBLOCK);
	if (rv == NNG_EAGAIN)
	{
		return;
	}
	else if (rv != 0)
	{
		NNG_EXIT_ON_ERROR (rv);
	}
	glBindBuffer (target, vbo);
	GLsizeiptr length = MIN (size8, sz);
	printf ("New message %i %lli\n", size8, sz);
	if (flag & NET_RECV_DISCARD)
	{
		glBufferData (GL_PIXEL_UNPACK_BUFFER, length, 0, GL_STREAM_DRAW);
	}
	void * v = glMapBufferRange (GL_ARRAY_BUFFER, 0, length, GL_MAP_WRITE_BIT);
	if (v)
	{
		memcpy (v, val, length);
		glUnmapBuffer (target);
	}
	nng_free (val, sz);
}



static void APIENTRY openglCallbackFunction(GLenum source,GLenum type,GLuint id,GLenum severity,GLsizei length,const GLchar* message,const void* userParam)
{
	(void)source;
	(void)type;
	(void)id;
	(void)severity;
	(void)length;
	(void)userParam;
	fprintf (stderr, "%s\n", message);
	if (severity == GL_DEBUG_SEVERITY_HIGH)
	{
		fprintf (stderr, "Aborting...\n");
		abort();
	}
}


#define main_glattr_pos 0
#define main_glattr_col 1
#define main_glattr_tex 2


struct demo_mesh_rectangle
{
	unsigned cap;
	GLuint vao;
	GLuint vbop;
	GLuint vboc;
	GLuint vbot;
	GLuint texture;
	GLuint program;
	GLuint uniform_mvp;
	float model[4*4];
};

struct demo_mesh_pointcloud
{
	unsigned cap;
	GLuint vao;
	GLuint vbop;
	GLuint vboc;
	GLuint program;
	GLuint uniform_mvp;
};

struct demo_mesh_voxel
{
	unsigned cap;
	unsigned last;
	GLuint vao;
	GLuint vbop;
	GLuint vbob;
	GLuint program;
	GLuint uniform_mvp;
	GLuint texture_pallete;
	float model[4*4];
};

struct demo_mesh_lines
{
	unsigned cap;
	GLuint vao;
	GLuint vbop;
	GLuint vboc;
	GLuint program;
	GLuint uniform_mvp;
};


static void demo_mesh_pointcloud_init (struct demo_mesh_pointcloud * m)
{
	glGenVertexArrays(1, &m->vao);
	glGenBuffers(1, &m->vbop);
	glGenBuffers(1, &m->vboc);


	glBindVertexArray (m->vao);

	glBindBuffer(GL_ARRAY_BUFFER, m->vbop);
	glBufferData(GL_ARRAY_BUFFER, m->cap*sizeof(float)*4, NULL, GL_STATIC_DRAW);
	glVertexAttribPointer (main_glattr_pos, 4, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray (main_glattr_pos);

	glBindBuffer(GL_ARRAY_BUFFER, m->vboc);
	glBufferData(GL_ARRAY_BUFFER, m->cap*sizeof(uint32_t), NULL, GL_STATIC_DRAW);
	glVertexAttribPointer (main_glattr_col, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, (void*)0);
	glEnableVertexAttribArray (main_glattr_col);

	float * v;
	glBindBuffer (GL_ARRAY_BUFFER, m->vbop);
	v = (float*)glMapBufferRange (GL_ARRAY_BUFFER, 0, m->cap*sizeof(float)*4, GL_MAP_WRITE_BIT);
	for (unsigned i = 0; i < m->cap; ++i)
	{
		v[0] = ((float)rand() / (float)RAND_MAX) - 0.5f;
		v[1] = ((float)rand() / (float)RAND_MAX) - 0.5f;
		v[2] = ((float)rand() / (float)RAND_MAX) - 0.5f;
		v[3] = 1.0f;
		v += 4;
	}
	glUnmapBuffer (GL_ARRAY_BUFFER);


	glBindBuffer (GL_ARRAY_BUFFER, m->vboc);
	uint32_t * c = (uint32_t*)glMapBufferRange (GL_ARRAY_BUFFER, 0, m->cap*sizeof(uint32_t), GL_MAP_WRITE_BIT);
	for (unsigned i = 0; i < m->cap; ++i)
	{
		//       AABBGGRR
		c[i] = 0xFFFFFFFF;
	}
	glUnmapBuffer (GL_ARRAY_BUFFER);
}


static void demo_mesh_pointcloud_draw (struct demo_mesh_pointcloud * m, float * mvp)
{
	glUseProgram (m->program);
	glUniformMatrix4fv (m->uniform_mvp, 1, GL_FALSE, (const GLfloat *) mvp);
	glBindVertexArray (m->vao);
	glDrawArrays (GL_POINTS, 0, m->cap);
}


static void mesh_rectangle_set4pos (struct demo_mesh_rectangle * m, float const a[3], float const b[3], float const c[3], float const d[3])
{
	float const p[] =
	{
	a[0], a[1], a[2], 1.0f, // a, left, bottom
	b[0], b[1], b[2], 1.0f, // b, right, bottom
	c[0], c[1], c[2], 1.0f, // c, right, top
	a[0], a[1], a[2], 1.0f, // a, left, bottom
	c[0], c[1], c[2], 1.0f, // c, right, top
	d[0], d[1], d[2], 1.0f  // d, left, top
	};
	glBindBuffer (GL_ARRAY_BUFFER, m->vbop);
	GLsizeiptr length = m->cap*sizeof(float)*4;
	float * v = (float*)glMapBufferRange (GL_ARRAY_BUFFER, 0, length, GL_MAP_WRITE_BIT);
	if (v)
	{
		ASSERT ((GLsizeiptr)sizeof (p) <= length);
		memcpy(v, p, sizeof (p));
		glUnmapBuffer (GL_ARRAY_BUFFER);
	}
}


static void demo_mesh_rectangle_init (struct demo_mesh_rectangle * m, float uv)
{
	m4f32_identity (m->model);
	glGenVertexArrays(1, &m->vao);
	glGenBuffers(1, &m->vbop);
	glGenBuffers(1, &m->vboc);
	glGenBuffers(1, &m->vbot);

	float const p[] =
	{
	-0.5f, -0.5f, 0.0f, 1.0f, // left, bottom
	 0.5f, -0.5f, 0.0f, 1.0f, // right, bottom
	 0.5f,  0.5f, 0.0f, 1.0f, // right, top
	-0.5f, -0.5f, 0.0f, 1.0f, // left, bottom
	 0.5f,  0.5f, 0.0f, 1.0f, // right, top
	-0.5f,  0.5f, 0.0f, 1.0f  // left, top
	};
	float const c[] =
	{
	1.0f, 1.0f, 1.0f, 1.0f, // left, bottom
	1.0f, 1.0f, 1.0f, 1.0f, // right, bottom
	1.0f, 1.0f, 1.0f, 1.0f, // right, top
	1.0f, 1.0f, 1.0f, 1.0f, // left, bottom
	1.0f, 1.0f, 1.0f, 1.0f, // right, top
	1.0f, 1.0f, 1.0f, 1.0f  // left, top
	};
	float const t[] =
	{
	0.0f, 0.0f, // left, bottom
	uv, 0.0f, // right, bottom
	uv, uv, // right, top
	0.0f, 0.0f, // left, bottom
	uv, uv, // right, top
	0.0f, uv,  // left, top
	};

	glBindVertexArray (m->vao);

	glBindBuffer (GL_ARRAY_BUFFER, m->vbop);
	glBufferData (GL_ARRAY_BUFFER, m->cap*sizeof(float)*4, p, GL_STATIC_DRAW);
	glVertexAttribPointer (main_glattr_pos, 4, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray (main_glattr_pos);

	glBindBuffer (GL_ARRAY_BUFFER, m->vboc);
	glBufferData (GL_ARRAY_BUFFER, m->cap*sizeof(float)*4, c, GL_STATIC_DRAW);
	glVertexAttribPointer (main_glattr_col, 4, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray (main_glattr_col);

	glBindBuffer (GL_ARRAY_BUFFER, m->vbot);
	glBufferData (GL_ARRAY_BUFFER, m->cap*sizeof(float)*2, t, GL_STATIC_DRAW);
	glVertexAttribPointer (main_glattr_tex, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray (main_glattr_tex);
}


static void demo_mesh_rectangle_draw (struct demo_mesh_rectangle * m, float * mvp)
{
	glUseProgram (m->program);
	glUniform1i (glGetUniformLocation (m->program, "texture1"), 0);
	glActiveTexture (GL_TEXTURE0);
	glBindTexture (GL_TEXTURE_2D, m->texture);
	float mmvp[4*4];
	m4f32_mul (mmvp, mvp, m->model);
	glUniformMatrix4fv (m->uniform_mvp, 1, GL_FALSE, (const GLfloat *) mmvp);
	glBindVertexArray (m->vao);
	glDrawArrays (GL_TRIANGLES, 0, m->cap);
}



static void demo_mesh_lines_init (struct demo_mesh_lines * m)
{
	glGenVertexArrays(1, &m->vao);
	glGenBuffers(1, &m->vbop);
	glGenBuffers(1, &m->vboc);

	glBindVertexArray (m->vao);

	glBindBuffer (GL_ARRAY_BUFFER, m->vbop);
	glBufferData (GL_ARRAY_BUFFER, m->cap*sizeof(float)*4, NULL, GL_STATIC_DRAW);
	glVertexAttribPointer (main_glattr_pos, 4, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray (main_glattr_pos);

	glBindBuffer(GL_ARRAY_BUFFER, m->vboc);
	glBufferData(GL_ARRAY_BUFFER, m->cap*sizeof(uint32_t), NULL, GL_STATIC_DRAW);
	glVertexAttribPointer (main_glattr_col, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, (void*)0);
	glEnableVertexAttribArray (main_glattr_col);
}

static void demo_mesh_lines_draw (struct demo_mesh_lines * m, float * mvp)
{
	glUseProgram (m->program);
	glUniformMatrix4fv (m->uniform_mvp, 1, GL_FALSE, (const GLfloat *) mvp);
	glBindVertexArray (m->vao);
	glDrawArrays (GL_LINES, 0, m->cap);
}




#define byte4(a,b,c,d) (((a) << 0) | ((b) << 8) | ((c) << 16) | ((d) << 24))
#define MESH_VOXEL_COUNT 36

#define MESH_VOXEL_PALLETE_W 256
#define MESH_VOXEL_PALLETE_H 1
#define MESH_VOXEL_PALLETE_C 4
#define MESH_VOXEL_PALLETE_WHC (MESH_VOXEL_PALLETE_W*MESH_VOXEL_PALLETE_H*MESH_VOXEL_PALLETE_C)
#define MESH_VOXEL_PALLETE_TYPE GL_UNSIGNED_BYTE
#define MESH_VOXEL_PALLETE_FORMAT GL_RGBA
#define MESH_VOXEL_PALLETE_UNIT 1


static void mesh_voxe_cube (uint32_t v[MESH_VOXEL_COUNT], uint8_t x, uint8_t y, uint8_t z, uint8_t type)
{
	v[0]  = byte4(x,     y,     z,     type);
	v[1]  = byte4(x,     y,     z + 1, type);
	v[2]  = byte4(x,     y + 1, z,     type);
	v[3]  = byte4(x,     y + 1, z,     type);
	v[4]  = byte4(x,     y,     z + 1, type);
	v[5]  = byte4(x,     y + 1, z + 1, type);

	v[6]  = byte4(x + 1, y,     z,     type);
	v[7]  = byte4(x + 1, y + 1, z,     type);
	v[8]  = byte4(x + 1, y,     z + 1, type);
	v[9]  = byte4(x + 1, y + 1, z,     type);
	v[10] = byte4(x + 1, y + 1, z + 1, type);
	v[11] = byte4(x + 1, y    , z + 1, type);

	v[12] = byte4(x,     y,     z,     type);
	v[13] = byte4(x,     y,     z + 1, type);
	v[14] = byte4(x + 1, y,     z,     type);
	v[15] = byte4(x + 1, y,     z,     type);
	v[16] = byte4(x,     y,     z + 1, type);
	v[17] = byte4(x + 1, y,     z + 1, type);

	v[18] = byte4(x,     y + 1, z,     type);
	v[19] = byte4(x + 1, y + 1, z,     type);
	v[20] = byte4(x,     y + 1, z + 1, type);
	v[21] = byte4(x + 1, y + 1, z,     type);
	v[22] = byte4(x + 1, y + 1, z + 1, type);
	v[23] = byte4(x,     y + 1, z + 1, type);

	v[24] = byte4(x,     y,     z,     type);
	v[25] = byte4(x,     y + 1, z,     type);
	v[26] = byte4(x + 1, y,     z,     type);
	v[27] = byte4(x + 1, y,     z,     type);
	v[28] = byte4(x,     y + 1, z,     type);
	v[29] = byte4(x + 1, y + 1, z,     type);

	v[30] = byte4(x,     y,     z + 1, type);
	v[31] = byte4(x + 1, y,     z + 1, type);
	v[32] = byte4(x,     y + 1, z + 1, type);
	v[33] = byte4(x + 1, y,     z + 1, type);
	v[34] = byte4(x + 1, y + 1, z + 1, type);
	v[35] = byte4(x,     y + 1, z + 1, type);
}


static void mesh_voxel_barycentric (uint32_t v[MESH_VOXEL_COUNT])
{
	for (int i = 0; i < 12; ++i)
	{
		v[0]  = byte4(1, 0, 0, 0);
		v[1]  = byte4(0, 1, 0, 0);
		v[2]  = byte4(0, 0, 1, 0);
		v += 3;
	}
}


static void demo_mesh_voxel_init (struct demo_mesh_voxel * m)
{
	m4f32_identity (m->model);
	glGenVertexArrays(1, &m->vao);
	glGenBuffers(1, &m->vbop);
	glGenBuffers(1, &m->vbob);
	glBindVertexArray (m->vao);

	glBindBuffer (GL_ARRAY_BUFFER, m->vbop);
	glBufferData (GL_ARRAY_BUFFER, m->cap*MESH_VOXEL_COUNT*sizeof(uint32_t), NULL, GL_STATIC_DRAW);
	glVertexAttribPointer (0, 4,  GL_UNSIGNED_BYTE, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray (0);

	glBindBuffer (GL_ARRAY_BUFFER, m->vbob);
	glBufferData (GL_ARRAY_BUFFER, m->cap*MESH_VOXEL_COUNT*sizeof(uint32_t), NULL, GL_DYNAMIC_DRAW);
	glVertexAttribPointer (1, 4,  GL_UNSIGNED_BYTE, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray (1);
	GLsizeiptr length = m->cap*MESH_VOXEL_COUNT*sizeof(uint32_t);
	uint32_t * v = (uint32_t*)glMapBufferRange (GL_ARRAY_BUFFER, 0, length, GL_MAP_WRITE_BIT);
	if (v)
	{
		for (uint32_t i = 0; i < m->cap; ++i)
		{
			mesh_voxel_barycentric (v);
			v += MESH_VOXEL_COUNT;
		}
		glUnmapBuffer (GL_ARRAY_BUFFER);
	}
}


static void demo_mesh_voxel_draw (struct demo_mesh_voxel * m, float mvp[4*4])
{
	glUseProgram (m->program);
	GLint location = glGetUniformLocation (m->program, "pallete");
	glUniform1i (location, MESH_VOXEL_PALLETE_UNIT);
	glActiveTexture (GL_TEXTURE0 + MESH_VOXEL_PALLETE_UNIT);
	glBindTexture (GL_TEXTURE_2D, m->texture_pallete);
	float mmvp[4*4];
	m4f32_mul (mmvp, mvp, m->model);
	glUniformMatrix4fv (m->uniform_mvp, 1, GL_FALSE, (const GLfloat *) mmvp);
	glBindVertexArray (m->vao);
	glDrawArrays (GL_TRIANGLES, 0, m->last*MESH_VOXEL_COUNT);
}


static void mesh_voxel_update (struct demo_mesh_voxel * m, uint8_t vox[], uint8_t nx, uint8_t ny, uint8_t nz)
{
	glBindBuffer (GL_ARRAY_BUFFER, m->vbop);
	GLsizeiptr length = m->cap*MESH_VOXEL_COUNT*sizeof(uint32_t);
	uint32_t * v = (uint32_t*)glMapBufferRange (GL_ARRAY_BUFFER, 0, length, GL_MAP_WRITE_BIT);
	m->last = 0;
	if (v)
	{
		for (uint8_t x = 0; x < nx; ++x)
		{
			for (uint8_t y = 0; y < ny; ++y)
			{
				for (uint8_t z = 0; z < nz; ++z)
				{
					uint8_t type = vox[z*nx*ny + y*nx + x];
					if (type)
					{
						mesh_voxe_cube (v, x, y, z, type);
						v += MESH_VOXEL_COUNT;
						m->last++;
					}
				}
			}
		}
		glUnmapBuffer (GL_ARRAY_BUFFER);
	}
}


static void demo_mesh_voxel_update_from_socket (struct demo_mesh_voxel * m, uint8_t nx, uint8_t ny, uint8_t nz, nng_socket sock)
{
	int rv;
	size_t sz;
	uint8_t * val = NULL;
	rv = nng_recv (sock, &val, &sz, NNG_FLAG_ALLOC | NNG_FLAG_NONBLOCK);
	if (rv == NNG_EAGAIN)
	{
		return;
	}
	else if (rv != 0)
	{
		NNG_EXIT_ON_ERROR (rv);
	}
	ASSERTF (sz == (size_t)nx*ny*nz, "%ix%ix%i=%i, sz=%i", nx, ny, nz, nx*ny*nz, sz);
	mesh_voxel_update (m, val, nx, ny, nz);
	nng_free (val, sz);
}




void demo_mesh_voxel_texture_pallete (GLuint program, GLuint tex, uint8_t data[MESH_VOXEL_PALLETE_W])
{
	//Setup Pallete texture format.
	glActiveTexture (GL_TEXTURE0 + MESH_VOXEL_PALLETE_UNIT);
	glBindTexture (GL_TEXTURE_2D, tex);
	glUseProgram (program);
	GLint location = glGetUniformLocation (program, "pallete");
	glUniform1i (location, MESH_VOXEL_PALLETE_UNIT);
	glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
	glTexImage2D (GL_TEXTURE_2D, 0, MESH_VOXEL_PALLETE_FORMAT, MESH_VOXEL_PALLETE_W, MESH_VOXEL_PALLETE_H, 0, MESH_VOXEL_PALLETE_FORMAT, MESH_VOXEL_PALLETE_TYPE, data);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glGenerateMipmap (GL_TEXTURE_2D);
}





void updatePixels (uint8_t a[], uint32_t n, uint64_t k)
{
	for (uint32_t i = 0; i < n; ++i)
	{
		a[i] = rand();
	}
}



//pbo[MAIN_GLPBO_0],TEX_W*TEX_H*4
static void net_update_texture (nng_socket sock, GLuint pbo, uint32_t size8)
{
	ASSERT (glIsBuffer (pbo));
	glBindBuffer (GL_PIXEL_UNPACK_BUFFER, pbo);
	// map the buffer object into client's memory
	// Note that glMapBuffer() causes sync issue.
	// If GPU is working with this buffer, glMapBuffer() will wait(stall)
	// for GPU to finish its job. To avoid waiting (stall), you can call
	// first glBufferData() with NULL pointer before glMapBuffer().
	// If you do that, the previous data in PBO will be discarded and
	// glMapBuffer() returns a new allocated pointer immediately
	// even if GPU is still working with the previous data.
	glBufferData (GL_PIXEL_UNPACK_BUFFER, size8, 0, GL_STREAM_DRAW);
	//GLubyte* ptr = (GLubyte*)glMapBuffer (GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
	GLubyte * ptr = (GLubyte*)glMapBufferRange (GL_PIXEL_UNPACK_BUFFER, 0, size8, GL_MAP_WRITE_BIT);
	ASSERT (ptr);


	for (uint32_t i = 0; i < size8; ++i)
	{
		//ptr[i] = rand();
	}

	//glUnmapBuffer (GL_PIXEL_UNPACK_BUFFER);
	//return;


	//update data directly on the mapped buffer
	//updatePixels (ptr, TEX_W*TEX_H*4, 1);
	//glUnmapBuffer (GL_PIXEL_UNPACK_BUFFER);
	//return;

	int rv;
	size_t sz;
	uint8_t * val = NULL;
	rv = nng_recv (sock, &val, &sz, NNG_FLAG_ALLOC | NNG_FLAG_NONBLOCK);
	//printf ("rv %i\n", rv);
	if (rv == NNG_EAGAIN)
	{}
	else if (rv != 0)
	{
		NNG_EXIT_ON_ERROR (rv);
	}
	else
	{
		ASSERT (size8 == sz);
		memcpy (ptr, val, size8);
		nng_free (val, sz);
	}
	glUnmapBuffer (GL_PIXEL_UNPACK_BUFFER);  // release pointer to mapping buffer
}


struct rendering_context
{
	uint32_t vao_cap;
	uint32_t vbo_cap;
	uint32_t program_cap;
	uint32_t * vao;
	uint32_t * vbo;
	uint32_t * program;
	char const ** program_files;
};


void rendering_context_init (struct rendering_context * ctx)
{
	//ASSERT (sizeof (uint32_t) == sizeof (GLuint));
	ASSERT (ctx->vao_cap >= 1);
	ASSERT (ctx->vbo_cap >= 1);
	ASSERT (ctx->program_cap >= 1);
	ctx->vao = (uint32_t*)calloc (ctx->vao_cap, sizeof (uint32_t));
	ctx->vbo = (uint32_t*)calloc (ctx->vbo_cap, sizeof (uint32_t));
	ctx->program = (uint32_t*)calloc (ctx->program_cap, sizeof (uint32_t));
	ctx->program_files = (char const**)calloc (ctx->program_cap, sizeof (char*));
	glGenVertexArrays (ctx->vao_cap - 1, ctx->vao + 1);
	glGenBuffers (ctx->vbo_cap - 1, ctx->vbo + 1);
	for (uint32_t i = 0; i < ctx->vao_cap; ++i)
	{
		printf ("VAO: %i\n", ctx->vao[i]);
	}
	for (uint32_t i = 0; i < ctx->vbo_cap; ++i)
	{
		printf ("VBO: %i\n", ctx->vbo[i]);
	}
}


void rendering_context_program_compile (struct rendering_context * ctx)
{
	//Start at index=1 because index=0 is reserved for default program shader:
	for (uint32_t i = 1; i < ctx->program_cap; ++i)
	{
		ctx->program[i] = csc_gl_program_from_files1 (ctx->program_files[i]);
	}
	for (uint32_t i = 1; i < ctx->program_cap; ++i)
	{
		printf ("Link OpenGL program %s\n", ctx->program_files[i]);
		glLinkProgram (ctx->program[i]);
	}
}



struct demo_texture
{
	uint32_t w;
	uint32_t h;
	uint32_t format;
	uint32_t type;
	uint32_t tex;
	uint32_t pbo;
};

void demo_texture (struct demo_texture * t)
{
	glBindTexture (GL_TEXTURE_2D, t->tex);
	glBindBuffer (GL_PIXEL_UNPACK_BUFFER, t->pbo);
	glTexSubImage2D (GL_TEXTURE_2D, 0, 0, 0, t->w, t->h, t->format, t->type, 0);
};





struct gl_mesh
{
	uint32_t vao;
	uint32_t program;
	uint32_t uniform_mvp;
	uint32_t tex_buffer;
	uint32_t tex_location;
	uint32_t tex_unit;
	uint32_t draw_mode;
	uint32_t draw_from;
	uint32_t draw_to;
	float model[4*4];
};



void gl_mesh_draw (struct gl_mesh * m, uint32_t n, float mvp[4*4])
{
	for (uint32_t i = 0; i < n; ++i, ++m)
	{
		glUseProgram (m->program);
		glUniform1i (m->tex_location, m->tex_unit);
		glActiveTexture (GL_TEXTURE0 + m->tex_unit);
		glBindTexture (GL_TEXTURE_2D, m->tex_buffer);
		float mmvp[4*4];
		m4f32_mul (mmvp, mvp, m->model);
		glUniformMatrix4fv (m->uniform_mvp, 1, GL_FALSE, (const GLfloat *) mmvp);
		glBindVertexArray (m->vao);
		glDrawArrays (m->draw_mode, m->draw_from, m->draw_to);
	}
}





struct ddgl_tex
{
	uint32_t pbo;
};







#define DD_IMG2D    1
#define DD_IMG3D    2
#define DD_VERTEX   3
#define DD_MESH     4
#define DD_STRING   5
#define DD_DATA     6


struct dd_img2d
{
	uint32_t w;
	uint32_t h;
	uint32_t format;
	uint32_t type;
	uint32_t n;
};

struct dd_img3d
{
	uint32_t w;
	uint32_t h;
	uint32_t d;
	uint32_t format;
	uint32_t type;
	uint32_t n;
};

struct dd_vertex
{
	uint32_t name;
	uint32_t dim;
	uint32_t stride;
	uint32_t type;
	uint32_t n;
};

struct dd_mesh
{
	uint32_t tex;
	uint32_t v;
	uint32_t v0;
	uint32_t v1;
	uint32_t program;
};



void * dd_img2d_calloc (struct dd_img2d * img)
{
	return calloc (1, img->w * img->h);
}


void dd_img2d_gl_allocate_tex (struct dd_img2d * img, uint32_t texture, void * data)
{
	glBindTexture (GL_TEXTURE_2D, texture);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA, img->w, img->h, 0, img->format, img->type, data);
	glGenerateMipmap (GL_TEXTURE_2D);
}


void dd_img2d_gl_allocate_pbo (struct dd_img2d * img, uint32_t buffer, void * data)
{
	glBindBuffer (GL_PIXEL_UNPACK_BUFFER, buffer);
	glBufferData (GL_PIXEL_UNPACK_BUFFER, img->w * img->h * 4, data, GL_STREAM_DRAW);
	glBindBuffer (GL_PIXEL_UNPACK_BUFFER, 0);
}


void dd_img2d_gl_copy_pbo_tex (struct dd_img2d * img, uint32_t pbo, uint32_t tex)
{
	// copy pixels from PBO to texture object
	// Use offset instead of ponter
	glBindTexture (GL_TEXTURE_2D, tex);
	glBindBuffer (GL_PIXEL_UNPACK_BUFFER, pbo);
	glTexSubImage2D (GL_TEXTURE_2D, 0, 0, 0, img->w, img->h, img->format, GL_UNSIGNED_BYTE, 0);
}




void dd_vertex_allocate (struct dd_vertex * vtx, uint32_t vbo, void * data)
{
	glBindBuffer (GL_ARRAY_BUFFER, vbo);
	glBufferData (GL_ARRAY_BUFFER, vtx->n * vtx->stride, data, GL_STATIC_DRAW);
	glVertexAttribPointer (vtx->name, vtx->dim, vtx->type, GL_FALSE, vtx->stride, (void*)0);
	glEnableVertexAttribArray (vtx->name);
}




struct ddtable
{
	uint32_t n;
	uint32_t * type;
	void ** data;
};


void ddtable_init (struct ddtable * t)
{
	t->type = (uint32_t *)calloc (t->n, sizeof (uint32_t));
	t->data = (void **)calloc (t->n, sizeof (void *));
}


void ddtable_set_type (struct ddtable * t, uint32_t i, uint32_t type)
{
	ASSERT (i < t->n);
	t->type[i] = type;
}


void ddtable_set_data (struct ddtable * t, uint32_t i, void * data)
{
	ASSERT (i < t->n);
	t->data[i] = data;
}


void ddtable_gl_allocate (struct ddtable * t)
{
	for (uint32_t i = 0; i < t->n; ++i)
	{
		switch (t->type[i])
		{
		case DD_IMG2D:
			dd_img2d_gl_allocate_tex ((struct dd_img2d*)t->data[i], 0, NULL);
			break;
		}
	}
}







/*

 1  DD_IMG2D
 2  DATA
 3  DD_IMG2D
 4  DATA
 5  DD_VERTEX
 6  DATA
 7  DD_MESH
 8  DD_STRING
 9  DATA




*/






















