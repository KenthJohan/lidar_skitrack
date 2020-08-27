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



#define GL_PBOINFO_MAX 10
struct gl_pboinfo
{
	//Size of each pbo in bytes:
	uint32_t size8[GL_PBOINFO_MAX];
	//PBO provides a memory mapping mechanism to map the OpenGL controlled buffer object to the client's memory address space.
	//So, the client can modify a portion of the buffer object or the entire buffer by using glMapBuffer() and glUnmapBuffer().
	uint32_t pbo[GL_PBOINFO_MAX];
};


void gl_pboinfo_init (struct gl_pboinfo * info)
{
	glGenBuffers (GL_PBOINFO_MAX, info->pbo);
}



void gl_pboinfo_allocate (struct gl_pboinfo * info, uint32_t index, uint32_t size8, void * data)
{
	printf ("gl_pboinfo_push index %jd %jd\n", (intmax_t)index, (intmax_t)size8);
	uint32_t pbo = info->pbo[index];
	if (index >= GL_PBOINFO_MAX)
	{
		printf ("gl_pboinfo_push max number of pixel buffer object reached!");
		return;
	}
	glBindBuffer (GL_PIXEL_UNPACK_BUFFER, pbo);
	glBufferData (GL_PIXEL_UNPACK_BUFFER, size8, data, GL_STREAM_DRAW);
	glBindBuffer (GL_PIXEL_UNPACK_BUFFER, 0);
	info->size8[index] = size8;
}



/**
 * @brief Data recived from the socket is put directly into a pixel buffer object (PBO)
 * @param info
 * @param index
 * @param sock
 */
void gl_pboinfo_nng_recv (struct gl_pboinfo * info, uint32_t index, nng_socket sock)
{
	ASSERT (index < GL_PBOINFO_MAX);

	uint32_t pbo = info->pbo[index];
	ASSERT (glIsBuffer (pbo));

	uint32_t size8 = info->size8[index];
	ASSERT (size8 > 0);

	//Bind the PBO to which we want to write to:
	glBindBuffer (GL_PIXEL_UNPACK_BUFFER, pbo);

	//Note that glMapBuffer() causes sync issue.
	//If GPU is working with this buffer, glMapBuffer() will wait(stall)
	//until GPU to finish its job. To avoid waiting (idle), you can call
	//first glBufferData() with NULL pointer before glMapBuffer().
	//If you do that, the previous data in PBO will be discarded and
	//glMapBuffer() returns a new allocated pointer immediately
	//even if GPU is still working with the previous data.
	glBufferData (GL_PIXEL_UNPACK_BUFFER, size8, 0, GL_STREAM_DRAW);

	//map all or part of a buffer object's data store into the client's address space
	void * mem = glMapBufferRange (GL_PIXEL_UNPACK_BUFFER, 0, size8, GL_MAP_WRITE_BIT);
	ASSERT (mem);

	/*
	for (uint32_t i = 0; i < size8; ++i)
	{
		ptr[i] = rand();
	}
	*/


	int rv;
	size_t sz = size8;

	//NNG_FLAG_NONBLOCK:The function returns immediately, regardless of whether the socket is able to accept the data or not.
	rv = nng_recv (sock, mem, &sz, NNG_FLAG_NONBLOCK);


	//If the socket is unable to accept the data
	//(such as if backpressure exists because the peers are consuming messages too slowly, or no peer is present),
	//then the function will return with NNG_EAGAIN
	if (rv == NNG_EAGAIN)
	{
		//TODO: What should we do here?
	}
	//Check socket error:
	else if (rv != 0)
	{
		NNG_EXIT_ON_ERROR (rv);
	}
	//If socket recieved data successfully:
	else
	{
		//printf ("size8: %jd. recv: %jd\n", (intmax_t)size8, (intmax_t)sz);
		ASSERTF (size8 == sz, "%jd == %jd", (intmax_t)size8, (intmax_t)sz);
	}

	//release pointer to mapping buffer (mem)
	mem = NULL;
	glUnmapBuffer (GL_PIXEL_UNPACK_BUFFER);
}

