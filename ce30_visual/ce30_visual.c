#include "csc_crossos.h"
#include "csc_basic.h"
#include "csc_debug.h"
#include "csc_malloc_file.h"
#include "csc_sdl_motion.h"
#include "csc_gcam.h"
#include "csc_gl.h"
#include "csc_math.h"
#include "csc_sdlglew.h"
#include "csc_vertex.h"

#include <SDL2/SDL.h>
#include <GL/glew.h>
#include <stdio.h>

#include "../ce30_driver/ce30.h"


#define WIN_X SDL_WINDOWPOS_UNDEFINED
#define WIN_Y SDL_WINDOWPOS_UNDEFINED
#define WIN_W 640
#define WIN_H 480
#define WIN_TITLE "CE30_Visual"


#define main_glattr_pos 0
#define main_glattr_col 1
#define main_glattr_tex 2












void fill_texture (uint8_t * data, int w, int h, uint32_t c, uint32_t n)
{
	memset (data, 0, w * h * c);
	/*
	for (int x = 0; x < width; ++x)
	for (int y = 0; y < height; ++y)
	{
		uint8_t * p = data + (x*4) + (y*width*4);
		p[0] = 0;
		p[1] = 0;
		p[2] = 0;
		p[3] = 0;
		p[index] = 255;
	}
	*/
	//ASSERT (index < channels);
	int x = w/2;
	int y = h/2;
	int dx = 0;
	int dy = 0;
	for (uint32_t i = 0; i < n; ++i)
	{
		x += CLAMP (dx, -2, 2);
		y += CLAMP (dy, -2, 2);
		dx += (rand() % 3) - 1;
		dy += (rand() % 3) - 1;
		if (x < 0 || x >= w){dx = -dx/2;}
		if (y < 0 || y >= h){dy = -dy/2;}
		x = CLAMP (x, 0, w-1);
		y = CLAMP (y, 0, h-1);
		int i = (x*c) + (y*w*c);
		ASSERT (i >= 0);
		data[i + 0] = 255;
		//data[i + 1] = 255;
		//data[i + 2] = 255;
		//data[i + 3] = 255;
	}
}


void fill_texture2 (uint8_t * data, int w, int h, int c, uint8_t value)
{
	for (int i = 0; i < w * h * c; ++i)
	{
		data[i] = value;
	}
}




static FILE * thread1_file = NULL;
static float thread1_frame[CE30_WIDTH*CE30_HEIGHT*4] = {0};
static uint32_t thread1_color[CE30_WIDTH*CE30_HEIGHT*4] = {0};
static int thread1 (void *ptr)
{
	for (uint32_t i = 0; i < CE30_WIDTH*CE30_HEIGHT; ++i)
	{
		thread1_color[i] = 0xFFFFFFFF;
	}
	while(1)
	{
		size_t r = fread (thread1_frame, sizeof (thread1_frame), 1, thread1_file);
		if (r != 1) {break;}
		for (uint32_t x = 0; x < CE30_WIDTH; ++x)
		{
			for (uint32_t y = 0; y < CE30_HEIGHT; ++y)
			{
				//float * p = frame + CE30_XY_INDEX (x,y);
				//printf ("%10.5f %10.5f %10.5f %10.5f\n", p[0], p[1], p[2], p[3]);
			}
		}
		SDL_Delay(1000);
	}
}







int main (int argc, char * argv[])
{
	csc_crossos_enable_ansi_color ();
	ASSERT (argc);
	ASSERT (argv);



	uint32_t main_flags = CSC_SDLGLEW_RUNNING;

	SDL_Window * window;
	SDL_GLContext context;
	csc_sdlglew_create_window (&window, &context, WIN_TITLE, WIN_X, WIN_Y, WIN_W, WIN_H, SDL_WINDOW_OPENGL);


	char const * shaderfiles[] = {CSC_SRCDIR"shader.glvs", CSC_SRCDIR"shader.glfs", NULL};
	GLint glprogram = csc_gl_program_from_files (shaderfiles);
	glBindAttribLocation (glprogram, main_glattr_pos, "pos");
	glBindAttribLocation (glprogram, main_glattr_col, "col");
	glBindAttribLocation (glprogram, main_glattr_tex, "tex");
	glLinkProgram (glprogram);
	glUseProgram (glprogram);
	GLint uniform_mvp = glGetUniformLocation (glprogram, "mvp");
	GLint uniform_texture1 = glGetUniformLocation (glprogram, "texture1");
	ASSERT (uniform_mvp >= 0);
	ASSERT (uniform_texture1 >= 0);

	glEnable (GL_VERTEX_PROGRAM_POINT_SIZE);
	glEnable (GL_BLEND);
	glEnable (GL_DEPTH_TEST);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


	float const p[] =
	{
	-1.0f, -1.0f, 0.0f, 1.0f, // left, bottom
	 1.0f, -1.0f, 0.0f, 1.0f, // right, bottom
	 1.0f,  1.0f, 0.0f, 1.0f, // right, top
	-1.0f, -1.0f, 0.0f, 1.0f, // left, bottom
	 1.0f,  1.0f, 0.0f, 1.0f, // right, top
	-1.0f,  1.0f, 0.0f, 1.0f  // left, top
	};
	uint32_t const c[] =
	{
	0xAA004400, // left, bottom
	0xAA004433, // right, bottom
	0xAA004400, // right, top
	0xAAFF4400, // left, bottom
	0xAA004400, // right, top
	0xAA004433  // left, top
	};
	float const t[] =
	{
	0.0f, 0.0f, // left, bottom
	1.0f, 0.0f, // right, bottom
	1.0f, 1.0f, // right, top
	0.0f, 0.0f, // left, bottom
	1.0f, 1.0f, // right, top
	0.0f, 1.0f  // left, top
	};


	GLuint vbop;
	GLuint vboc;
	GLuint vbot;
	GLuint vao;

	glGenVertexArrays (1, &vao);
	glGenBuffers (1, &vbop);
	glGenBuffers (1, &vboc);
	glGenBuffers (1, &vbot);

	glBindVertexArray (vao);

	glBindBuffer (GL_ARRAY_BUFFER, vbop);
	glBufferData (GL_ARRAY_BUFFER, sizeof(p), p, GL_STATIC_DRAW);
	glVertexAttribPointer (main_glattr_pos, 4, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray (main_glattr_pos);

	glBindBuffer (GL_ARRAY_BUFFER, vboc);
	glBufferData (GL_ARRAY_BUFFER, sizeof(c), c, GL_STATIC_DRAW);
	glVertexAttribPointer (main_glattr_col, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, (void*)0);
	glEnableVertexAttribArray (main_glattr_col);

	glBindBuffer (GL_ARRAY_BUFFER, vbot);
	glBufferData (GL_ARRAY_BUFFER, sizeof(t), t, GL_STATIC_DRAW);
	glVertexAttribPointer (main_glattr_tex, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray (main_glattr_tex);


	//https://sites.google.com/site/john87connor/texture-object/tutorial-09-6-array-texture
	//https://community.khronos.org/t/when-to-use-glactivetexture/64913/2
	GLuint textures[1];
	{
		srand (0);
		int width = 256;
		int height = 256;
		int layers = 4;
		int channels = 4;
		unsigned size = width * height * channels * sizeof(uint8_t);
		uint8_t * data = calloc (size, 1);
		glGenTextures (1, textures);
		glActiveTexture (GL_TEXTURE0 + 0);
		glBindTexture (GL_TEXTURE_2D_ARRAY, textures[0]);//Depends on glActiveTexture()
		glTexParameteri (GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);//Depends on glBindTexture()
		glTexParameteri (GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);//Depends on glBindTexture()
		glTexParameteri (GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);//Depends on glBindTexture()
		glTexParameteri (GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);//Depends on glBindTexture()
		glTexStorage3D (GL_TEXTURE_2D_ARRAY, 1, GL_RGBA8, width, height, layers);//Depends on glBindTexture()
		//fill_texture (data, width, height, channels, 0);
		//fill_texture2 (data, width, height, channels, 255);
		fill_texture (data, width, height, channels, 200);
		glTexSubImage3D (GL_TEXTURE_2D_ARRAY, 0, 0, 0, 0, width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE, data);//Depends on glBindTexture()
		//fill_texture2 (data, width, height, channels, 200);
		fill_texture (data, width, height, channels, 200);
		glTexSubImage3D (GL_TEXTURE_2D_ARRAY, 0, 0, 0, 1, width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE, data);//Depends on glBindTexture()
		//fill_texture2 (data, width, height, channels, 100);
		fill_texture (data, width, height, channels, 200);
		glTexSubImage3D (GL_TEXTURE_2D_ARRAY, 0, 0, 0, 2, width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE, data);//Depends on glBindTexture()
		//fill_texture2 (data, width, height, channels, 50);
		fill_texture (data, width, height, channels, 200);
		glTexSubImage3D (GL_TEXTURE_2D_ARRAY, 0, 0, 0, 3, width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE, data);//Depends on glBindTexture()
	}





	struct csc_pointcloud pointcloud;
	pointcloud.count = CE30_WIDTH*CE30_HEIGHT;
	pointcloud.program = csc_gl_program_from_files1 (CSC_SRCDIR"pointcloud.glfs;"CSC_SRCDIR"pointcloud.glvs");
	ASSERT (pointcloud.program > 0);
	glLinkProgram (pointcloud.program);
	glUseProgram (pointcloud.program);
	pointcloud.uniform_mvp = glGetUniformLocation (pointcloud.program, "mvp");
	ASSERT (pointcloud.uniform_mvp >= 0);
	csc_pointcloud_init (&pointcloud);







	thread1_file = fopen ("ce30_pointcloud.out", "rb");
	ASSERT (thread1_file);
	SDL_Thread * thread = SDL_CreateThread (thread1, "TestThread", (void *)NULL);



	struct csc_gcam gcam;
	csc_gcam_init (&gcam);
	v4f32_set_xyzw (gcam.p, 0.0f, 0.0f, -4.0f, 1.0f);

	const Uint8 * keyboard = SDL_GetKeyboardState (NULL);
	while (main_flags & CSC_SDLGLEW_RUNNING)
	{
		SDL_Event event;
		while (SDL_PollEvent (&event))
		{
			csc_sdlglew_event_loop (window, &event, &main_flags, &gcam);
		}

		{
			//Control graphics camera
			csc_sdl_motion_wasd (keyboard, gcam.d);
			csc_sdl_motion_pyr (keyboard, gcam.pyrd);
			vsf32_mul (3, gcam.d, gcam.d, 0.01f);
			vsf32_mul (3, gcam.pyrd, gcam.pyrd, 0.01f);
			csc_gcam_update (&gcam);
		}

		glClearColor (0.2f, 0.3f, 0.3f, 1.0f);
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);




		glUseProgram (glprogram);
		glBindVertexArray (vao);
		glUniform1i (uniform_texture1, 0);
		glUniformMatrix4fv (uniform_mvp, 1, GL_FALSE, (const GLfloat *) gcam.mvp);
		glDrawArrays (GL_TRIANGLES, 0, 6);


		{
			csc_pointcloud_update_pos (&pointcloud, thread1_frame, sizeof(thread1_frame));
			csc_pointcloud_update_col (&pointcloud, thread1_color, sizeof(thread1_color));
			csc_pointcloud_draw (&pointcloud, gcam.mvp);
		}



		SDL_Delay (10);
		SDL_GL_SwapWindow (window);
	}

	SDL_GL_DeleteContext (context);
	SDL_DestroyWindow (window);
	SDL_Quit();
	return 0;
}
