gcc main_demo1.c -ovisualizer_linux64 \
-DNNG_STATIC_LIB \
-I../csc \
-lglew32 -lopengl32 \
-lmingw32 -lSDL2main -lSDL2 \
-lnng \