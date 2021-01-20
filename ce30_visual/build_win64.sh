gcc main_demo1.c -ovisualizer_win64.exe \
-D__USE_MINGW_ANSI_STDIO=1 \
-DNNG_STATIC_LIB \
-I../csc \
-IC:/msys64/mingw64/include \
-LC:/msys64/mingw64/lib \
-lglew32 -lopengl32 \
-lmingw32 -lSDL2main -lSDL2 \
-lnng \
-lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32 \