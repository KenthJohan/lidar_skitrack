gcc main_demo1.c -ographics_server.exe -DNNG_STATIC_LIB -DGLEW_STATIC -IC:/msys64/mingw64/include -LC:/msys64/mingw64/lib \
-lmingw32 -lSDL2main -lSDL2 -mwindows -Wl,--no-undefined -Wl,--dynamicbase -Wl,--nxcompat -Wl,--high-entropy-va -lm -ldinput8 -ldxguid -ldxerr8 -luser32 -lgdi32 -lwinmm -limm32 -lole32 -loleaut32 -lshell32 -lsetupapi -lversion -luuid \
-static-libgcc \
-lglew32 -lopengl32 \
-lnng \
-lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32