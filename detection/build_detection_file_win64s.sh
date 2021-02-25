gcc pointfile.c -odetection_win64s.exe \
-Wall \
-Wno-unused-function \
-D__USE_MINGW_ANSI_STDIO=1 \
-DNNG_STATIC_LIB \
-IC:/msys64/mingw64/include \
-I../csc \
-LC:/msys64/mingw64/lib \
-Wl,-Bstatic \
-lnng \
-lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32 \
-lopenblas \
-lm

cp detection_win64s.exe ../win64s/detection.exe