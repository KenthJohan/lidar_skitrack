gcc main_demo1.c -ovisualizer_win64s.exe \
-Wall \
-Wno-unused-function \
-DBUILD_DATE="$(date +"%Y-%m-%d %H:%M:%S")" \
-DGIT_VERSION="$(git describe --always)" \
-D__USE_MINGW_ANSI_STDIO=1 \
-DNNG_STATIC_LIB \
-DGLEW_STATIC \
-I../csc \
-IC:/msys64/mingw64/include \
-LC:/msys64/mingw64/lib \
-Wl,-Bstatic \
-lmingw32 -lSDL2main -lSDL2 -mwindows -Wl,--no-undefined -Wl,--dynamicbase -Wl,--nxcompat -Wl,--high-entropy-va -lm -ldinput8 -ldxguid -ldxerr8 -luser32 -lgdi32 -lwinmm -limm32 -lole32 -loleaut32 -lshell32 -lsetupapi -lversion -luuid \
-static-libgcc \
-lglew32 -lopengl32 \
-lnng \
-lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32


cp visualizer_win64s.exe ../win64s/visualizer.exe
cp standard.glfs ../win64s/standard.glfs
cp standard.glvs ../win64s/standard.glvs
cp line.glfs ../win64s/line.glfs
cp line.glvs ../win64s/line.glvs
cp pointcloud.glfs ../win64s/pointcloud.glfs
cp pointcloud.glvs ../win64s/pointcloud.glvs
cp voxel.glfs ../win64s/voxel.glfs
cp voxel.glvs ../win64s/voxel.glvs