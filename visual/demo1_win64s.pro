TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += USING_QT_CREATOR
DEFINES += BUILD_DATE='"$(shell date)"'
DEFINES += GIT_VERSION='"$(git describe --always)"'
DEFINES += NNG_STATIC_LIB
DEFINES += GLEW_STATIC
DEFINES += D__USE_MINGW_ANSI_STDIO=1


QMAKE_CFLAGS += -Wno-unused-function -g

SOURCES += main_demo1.c

HEADERS += csc/csc_sdlcam.h
HEADERS += csc/csc_math.h
HEADERS += csc/csc_crossos.h
HEADERS += mesh.h
HEADERS += shaper.h
HEADERS += proj1.h
HEADERS += ray_triangle_intersect.h

#contains(QT_ARCH, i386) {
#	message("32-bit")
#}else {
#	message("64-bit")
#	LIBS += -LC:\msys64\mingw64\lib
#}


INCLUDEPATH += C:/msys64/mingw64/include
INCLUDEPATH += ../csc

LIBS += -LC:\msys64\mingw64\lib

LIBS += -Wl,-Bstatic
LIBS += -lmingw32 -lSDL2main -lSDL2 -mwindows -Wl,--no-undefined -Wl,--dynamicbase -Wl,--nxcompat -Wl,--high-entropy-va -lm -ldinput8 -ldxguid -ldxerr8 -luser32 -lgdi32 -lwinmm -limm32 -lole32 -loleaut32 -lshell32 -lsetupapi -lversion -luuid
LIBS += -static-libgcc
#LIBS += -Wl,-Bdynamic
LIBS += -lglew32 -lopengl32
LIBS += -lnng
LIBS += -lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32


