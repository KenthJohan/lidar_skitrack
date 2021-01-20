TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += BUILD_DATE='"$(shell date)"'
DEFINES += GIT_VERSION='"$(git describe --always)"'
DEFINES += NNG_STATIC_LIB
DEFINES += GLEW_STATIC
DEFINES += CSC_SRCDIR=\\\"../ce30_visual/\\\"

QMAKE_CFLAGS += -Wno-unused-function

SOURCES += ce30_visual.c

INCLUDEPATH += ../csc

#LIBS += -lmingw32 -lSDL2main -lSDL2 -lopengl32 -lglew32 -lnng

LIBS += -Wl,-Bstatic
LIBS += -lmingw32 -lSDL2main -lSDL2 -mwindows -Wl,--no-undefined -Wl,--dynamicbase -Wl,--nxcompat -Wl,--high-entropy-va -lm -ldinput8 -ldxguid -ldxerr8 -luser32 -lgdi32 -lwinmm -limm32 -lole32 -loleaut32 -lshell32 -lsetupapi -lversion -luuid -static-libgcc
#LIBS += -Wl,-Bdynamic

LIBS += -lglew32 -lopengl32

LIBS += -lnng
LIBS += -lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32


