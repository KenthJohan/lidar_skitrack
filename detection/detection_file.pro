TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += static

DEFINES += USING_QT_CREATOR
DEFINES += NNG_STATIC_LIB

QMAKE_CFLAGS += -Wno-unused-function

SOURCES += detection_file.c
HEADERS += calculation.h
HEADERS += myent.h
HEADERS += mathmisc.h
HEADERS += skitrack.h
HEADERS += mg_comp.h
HEADERS += mg_attr.h
HEADERS += ../shared/shared.h
HEADERS += ../shared/log.h

HEADERS += csc/csc_math.h
HEADERS += csc/csc_linmat.h
HEADERS += csc/csc_m3f32.h
HEADERS += csc/csc_m3f32_print.h
HEADERS += csc/csc_m4f32.h
HEADERS += csc/csc_v3f32.h
HEADERS += csc/csc_v3f32_print.h
HEADERS += csc/csc_qf32.h


INCLUDEPATH += C:/msys64/mingw64/include

LIBS += -LC:/msys64/mingw64/lib

LIBS += -Wl,-Bstatic
LIBS += -lnng
LIBS += -lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32
#LIBS += -Wl,-Bdynamic

LIBS += -lopenblas
#LIBS += -llapack
#LIBS += -llapacke
#LIBS += -lcblas
#LIBS += -lblas
LIBS += -lm
