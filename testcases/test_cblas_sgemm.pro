TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CFLAGS += -Wno-unused-function

SOURCES += test_cblas_sgemm.c

INCLUDEPATH += C:/msys64/mingw64/include
INCLUDEPATH += ..
HEADERS += ../csc/csc_math.h

LIBS += -LC:/msys64/mingw64/lib

LIBS += -Wl,-Bstatic
LIBS += -lopenblas -lm
