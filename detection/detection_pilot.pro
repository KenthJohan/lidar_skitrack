TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CFLAGS += -Wno-unused-function

SOURCES += detection_pilot.c
HEADERS += calculation.h
HEADERS += mathmisc.h
HEADERS += skitrack.h
HEADERS += csc/csc_math.h
HEADERS += csc/csc_v3f32.h
HEADERS += csc/csc_linmat.h
HEADERS += csc/csc_m3f32.h
HEADERS += csc/csc_m4f32.h

LIBS += -lmosquitto
LIBS += -lopenblas
LIBS += -lm


