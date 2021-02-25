TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

#DEFINES += UDP_SOCKET_USE_BOOST_API
DEFINES += UDP_SOCKET_USE_WINSOCK2_API
DEFINES += __USE_MINGW_ANSI_STDIO=1
DEFINES += _POSIX_C_SOURCE
DEFINES += NNG_STATIC_LIB

QMAKE_CFLAGS += -Wno-unused-function

INCLUDEPATH += ../
SOURCES += ../ce30_driver/utils.cpp
SOURCES += ../ce30_driver/udp_socket.cpp
SOURCES += ../ce30_driver/timed_udp_socket.cpp
SOURCES += ../ce30_driver/data_types.cpp
SOURCES += ../ce30_driver/packet.cpp
SOURCES += ../ce30_driver/udp_server.cpp
SOURCES += detection_ce30.cpp

LIBS += -lboost_system-mt -lm -lWs2_32

LIBS += -Wl,-Bstatic
LIBS += -lnng
LIBS += -lws2_32 -lmswsock -ladvapi32 -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32

LIBS += -lopenblas
LIBS += -lm
