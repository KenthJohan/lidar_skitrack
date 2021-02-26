g++ -oce30_recorder \
ce30_recorder.cpp \
../ce30_driver/utils.cpp \
../ce30_driver/udp_socket.cpp \
../ce30_driver/timed_udp_socket.cpp \
../ce30_driver/data_types.cpp \
../ce30_driver/packet.cpp \
../ce30_driver/udp_server.cpp \
-I.. -lboost_system -lpthread
