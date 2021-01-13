g++ -oce30_demo ce30_demo.cpp -I.. -lce30_driver -lboost_system -lpthread
g++ -oce30_demo2 ce30_demo2.cpp -I.. -lce30_driver -lboost_system -lpthread

#g++ -Wl,--dynamicbase,--high-entropy-va,--nxcompat,--default-image-base-high -Wl,-subsystem,console -mthreads -o 
#debug/ce30_demo.exe debug/utils.o debug/udp_socket.o debug/udp_socket_win_impl.o debug/timed_udp_socket.o 
#debug/data_types.o debug/packet.o debug/udp_server.o debug/ce30_demo.o  -LC:/msys64/mingw64/lib -lboost_system-mt -lm -lWs2_32 