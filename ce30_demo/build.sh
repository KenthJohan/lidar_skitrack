#sudo apt-get update
#sudo apt-get install build-essential
#sudo apt-get install libboost-all-dev
g++ -c -I.. ../ce30_driver/utils.cpp
g++ -c -I.. ../ce30_driver/udp_socket.cpp
g++ -c -I.. ../ce30_driver/timed_udp_socket.cpp
g++ -c -I.. ../ce30_driver/data_types.cpp
g++ -c -I.. ../ce30_driver/packet.cpp
g++ -c -I.. ../ce30_driver/udp_server.cpp
g++ -c -I.. ce30_demo2.cpp
g++ udp_socket.o utils.o timed_udp_socket.o data_types.o packet.o udp_server.o ce30_demo2.o -lboost_system -lm -pthread


#g++ -Wl,--dynamicbase,--high-entropy-va,--nxcompat,--default-image-base-high -Wl,-subsystem,console -mthreads -o 
#debug/ce30_demo.exe debug/utils.o debug/udp_socket.o debug/udp_socket_win_impl.o debug/timed_udp_socket.o 
#debug/data_types.o debug/packet.o debug/udp_server.o debug/ce30_demo.o  -LC:/msys64/mingw64/lib -lboost_system-mt -lm -lWs2_32 