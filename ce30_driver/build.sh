#sudo apt-get update
#sudo apt-get install build-essential
#sudo apt-get install libboost-all-dev
g++ -c -I.. ./utils.cpp
g++ -c -I.. udp_socket.cpp
g++ -c -I.. timed_udp_socket.cpp
g++ -c -I.. data_types.cpp
g++ -c -I.. packet.cpp
g++ -c -I.. udp_server.cpp
g++ -shared utils.o udp_socket.o timed_udp_socket.o data_types.o packet.o udp_server.o -o libce30_driver.so
mv libce30_driver.so /usr/lib
#g++ -o ce30_demo2 utils.o udp_socket.o timed_udp_socket.o data_types.o packet.o udp_server.o ce30_demo2.o -lboost_system -lpthread