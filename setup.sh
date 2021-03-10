sudo apt-get install -y liblapacke-dev
sudo apt-get install -y libboost-all-dev
sudo apt-get install -y cmake
sudo apt-get install -y mosquitto
sudo apt-get install -y mosquitto-dev
sudo apt-get install -y libmosquitto-dev
sudo apt-get install -y mosquitto-clients

cd ~
git clone https://github.com/nanomsg/nng
cd nng
mkdir build
cd build
sudo cmake ..
sudo make
sudo make test
sudo make install
sudo ldconfig
