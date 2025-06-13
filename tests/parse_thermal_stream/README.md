# EXAMPLE RECEIVE RAW DATA SOCKET

### Install OpenCV
```
sudo apt-get install libopencv-dev
```

### Change IP Vio Payload
- Change `define PAYLOAD_IP` on `main.h` file to connect to Vio Payload

### Build and Run example
```
mkdir build && cd build
cmake ..
make
./parseThermalStream
```