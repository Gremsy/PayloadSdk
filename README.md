# PayloadSdk
This repo is officially SDK for all Gremsy's Payloads

## Hardware
- Ubuntu PC (x86_64)
- Jetson platform (aarch64)
- Raspberry Pi (developing)

## Clone the project 
```
git clone -b develop --recurse-submodules https://github.com/Gremsy/PayloadSdk.git

cd PayloadSdk/libs/gSDK/
git checkout gSDK_V3_alpha
git pull origin gSDK_V3_alpha
```
## How to build
- Install required lib
```
sudo apt-get install libcurl4-openssl-dev libjsoncpp-dev
```

- Build project
```
cd PayloadSdk 
mkdir build && cd build  
cmake ..  
make -j6  
```
