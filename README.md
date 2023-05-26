# PayloadSdk
This repo is officially SDK for all Gremsy's Payloads

## Hardware
  This source code can run on any Linux system  
  In this repo, all examples were tested on Jetson Xavier NX (Jetpack 4.6)

## Clone the project 
```
git clone -b develop --recurse-submodules git@github.com:Gremsy/PayloadSdk.git
```
## How to build
```
cd PayloadSdk/src  
mkdir build && cd build  
cmake ..  
make -j6  
```