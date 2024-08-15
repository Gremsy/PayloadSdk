# Mavlink library v2.0

## Introduction

This repository contains mavlink library and code test.<br/>
To include this library into the main project, add it into the CMakeLists.txt in the root directory:
```
add_subdirectory(<path-to-mavlink-directory>/libs)
```

## Test on Linux PC Serial
1. Install **socat** application to create a bridge between UDP and virtual comport
```
sudo apt-get update && sudo apt-get install socat
```
After installing completed, start to create new bridge:
```
sudo socat UDP:localhost:14550 pty,link=/dev/virtualcom0,raw
```

2. Download [QGround Control](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu) and run
```
./QGroundControl.AppImage
```

3. Create build folder, compile source code and run<br/>
Compile:
```
cd cameraManager
mkdir build
cd build
cmake ../
make
```
Then run:
```
sudo ./test/cameraManager-test
```

5. Verify operation<br/>
In the QGround Control application, click on the Q symbol on the top left, choose Analyze Tools -> MAVLink Inspector.<br/>
Then observe whether the test companion ID (MAV_COMP_ID_CAMERA6 = 105) is shown on the app screen.

## Test on Linux PC UDP
1. change in file mavlink_test.h
Mavlinklib_Test(const mavlink_dev_info_t *mav_dev_info) : \
                    Mavlink_Interface(mav_dev_info, false) {
        MAVLINK_LOG("Init Mavlinklib_Test class!");
    };

2. Download [QGround Control](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu) and run
```
./QGroundControl.AppImage
```

3. Create build folder, compile source code and run<br/>
Compile:
```
cd cameraManager
mkdir build
cd build
cmake ../
make
```
Then run:
```
sudo ./tests/mavlink-test -d 127.0.0.1 -b 14550
```

5. Verify operation<br/>
In the QGround Control application, click on the Q symbol on the top left, choose Analyze Tools -> MAVLink Inspector.<br/>
Then observe whether the test companion ID (MAV_COMP_ID_CAMERA6 = 105) is shown on the app screen.
