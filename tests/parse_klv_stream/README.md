# KLV Footprint Map

Sample code for parsing KLV stream metadata and displaying GPS footprint on a map.

## Requirements

```bash
sudo apt install -y python3 python3-pip
pip3 install flask
```

## Build

```bash
cd PayloadSdk
mkdir build
cd build

cmake -DMB1=1 ../
make -j$(nproc)
```

## Run KLV Parser

```bash
./tests/parse_klv_stream/Test_Decode_KLV
```

Enable debug log:

```bash
GAPP_LOG_LEVEL=LOG_DEBUG ./tests/parse_klv_stream/Test_Decode_KLV
```

## Run GPS Footprint Map

In another terminal:

```bash
cd PayloadSdk
python3 tests/parse_klv_stream/map_viewer/map_viewer_app.py
```
The Flask application provides the GPS footprint map through the local web server.

## Map Viewer Execution Result

![alt text](map_viewer_1.png)

![alt text](map_viewer_2.png)

