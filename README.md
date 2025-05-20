# PayloadSdk Python
This repo is officially SDK for all Gremsy's Payloads using PyMavLink.

## Hardware
- Ubuntu PC (x86_64)
- Jetson platform (aarch64)
- Raspberry Pi
- Qualcomm RB5165

## Software
This branch supports:
- Vio payload: Software v2.0.0 or higher
- Zio payload: not supported yet
- GHardron payload: not supported yet
- OrusL payload: Software v2.0.0 or higher
- Supported Python versions: 3.6, 3.7, 3.8, 3.9, 3.10, 3.11, 3.12

## Clone the project 
```shell
git clone --recurse-submodules -b python_sdk_v3 git@github.com:Gremsy/PayloadSdk.git
```

## Install required libraries
After cloning the submodule PayloadSdk C++, install the required libraries:

```shell
sudo apt-get install libcurl4-openssl-dev libjsoncpp-dev
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

## Setup Environment
You can set up the environment using either `conda` or `virtualenv`.

### Using Conda
```shell
cd PayloadSdk/
conda create -n payloadsdk_env python=3.8
conda activate payloadsdk_env
pip install -r requirements.txt
```

### Using Virtualenv
```shell
cd PayloadSdk/
python3 -m venv payloadsdk_env
source payloadsdk_env/bin/activate
pip install -r requirements.txt
```

## How to build and run example
- Navigate to the cloned directory and install required packages:

```shell
cd PayloadSdk/
pip3 install -r requirements.txt
```

- Run the example:
```shell
python3 payload_do_object_detection.py
```