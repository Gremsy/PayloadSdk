# PayloadSdk PyMavLink
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
git clone -b develop ssh://git@gitlab.gremsy.vn:2224/ai/tay-cu/pymavlink.git
```

## Setup Environment
You can set up the environment using either `conda` or `virtualenv`.

### Using Conda
```shell
cd PayloadSdk/
conda create -n payloadsdk_env python=3.8
conda activate payloadsdk_env
```

### Using Virtualenv
```shell
cd PayloadSdk/
python3 -m venv payloadsdk_env
source payloadsdk_env/bin/activate
```

## How to build and run example
- Navigate to the cloned directory and install required packages:

```shell
cd PayloadSdk/
pip install -r requirements.txt
```

- Run the example:
```shell
python3 payload_do_object_detection.py
```