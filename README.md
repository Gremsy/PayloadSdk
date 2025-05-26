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
- Supported Python versions: 3.7, 3.8, 3.9, 3.10, 3.11, 3.12

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

## Configuring Connection Settings

To connect to the payload, configure the connection settings in `payload_sdk.py`. Follow these steps to set up the connection type and parameters.

### Step 1: Set Connection Type
- Define the connection method by setting `CONTROL_METHOD` in `payload_sdk.py`:

```python
# Default connection parameters
CONTROL_UART   = 0
CONTROL_UDP    = 1
CONTROL_METHOD = CONTROL_UDP  # Set to CONTROL_UART for serial or CONTROL_UDP for UDP
```

### Step 2: Configure Connection Parameters

- For UDP Connection, If CONTROL_METHOD = CONTROL_UDP, set the IP address and port:

```python
udp_ip_target = "192.168.12.238"  # IP address of the payload
udp_port_target = 14566           # Do not change
```

- Example: To use a different IP (e.g., your local network), update udp_ip_target. For instance:

```python
udp_ip_target = "192.168.1.100"  # Replace with your payload's IP
```

- For UART Connection, If CONTROL_METHOD = CONTROL_UART, configure the serial port and baud rate:

```python
payload_uart_port = "/dev/ttyUSB0"  # Serial port (e.g., /dev/ttyUSB0, /dev/ttyACM0)
payload_uart_baud = 115200          # Baud rate
```

- Example: For a different port or baud rate:

```python
payload_uart_port = "/dev/ttyACM0"  # Update to your serial port
payload_uart_baud = 57600          # Adjust if required
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