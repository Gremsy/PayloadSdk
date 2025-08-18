# Gremsy Payload SDK for Python

Official Python SDK for Gremsy Payload systems using PyMAVLink protocol.

## Overview

The Gremsy Payload SDK provides a comprehensive Python interface for controlling and communicating with Gremsy payload systems. This SDK enables developers to integrate payload functionality into their applications with support for camera control, gimbal operations, GPS data transmission, and advanced features like object tracking.

## Supported Hardware

- **Computing Platforms:**
  - Ubuntu PC (x86_64)
  - NVIDIA Jetson (aarch64)
  - Raspberry Pi
  - Qualcomm RB5165

## Supported Payloads

| Payload Model | Minimum Firmware Version | Status |
|---------------|---------------------------|---------|
| VIO Payload   | v2.0.0 or higher         | ✅ Fully Supported |
| ZIO Payload   | v2.0.0 or higher         | ⚠️ Limited Support |
| GHardron Payload | v2.0.0 or higher      | ⚠️ Limited Support |

**Python Compatibility:** 3.7, 3.8, 3.9, 3.10, 3.11, 3.12

## Installation

### 1. Clone the Repository
```bash
git clone -b payloadsdk_v3_python3 https://github.com/Gremsy/PayloadSdk.git
cd PayloadSdk
```

#### Install required system dependencies
```bash
sudo apt install -y libxml2-dev libxslt-dev python3-dev gobject-introspection libgirepository1.0-dev libcairo2-dev libglib2.0-dev pkg-config meson ninja-build gir1.2-gtk-3.0 libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

### 2. Setup Python Environment

#### Option A: Using Conda (Recommended)
```bash
conda create -n payloadsdk_env python=3.8
conda activate payloadsdk_env
pip install -r requirements.txt
```

#### Option B: Using Virtual Environment
```bash
python3 -m venv payloadsdk_env
source payloadsdk_env/bin/activate  # On Windows: payloadsdk_env\Scripts\activate
pip install -r requirements.txt
```

## Project Structure

```
PayloadSdk/
├── examples/           # Example applications and use cases
├── libs/              # Core SDK libraries
│   ├── config.py      # Centralized configuration management
│   ├── payload_sdk.py # Main SDK interface
│   └── payload_define.py # Constants and enumerations
├── requirements.txt   # Python dependencies
├── PayloadSDK.md     # Detailed API documentation
└── README.md         # This file
```

## Configuration

All configuration is centralized in `libs/config.py`. The SDK automatically sets up the environment when imported.

### Connection Configuration

#### UDP Connection (Default)
```python
class ConnectionConfig:
    CONTROL_METHOD = CONTROL_UDP
    UDP_IP_TARGET = "192.168.55.1"      # Change to your payload's IP
    UDP_PORT_TARGET = 14566             # Standard MAVLink port
```

#### UART/Serial Connection
```python
class ConnectionConfig:
    CONTROL_METHOD = CONTROL_UART
    UART_PORT = "/dev/ttyUSB0"          # Linux: /dev/ttyUSB0, Windows: COM3
    UART_BAUDRATE = 115200              # Standard baudrate
```

### MAVLink Protocol Configuration

The SDK uses MAVLink 2.0 by default for enhanced features and security:

```python
class EnvironmentConfig:
    @staticmethod
    def setup_mavlink_environment():
        os.environ['MAVLINK20'] = '1'    # MAVLink 2.0 (recommended)
        os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'
```

### Advanced Configuration Options

- **Camera Settings:** Zoom/focus types, capture intervals, recording parameters
- **Gimbal Settings:** Angle limits, movement speeds, calibration parameters  
- **Communication:** Timeouts, message rates, heartbeat intervals
- **Debug Options:** Logging levels, error reporting, diagnostic output

## Quick Start

### Basic Connection Test
```bash
python3 examples/check_connect.py
```

### Example Usage in Your Code
```python
#!/usr/bin/env python3
import sys
import os

# Add SDK to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'libs'))
from config import config
from payload_sdk import PayloadSdkInterface

# Create and initialize payload interface
payload = PayloadSdkInterface()

# Establish connection
if not payload.sdkInitConnection():
    print("Failed to initialize connection")
    sys.exit(1)

# Check payload connectivity
if not payload.checkPayloadConnection():
    print("Failed to connect to payload")
    sys.exit(1)

print("✅ Successfully connected to payload!")

# Your application logic here...

# Clean shutdown
payload.sdkQuit()
```

## Example Applications

### Camera Control
```bash
# Basic camera settings
python3 examples/camera_change_settings.py

# Image capture
python3 examples/camera_eo_capture_image.py

# Video recording
python3 examples/camera_eo_record_video.py

# Zoom control
python3 examples/camera_do_setzoom_individual.py

# Time-lapse photography
python3 examples/camera_time_lapse_photography.py

# IR camera controls
python3 examples/camera_ir_set_palette.py
python3 examples/camera_ir_capture_image.py
```

### Gimbal Operations
```bash
# Speed-based movement
python3 examples/gimbal_move_speed.py

# Angle-based positioning
python3 examples/gimbal_move_angle.py

# Gimbal calibration
python3 examples/gimbal_do_calib.py

# Settings management
python3 examples/gimbal_change_settings.py
```

### Advanced Features
```bash
# Object detection
python3 examples/payload_do_object_detection.py

# Object tracking
python3 examples/payload_do_object_tracking.py

# GPS data transmission
python3 examples/payload_set_gps.py

# Media file management
python3 examples/payload_download_media_files.py

# System time synchronization
python3 examples/payload_set_system_time.py
```

## Key Features

### 🔧 **Centralized Configuration**
- Single configuration file for all settings
- Automatic environment setup
- Parameter validation and error checking
- Easy IP/port changes without code modification

### 🔗 **Robust Connection Management**
- Automatic connection detection and validation
- Support for both UDP and UART connections
- Connection timeout and retry mechanisms
- Graceful error handling and recovery

### 📷 **Comprehensive Camera Control**
- Image capture and video recording
- Zoom and focus control (EO and IR cameras)
- Camera mode switching and settings management
- Storage management and media download

### 🎯 **Advanced Gimbal Control**
- Precise angle and speed-based movement
- Multiple control modes (lock, follow, mapping)
- Automatic calibration procedures
- Real-time attitude feedback

### 🤖 **AI-Powered Features**
- Object detection and tracking
- Smart tracking with bounding box control
- Real-time tracking status feedback

### 📡 **Data Integration**
- GPS position transmission
- System time synchronization
- Custom parameter streaming
- Real-time telemetry data

## Troubleshooting

### Connection Issues

**UDP Connection Problems:**
```bash
# Check network connectivity
ping 192.168.55.1

# Verify firewall settings
sudo ufw allow 14566/udp

# Test with different IP if needed
# Update libs/config.py -> ConnectionConfig.UDP_IP_TARGET
```

**UART Connection Problems:**
```bash
# Check port availability
ls /dev/tty*

# Fix permissions (Linux)
sudo chmod 666 /dev/ttyUSB0
# or add user to dialout group
sudo usermod -a -G dialout $USER

# Verify baudrate compatibility
# Update libs/config.py -> ConnectionConfig.UART_BAUDRATE
```

### Common Error Solutions

| Error | Solution |
|-------|----------|
| `ModuleNotFoundError: No module named 'config'` | Ensure you're running from correct directory and libs path is set |
| `AttributeError: 'NoneType' object has no attribute 'recv_match'` | Connection failed - check network/serial connection |
| `No payload detected after X seconds` | Verify payload is powered on and network is accessible |
| `Permission denied: '/dev/ttyUSB0'` | Add user to dialout group or use sudo |

### Debug Mode

Enable detailed logging by modifying `libs/config.py`:
```python
class DebugConfig:
    ENABLE_DEBUG = True
    ENABLE_INFO = True
```

## API Documentation

For detailed API documentation, function parameters, and advanced usage examples, see:
- **[PayloadSDK.md](PayloadSDK.md)** - Complete API reference
- **[examples/](examples/)** - Working code examples
- **[libs/config.py](libs/config.py)** - Configuration options

## Support and Contact

- **Technical Documentation:** See PayloadSDK.md for detailed API reference
- **Example Code:** Check the examples/ directory for implementation patterns
- **Configuration Help:** Review libs/config.py for all available settings
- **Issue Reporting:** Contact Gremsy technical support team

## Version Information

- **SDK Version:** 3.0.0_build.27052025
- **MAVLink Protocol:** 2.0 (with 1.0 compatibility)
- **Supported Software:** v2.0.0 and higher

---

**© 2025 Gremsy. All rights reserved.**