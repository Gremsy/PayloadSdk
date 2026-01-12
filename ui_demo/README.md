# Payload SDK UI Demo - Python

Graphical User Interface for Gremsy Payload SDK, ported from C++ GTK implementation.

## System Requirements

### System Dependencies (Ubuntu/Debian)

```bash
# GTK+ 3.0 and GStreamer
sudo apt-get install python3-gi python3-gi-cairo gir1.2-gtk-3.0
sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
sudo apt-get install gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
sudo apt-get install gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0
```

### Python Dependencies

```bash
pip install PyGObject
```

## Usage

### Standard Payload Mode

```bash
cd payloadsdk_python/ui_demo
python ui_demo.py
```

### MB1 Payload Mode

```bash
cd payloadsdk_python/ui_demo
python ui_demo.py --mb1
```

## Features

### Connection
- Payload IP address configuration
- Connect/Disconnect button
- Connection status display

### Video Streaming
- RTSP video playback using GStreamer
- Play/Stop/Fullscreen controls
- Touch-to-track on video area

### Payload Settings
- **Camera View & Record**: View mode (EO/IR/EO+IR/IR+EO), Record source
- **Capture/Record**: Capture button, Record button, SD card status

### Camera Settings
- **Zoom Controls**: Continuous zoom, Step zoom, Range zoom, Speed slider
- **Focus Controls**: Continuous focus, Auto focus, Speed slider
- **Exposure**: AE mode, Shutter, Iris, Gain
- **White Balance**: Mode selection, WB trigger
- **IR Camera**: Palette selection, FFC mode, FFC trigger
- **LRF**: Frequency mode
- **OSD**: Disable/Debug/Status
- **Image Flip**: Flip image on/off

### Gimbal Settings
- **Gimbal Mode**: Off/Lock/Follow/Mapping
- **Speed Control**: Speed slider, Direction buttons (Up/Down/Left/Right/Home)
- **Angle Control**: Pitch/Roll/Yaw sliders

### Payload Info Display
- Gimbal Mode, Pitch, Roll, Yaw
- View Mode, Record Source
- EO/IR Zoom Levels
- IR Type, Palette, FFC Mode, Temperatures
- LRF Offset X/Y, Range
- Target GPS Coordinates
- Payload GPS Coordinates

### MB1 Additional Features (when running with --mb1)
- **Setting Target**: Select target device (EO Camera/IR Camera/Gimbal)
- **RC Mode**: Select RC mode (Gremsy/Standard)
- **Storage Type**: Select storage (Internal/SD Card)
- **EO Advanced**: Scene Mode, AE Compensation, White Balance, ISO, Sharpness
- **IR Advanced**: Gain Mode, Contrast Mode, AGC Mode, AGC Linear Percent
- **IR SpotMeter**: Mode, Units, Size
- **IR Isotherm**: Mode, Units, Threshold
- **Object Detection**: Enable/Disable
- **Gimbal Forward Flag**: Overwrite/Forward

## File Structure

```
ui_demo/
├── README.md                  # This file
├── ui_demo.py                 # Main entry point, handles connection and callbacks
├── main_window.py             # Main window class
└── payload_settings_tab.py    # Payload settings tab with all controls
```

## Architecture

The UI follows the same callback-based event-driven architecture as the C++ version:

1. **UI to SDK Communication**: UI commands are sent through callbacks
2. **SDK to UI Communication**: Status updates via registered callbacks
3. **Thread Safety**: GLib.idle_add() used to update UI from background threads

### Data Flow

```
+-------------+     Callback      +-------------+     MAVLink      +----------+
|   UI Demo   | <---------------- | Payload SDK | <-------------> | Payload  |
|  (GTK UI)   | ---------------> |  (Python)   |                 | (Camera) |
+-------------+   UI Commands     +-------------+                 +----------+
```

### Main Callbacks

- `regPayloadStatusChanged`: Receives capture status, storage info, gimbal attitude
- `regPayloadParamChanged`: Receives camera and gimbal parameters
- `regPayloadStreamChanged`: Receives streaming URL

## Comparison with C++ Version

| Feature | C++ Version | Python Version |
|---------|-------------|----------------|
| UI Framework | gtkmm-3.0 | PyGObject (GTK+ 3.0) |
| Video Streaming | GStreamer C API | GStreamer Python bindings |
| Threading | pthread | Python threading |
| Build System | CMake | None (interpreted) |
| Dependency | Compile libraries | pip install |

## Troubleshooting

### Video not displaying
- Check if RTSP URL is correct
- Verify GStreamer plugins are installed
- Check network connection to payload

### Cannot connect to payload
- Verify IP address is correct
- Check if payload is powered on
- Check firewall is not blocking UDP port

### Not receiving parameters
- Check if payload supports PARAM_EXT
- Verify connection is successful
- Check terminal logs for debugging
