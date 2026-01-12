#!/usr/bin/env python3
"""
Payload SDK UI Demo - Python
Based on C++ PayloadSdk UI Demo implementation

Usage:
    python ui_demo.py          # Standard payload mode
    python ui_demo.py --mb1    # MB1 payload mode
"""

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gst', '1.0')
from gi.repository import Gtk, GLib, Gst

import sys
import os
import threading
import time
import argparse

# Add libs path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'libs'))

from main_window import MainWindow
from payload_settings_tab import UICommand

# Import payload SDK
try:
    from payload_sdk import PayloadSdkInterface
    from config import ConnectionConfig
    # Import payload definitions for param IDs
    from payload_define import (
        PAYLOAD_CAMERA_VIEW_SRC,
        PAYLOAD_CAMERA_RECORD_SRC,
        PAYLOAD_CAMERA_VIDEO_OSD_MODE,
        PAYLOAD_CAMERA_GIMBAL_MODE,
        PAYLOAD_CAMERA_VIDEO_FLIP,
        PAYLOAD_CAMERA_IR_PALETTE,
        PAYLOAD_CAMERA_TRACKING_MODE,
    )
    from payload_sdk import input_mode_t
except ImportError as e:
    print(f"Error importing payload SDK: {e}")
    print("Make sure the SDK is properly installed")
    sys.exit(1)

# Import MB1-specific definitions
try:
    from mb1_define import (
        PAYLOAD_CAMERA_SETTING_TARGET,
        PAYLOAD_CAMERA_RC_MODE,
        PAYLOAD_CAMERA_STORAGE_TYPE,
        PAYLOAD_CAMERA_EO_SCENE_MODE,
        PAYLOAD_CAMERA_EO_AE_COMPENSATION,
        PAYLOAD_CAMERA_EO_WHITE_BALANCE,
        PAYLOAD_CAMERA_EO_ISO,
        PAYLOAD_CAMERA_EO_SHARPNESS,
        PAYLOAD_CAMERA_IR_GAIN,
        PAYLOAD_CAMERA_IR_CONTRAST_MODE,
        PAYLOAD_CAMERA_IR_AGC_MODE,
        PAYLOAD_CAMERA_IR_AGC_LINEAR_PERCENT,
        PAYLOAD_CAMERA_IR_SPOTMETER_MODE,
        PAYLOAD_CAMERA_IR_SPOTMETER_UNITS,
        PAYLOAD_CAMERA_IR_SPOTMETER_SIZE,
        PAYLOAD_CAMERA_IR_ISOTHERM_MODE,
        PAYLOAD_CAMERA_IR_ISOTHERM_UNITS,
        PAYLOAD_CAMERA_IR_ISOTHERM_THRESHOLD,
        PAYLOAD_CAMERA_GIMBAL_FW_FLAG,
        PAYLOAD_CAMERA_OBJECT_DETECTION,
        PAYLOAD_CAMERA_IR_ISOTHERMS_GAIN,
    )
    MB1_DEFINITIONS_AVAILABLE = True
except ImportError:
    MB1_DEFINITIONS_AVAILABLE = False

# MAVLink parameter types
PARAM_TYPE_UINT8 = 1
PARAM_TYPE_INT8 = 2
PARAM_TYPE_UINT16 = 3
PARAM_TYPE_INT16 = 4
PARAM_TYPE_UINT32 = 5
PARAM_TYPE_INT32 = 6
PARAM_TYPE_REAL32 = 9


class PayloadUIDemo:
    """Main application class"""

    def __init__(self, is_mb1=False):
        self.sdk = None
        self.window = None
        self.is_connected = False
        self.running = True
        self.is_mb1 = is_mb1

        # Create main window (pass is_mb1 flag)
        self.window = MainWindow(1600, 900, is_mb1=is_mb1)
        self.window.connect("destroy", self._on_window_destroy)

        # Register callbacks
        self.window.reg_ui_command_changed(self._on_ui_command)
        self.window.reg_ui_connect_command_changed(self._on_ui_connect_command)

    def _on_window_destroy(self, widget):
        """Handle window close"""
        self.running = False
        if self.sdk:
            self.sdk.sdkQuit()
        Gtk.main_quit()

    def _on_ui_connect_command(self, command, param):
        """Handle connection commands from UI"""
        if command == "CONNECT_PAYLOAD":
            self._connect_payload(param)
        elif command == "DISCONNECT_PAYLOAD":
            self._disconnect_payload()

    def _connect_payload(self, ip):
        """Connect to payload"""
        print(f"Connecting to payload at {ip}...")

        # Update config with new IP
        ConnectionConfig.UDP_IP_TARGET = ip

        # Create SDK instance
        self.sdk = PayloadSdkInterface()

        # Register callbacks
        self.sdk.regPayloadStatusChanged(self._on_payload_status_changed)
        self.sdk.regPayloadParamChanged(self._on_payload_param_changed)
        self.sdk.regPayloadStreamChanged(self._on_payload_stream_changed)

        # Initialize connection
        if self.sdk.sdkInitConnection():
            # Start connection check thread
            threading.Thread(target=self._check_connection_thread, daemon=True).start()
        else:
            print("Failed to initialize connection")
            GLib.idle_add(self._update_ui_disconnected)

    def _check_connection_thread(self):
        """Thread to check payload connection"""
        timeout = 5.0
        start_time = time.time()

        while self.running and (time.time() - start_time) < timeout:
            if self.sdk and self.sdk.checkPayloadConnection():
                self.is_connected = True
                GLib.idle_add(self._update_ui_connected)
                print("Payload connected!")

                # Query payload parameters (like C++ query_payload_param)
                self._query_payload_params()

                # Start status update thread
                threading.Thread(target=self._status_update_thread, daemon=True).start()
                return
            time.sleep(0.1)

        print("Connection timeout")
        GLib.idle_add(self._update_ui_disconnected)

    def _query_payload_params(self):
        """Query payload parameters after connection (like C++ QUERY_PAYLOAD_PARAM)"""
        if not self.sdk:
            return

        try:
            # Get camera information first
            print("Querying payload camera information...")
            self.sdk.getPayloadCameraInformation()
            time.sleep(0.1)

            # Get all camera settings
            print("Querying payload camera settings...")
            self.sdk.getPayloadCameraSettingList()
            time.sleep(0.1)

            # Request specific settings that may not be included in the list
            print("Querying specific camera settings...")
            # View mode
            self.sdk.getPayloadCameraSettingByID(PAYLOAD_CAMERA_VIEW_SRC)
            time.sleep(0.05)
            # Record source
            self.sdk.getPayloadCameraSettingByID(PAYLOAD_CAMERA_RECORD_SRC)
            time.sleep(0.05)
            # Gimbal mode
            self.sdk.getPayloadCameraSettingByID(PAYLOAD_CAMERA_GIMBAL_MODE)
            time.sleep(0.05)
            # IR Palette
            self.sdk.getPayloadCameraSettingByID(PAYLOAD_CAMERA_IR_PALETTE)
            time.sleep(0.05)
            # OSD mode
            self.sdk.getPayloadCameraSettingByID(PAYLOAD_CAMERA_VIDEO_OSD_MODE)
            time.sleep(0.05)
            # Video flip
            self.sdk.getPayloadCameraSettingByID(PAYLOAD_CAMERA_VIDEO_FLIP)
            time.sleep(0.05)
            # IR FFC mode (not in payload_define, use string directly)
            self.sdk.getPayloadCameraSettingByID("IR_FFCMODE")

        except Exception as e:
            print(f"Error querying payload params: {e}")

    def _status_update_thread(self):
        """Thread to periodically request status updates"""
        while self.running and self.is_connected:
            if self.sdk:
                # Request various status updates
                self.sdk.getPayloadCaptureStatus()
                self.sdk.getPayloadStorage()
            time.sleep(1.0)

    def _disconnect_payload(self):
        """Disconnect from payload"""
        print("Disconnecting from payload...")
        self.is_connected = False

        if self.sdk:
            self.sdk.sdkQuit()
            self.sdk = None

        GLib.idle_add(self._update_ui_disconnected)

    def _update_ui_connected(self):
        """Update UI for connected state (must be called from main thread)"""
        if self.window:
            self.window.send_connected()
        return False

    def _update_ui_disconnected(self):
        """Update UI for disconnected state (must be called from main thread)"""
        if self.window:
            self.window.send_disconnected()
        return False

    def _on_payload_status_changed(self, event, data):
        """Handle payload status change callback"""
        # Update UI from main thread
        GLib.idle_add(self._update_payload_status, event, data)

    def _update_payload_status(self, event, data):
        """Update payload status in UI (must be called from main thread)"""
        if not self.window:
            return False

        # Convert event enum to string if needed
        event_name = str(event.name) if hasattr(event, 'name') else str(event)

        if "CAPTURE_STATUS" in event_name:
            # SDK returns list: [image_status, video_status, image_count, recording_time_ms]
            if data and isinstance(data, list) and len(data) >= 4:
                self.window.update_capture_info(
                    data[0],  # image_status
                    data[1],  # video_status
                    data[2],  # image_count
                    data[3]   # recording_time_ms
                )
        elif "STORAGE" in event_name:
            # SDK returns list: [total_capacity, used_capacity, available_capacity, status]
            # Capacities are in MB (MiB)
            if data and isinstance(data, list) and len(data) >= 4:
                self.window.update_storage_info(
                    data[3],  # status
                    data[0],  # total_capacity (MB)
                    data[1],  # used_capacity (MB)
                    data[2]   # available_capacity (MB)
                )
        elif "GB_ATTITUDE" in event_name:
            # SDK returns list: [pitch, roll, yaw]
            if data and isinstance(data, list) and len(data) >= 3:
                self.window.update_gimbal_attitude(data[0], data[1], data[2])
        elif "PAYLOAD_PARAMS" in event_name or event_name == "PAYLOAD_PARAMS":
            # SDK returns list: [param_index, value]
            # This is for numeric param updates via PARAM_VALUE or DEBUG messages
            if data and isinstance(data, list) and len(data) >= 2:
                self.window.update_payload_status(data)
        elif "CAM_INFO" in event_name:
            # Camera info received - request streaming info if available
            if data and isinstance(data, list) and len(data) >= 1:
                flags = int(data[0])
                # CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM = 0x02
                if flags & 0x02:
                    print("   ---> Got payload has streaming video, Check streaming URI")
                    if self.sdk:
                        self.sdk.getPayloadCameraStreamingInformation()
                else:
                    print("   ---> Payload has no streaming video")

        return False

    def _on_payload_param_changed(self, event, param_id, params):
        """Handle payload parameter change callback

        Args:
            event: Event type (payload_status_event_t)
            param_id: Parameter ID string
            params: List of parameter values [param_index, value]
        """
        GLib.idle_add(self._update_payload_param, event, param_id, params)

    def _update_payload_param(self, event, param_id, params):
        """Update payload parameter in UI (must be called from main thread)"""
        if not self.window:
            return False

        # Convert event enum to string if needed
        event_name = str(event.name) if hasattr(event, 'name') else str(event)

        # Handle gimbal attitude updates
        if "GB_ATTITUDE" in event_name:
            if params and len(params) >= 3:
                self.window.update_gimbal_attitude(params[0], params[1], params[2])
            # Also update gimbal mode from param_id (mode string like "LOCK_MODE", "FOLLOW_MODE", etc.)
            if param_id:
                self.window.update_gimbal_mode_from_string(param_id)
        elif "CAM_PARAMS" in event_name:
            # PAYLOAD_CAM_PARAMS: params[0] = param_index, params[1] = value
            # param_id is the parameter ID string
            if params and len(params) >= 2:
                value = params[1]
                self.window.update_payload_param(param_id, value)
        elif "GB_PARAMS" in event_name:
            # Gimbal params
            if params and len(params) >= 2:
                value = params[1]
                self.window.update_payload_param(param_id, value)

        return False

    def _on_payload_stream_changed(self, event, url, params):
        """Handle streaming URL change callback"""
        GLib.idle_add(self._update_stream_url, url)

    def _update_stream_url(self, url):
        """Update stream URL in UI (must be called from main thread)"""
        if self.window:
            self.window.update_url_streaming(url)
        return False

    def _on_ui_command(self, command, params):
        """Handle UI commands"""
        if not self.sdk or not self.is_connected:
            print(f"Not connected, ignoring command: {command}")
            return

        print(f"UI Command: {command}, params: {params}")

        try:
            # Camera commands
            if command == UICommand.CAM_CAPTURE:
                self.sdk.setPayloadCameraCaptureImage()

            elif command == UICommand.CAM_RECORD:
                status = int(params[0]) if params else 0
                print(f"Record command: current status={status}")
                if status == 0:
                    # Not recording -> Start recording
                    print("Starting video recording...")
                    self.sdk.setPayloadCameraRecordVideoStart()
                else:
                    # Recording -> Stop recording
                    print("Stopping video recording...")
                    self.sdk.setPayloadCameraRecordVideoStop()

            elif command == UICommand.CAM_VIEW_MODE:
                mode = int(params[0]) if params else 0
                # Use correct param ID from payload_define
                self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_SOURCE_RECORD:
                # Record source: values from UI_CAM_RECORD_SRC_LIST in mb1_define.py
                source = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_RECORD_SRC, source, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_ZOOM_CONTINIOUS:
                direction = int(params[0]) if params else 0
                self.sdk.setCameraZoom(1, direction)  # ZOOM_TYPE_CONTINUOUS

            elif command == UICommand.CAM_ZOOM_STEP:
                direction = int(params[0]) if params else 0
                self.sdk.setCameraZoom(0, direction)  # ZOOM_TYPE_STEP

            elif command == UICommand.CAM_ZOOM_RANGE:
                level = params[0] if params else 0
                self.sdk.setCameraZoom(2, level)  # ZOOM_TYPE_RANGE

            elif command == UICommand.CAM_ZOOM_SPEED:
                speed = int(params[0]) if params else 3
                self.sdk.setPayloadCameraParam("C_V_ZOOM_SPEED", speed, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_FOCUS_CONTINIOUS:
                direction = int(params[0]) if params else 0
                self.sdk.setCameraFocus(1, direction)  # FOCUS_TYPE_CONTINUOUS

            elif command == UICommand.CAM_FOCUS_AUTO:
                self.sdk.setCameraFocus(4, 0)  # FOCUS_TYPE_AUTO

            elif command == UICommand.CAM_FOCUS_SPEED:
                speed = int(params[0]) if params else 3
                self.sdk.setPayloadCameraParam("C_V_FOCUS_SPEED", speed, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_AE_MODE:
                mode = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam("C_V_AE", mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_SHUTTER:
                value = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam("C_V_SP", value, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IRIS:
                value = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam("C_V_IrP", value, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_GAIN:
                value = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam("C_V_GAIN_LS", value, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_WHITE_BALANCE:
                mode = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam("C_V_WB", mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_WHITE_BALANCE_TRIGGER:
                self.sdk.setPayloadCameraWBTrigg()

            elif command == UICommand.CAM_IR_PALETTE:
                palette = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, palette, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_FFC_MODE:
                mode = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam("IR_FFCMODE", mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_FFC_TRIGGER:
                self.sdk.setPayloadCameraFFCTrigg()

            elif command == UICommand.CAM_LRF_MODE:
                mode = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam("LRF_MODE", mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_OSD_MODE:
                mode = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_OSD_MODE, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IMAGE_FLIP:
                # Image Flip: values from UI_IMAGE_FLIP_LIST in mb1_define.py
                mode = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_FLIP, mode, PARAM_TYPE_UINT32)

            # Gimbal commands
            elif command == UICommand.GIMBAL_MODE:
                mode = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.GIMBAL_CONTROL_TILT:
                speed = params[0] if params else 0
                self.sdk.setGimbalSpeed(speed, 0, 0, input_mode_t.INPUT_SPEED)  # pitch speed

            elif command == UICommand.GIMBAL_CONTROL_PAN:
                speed = params[0] if params else 0
                self.sdk.setGimbalSpeed(0, 0, speed, input_mode_t.INPUT_SPEED)  # yaw speed

            elif command == UICommand.GIMBAL_CONTROL_ANGLE:
                pitch = params[0] if len(params) > 0 else 0
                roll = params[1] if len(params) > 1 else 0
                yaw = params[2] if len(params) > 2 else 0
                # Use setGimbalSpeed with INPUT_ANGLE mode
                self.sdk.setGimbalSpeed(pitch, roll, yaw, input_mode_t.INPUT_ANGLE)

            # Tracking commands
            elif command == UICommand.PAYLOAD_TOUCH:
                x = params[0] if len(params) > 0 else 960
                y = params[1] if len(params) > 1 else 540
                self.sdk.setPayloadObjectTrackingPosition(int(x), int(y))

            elif command == UICommand.PAYLOAD_TRACK:
                enable = int(params[0]) if params else 0
                self.sdk.setPayloadObjectTrackingMode(enable)

            elif command == UICommand.PAYLOAD_TRACK_MODE:
                mode = int(params[0]) if params else 0
                self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE, mode, PARAM_TYPE_UINT32)

            # MB1-specific commands
            elif command == UICommand.CAM_SETTING_TARGET:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_SETTING_TARGET, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_RC_MODE:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_STORAGE_TYPE:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_STORAGE_TYPE, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_EO_SCENE_MODE:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_EO_SCENE_MODE, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_EO_AE_COMPENSATION:
                if MB1_DEFINITIONS_AVAILABLE:
                    value = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_EO_AE_COMPENSATION, value, PARAM_TYPE_INT32)

            elif command == UICommand.CAM_EO_WHITE_BALANCE:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_EO_WHITE_BALANCE, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_EO_ISO:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_EO_ISO, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_EO_SHARPNESS:
                if MB1_DEFINITIONS_AVAILABLE:
                    value = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_EO_SHARPNESS, value, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_GAIN_MODE:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_GAIN, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_CONTRAST_MODE:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_CONTRAST_MODE, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_AGC_MODE:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_AGC_MODE, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_AGC_LINEAR_PERCENT:
                if MB1_DEFINITIONS_AVAILABLE:
                    value = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_AGC_LINEAR_PERCENT, value, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_SPOTMETER_MODE:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_SPOTMETER_MODE, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_SPOTMETER_UNITS:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_SPOTMETER_UNITS, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_SPOTMETER_SIZE:
                if MB1_DEFINITIONS_AVAILABLE:
                    value = int(params[0]) if params else 16
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_SPOTMETER_SIZE, value, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_ISOTHERM_MODE:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERM_MODE, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_ISOTHERM_UNITS:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERM_UNITS, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_ISOTHERM_THRESHOLD:
                if MB1_DEFINITIONS_AVAILABLE:
                    value = int(params[0]) if params else 50
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERM_THRESHOLD, value, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_GIMBAL_FW_FLAG:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_FW_FLAG, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_OBJECT_DETECTION:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_OBJECT_DETECTION, mode, PARAM_TYPE_UINT32)

            elif command == UICommand.CAM_IR_ISOTHERMS_GAIN:
                if MB1_DEFINITIONS_AVAILABLE:
                    mode = int(params[0]) if params else 0
                    self.sdk.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERMS_GAIN, mode, PARAM_TYPE_UINT32)

            else:
                print(f"Unknown command: {command}")

        except Exception as e:
            print(f"Error executing command {command}: {e}")

    def run(self):
        """Run the application"""
        Gtk.main()


def main():
    """Main entry point"""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Payload SDK UI Demo - Python")
    parser.add_argument('--mb1', action='store_true', help="Enable MB1 payload mode (MB1-specific controls)")
    args = parser.parse_args()

    print("=" * 60)
    print("Payload SDK UI Demo - Python")
    print("Based on C++ PayloadSdk UI Demo")
    if args.mb1:
        print("Mode: MB1 Payload")
        if not MB1_DEFINITIONS_AVAILABLE:
            print("Warning: MB1 definitions not found, some features may not work")
    else:
        print("Mode: Standard Payload")
    print("=" * 60)

    # Create and run application
    app = PayloadUIDemo(is_mb1=args.mb1)
    app.run()


if __name__ == "__main__":
    main()
