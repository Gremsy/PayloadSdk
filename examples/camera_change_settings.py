#!/usr/bin/env python3
import sys
import os

# Add the libs directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'libs'))

# Import config first to setup environment automatically
from config import config

import time
import signal
from pymavlink import mavutil
from payload_sdk import PayloadSdkInterface, payload_status_event_t, PAYLOAD_TYPE
from payload_define import *

my_payload = None

# Signal handler for quitting
def quit_handler(sig, frame):
    global my_payload
    print("\nTERMINATING AT USER REQUEST")

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

    # End program    
    sys.exit(0)

# Callback function for payload param changes
def onPayloadParamChanged(event, param_char, param):
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_PARAMS:
        # param[0]: param_index
		# param[1]: value
        print(f" --> Param_id: {param_char}, value: {param[1]:.2f}")

def main():
    global my_payload

    print("Starting SetPayloadSettings example...\n")
    signal.signal(signal.SIGINT, quit_handler)
    
    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    if not my_payload.sdkInitConnection():
        print("Failed to initialize connection. Exiting...")
        sys.exit(1)
        
    print("Waiting for payload signal!")
    
    # Register callback function
    my_payload.regPayloadParamChanged(onPayloadParamChanged)
    
    # Check connection
    if not my_payload.checkPayloadConnection():
        print("Failed to connect to payload. Exiting...")
        sys.exit(1)
    
    # Change setting of RC_MODE to STANDARD
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, payload_camera_rc_mode.PAYLOAD_CAMERA_RC_MODE_STANDARD, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) 
    
    # Change setting of OSD_MODE to STATUS to enable viewing of the zoom factor
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_OSD_MODE , payload_camera_osd_mode.PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)  
    
    print("------------------------> Init values \n")
    # Request to read all settings of the payload and then check the RC_MODE setting
    my_payload.getPayloadCameraSettingList()
    time.sleep(3)  

    print("\nChange some params\n")
    # Change zoom mode to SuperResolution
    if PAYLOAD_TYPE in ["VIO", "ZIO"]:
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, payload_camera_video_zoom_mode.PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) 
        # time.sleep(3)  

    # Request to read all settings of the payload to verify the changes
    print("------------------------> Changed values \n")
    my_payload.getPayloadCameraSettingList()
    time.sleep(3)  

    # Set ICR to AUTO mode
    print("------------------------> Setting ICR mode to AUTO")
    my_payload.setPayloadCameraParam(
        PAYLOAD_CAMERA_VIDEO_ICR_MODE,
        payload_camera_video_icr_mode.PAYLOAD_CAMERA_VIDEO_ICR_MODE_AUTO,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32
    )
    time.sleep(2)

    # Switch to MANUAL ICR mode
    print("------------------------> Setting ICR mode to MANUAL")
    my_payload.setPayloadCameraParam(
        PAYLOAD_CAMERA_VIDEO_ICR_MODE,
        payload_camera_video_icr_mode.PAYLOAD_CAMERA_VIDEO_ICR_MODE_MANUAL,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32
    )
    time.sleep(2)

    # Enable manual ICR control
    print("------------------------> Enabling manual ICR")
    my_payload.setPayloadCameraParam(
        PAYLOAD_CAMERA_VIDEO_ICR_MANUAL,
        payload_camera_video_icr_manual.PAYLOAD_CAMERA_VIDEO_ICR_MANUAL_ON,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32
    )
    time.sleep(2)

    # Set ICR threshold
    threshold = 128  # Value between 0 and 255
    print(f"------------------------> Setting ICR threshold to {threshold}")
    my_payload.setPayloadCameraParam(
        PAYLOAD_CAMERA_VIDEO_ICR_THRESHOLD,
        threshold,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32
    )
    time.sleep(2)

    # Disable manual ICR control
    print("------------------------> Disabling manual ICR")
    my_payload.setPayloadCameraParam(
        PAYLOAD_CAMERA_VIDEO_ICR_MANUAL,
        payload_camera_video_icr_manual.PAYLOAD_CAMERA_VIDEO_ICR_MANUAL_OFF,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32
    )
    time.sleep(2)

    print("------------------------> ICR example finished!")

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

if __name__ == "__main__":
    main()