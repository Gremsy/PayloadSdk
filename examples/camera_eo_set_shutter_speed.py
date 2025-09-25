#!/usr/bin/env python3
import sys
import os

# Add the libs directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'libs'))

# Import config first to setup environment automatically
from config import config

import time
import signal
import sys
from payload_sdk import PayloadSdkInterface, payload_status_event_t, payload_param_t, PAYLOAD_TYPE
from payload_define import *
from pymavlink import mavutil

my_payload = None
time_to_exit = False

# Signal handler for quitting
def quit_handler(sig, frame):
    global my_payload, time_to_exit
    print("\nTERMINATING AT USER REQUEST")
    time_to_exit = True

    # Close payload interface
    if my_payload:
        try:
            my_payload.sdkQuit()
        except Exception as e:
            print(f"Error while quitting payload: {e}")
    # End program 
    sys.exit(0)

# Shutter value to label conversion
def shutter_value_to_label(value):
    shutter_labels = {
        payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_10: "1/10",
        payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_20: "1/20",
        payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_50: "1/50",
        payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_100: "1/100",
        payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_125: "1/125",
        payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_500: "1/500",
        payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_725: "1/725",
        payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1000: "1/1000",
        payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1500: "1/1500",
        payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_2000: "1/2000"
    }
    return shutter_labels.get(value, "unknown")

# Callback function for payload status changes
def onPayloadParamChanged(event: int, param_char:str ,param: list):
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_PARAMS:
        # Only log shutter speed updates
        if param_char == PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED:
            code = int(param[1])
            print(f"Shutter (C_V_SP): {shutter_value_to_label(code)} (code {code})")

def main():
    global my_payload, time_to_exit

    print("Starting Set gimbal mode example...")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!")

    # Register callback function
    my_payload.regPayloadParamChanged(onPayloadParamChanged)

    # Check connection
    my_payload.checkPayloadConnection()

    # Set view source to EO
    print("Set view source to EO!")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, payload_camera_view_src.PAYLOAD_CAMERA_VIEW_EO, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(1) 

    # Only handle for VIO/ZIO where shutter param is defined
    if PAYLOAD_TYPE in ["VIO", "ZIO"]:
 
        # Read current shutter speed only (by ID)
        my_payload.getPayloadCameraSettingByID(PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED)
        time.sleep(1)

        # Ensure exposure mode allows shutter control
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE, payload_camera_video_auto_exposure.PAYLOAD_CAMERA_VIDEO_EXPOSURE_SHUTTER, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(1)

        while not time_to_exit:
            # Set shutter speed to 1/1000 (change constant as needed)
            my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED, payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_10, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
            time.sleep(1)

            # Read back shutter speed only (by ID)
            my_payload.getPayloadCameraSettingByID(PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED)
            time.sleep(1)

            # Set shutter speed to 1/1000 (change constant as needed)
            my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED, payload_camera_video_shutter_speed.PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1000, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
            time.sleep(1)

            # Read back shutter speed only (by ID)
            my_payload.getPayloadCameraSettingByID(PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED)
            time.sleep(1)

    else:
        sys.exit(-1)        

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

if __name__ == "__main__":
    main()