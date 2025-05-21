#!/usr/bin/env python
import os
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

import time
import signal
import sys
from pymavlink import mavutil
from libs.payload_sdk import PayloadSdkInterface, PAYLOAD_TYPE
from libs.payload_define import *

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

def main():
    global my_payload

    print("Starting Set gimbal mode example...\n")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!\n")

    # Check payload connection
    my_payload.checkPayloadConnection()

    # Set view source to EO
    print("Set view source to EO!")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, payload_camera_view_src.PAYLOAD_CAMERA_VIEW_EO, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(0.5)  

    # Enable object detection
    print("Enable object detection, delay in 5 secs")
    if PAYLOAD_TYPE in ["VIO", "ZIO"]:
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE, payload_camera_tracking_mode.PAYLOAD_CAMERA_TRACKING_OBJ_DETECTION, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    elif PAYLOAD_TYPE == "GHADRON":
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_OBJECT_DETECTION, payload_camera_object_detection.PAYLOAD_CAMERA_OBJECT_DETECTION_ENABLE, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(5) 

    # Disable object detection
    print("Disable object detection. Exit!")
    if PAYLOAD_TYPE in ["VIO", "ZIO"]:
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE, payload_camera_tracking_mode.PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    elif PAYLOAD_TYPE == "GHADRON":
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_OBJECT_DETECTION, payload_camera_object_detection.PAYLOAD_CAMERA_OBJECT_DETECTION_DISABLE, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(0.5)  

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

if __name__ == "__main__":
    main()