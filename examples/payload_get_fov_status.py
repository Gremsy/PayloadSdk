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
from payload_sdk import PayloadSdkInterface, payload_status_event_t, camera_type_t
from payload_define import *

my_payload = None

# Signal handler for quitting
def quit_handler(sig, frame):
    global my_payload
    print("\nTERMINATING AT USER REQUEST")

    # Close payload interface
    if my_payload:
        try:
            my_payload.sdkQuit()
        except Exception as e:
            print(f"Error while quitting payload: {e}")
    # End program 
    sys.exit(0)

# Callback function for payload status changes
def onPayloadStatusChanged(event: int, param: list):
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_PARAM_CAM_FOV_STATUS:
        # param[0]: camera id
        # param[1]: hfov
        # param[2]: vfov
        print(f"HFOV: {param[1]:.2f} - VFOV: {param[2]:.2f}")

def main():
    global my_payload

    print("Starting Set gimbal mode example...")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Register callback function
    my_payload.regPayloadStatusChanged(onPayloadStatusChanged)

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!")

    # Check connection
    my_payload.checkPayloadConnection()
    
    while True:
        my_payload.getPayloadCameraFOVStatus(camera_type_t.CAMERA_EO)
        time.sleep(0.5)  # 500ms, 2Hz

        my_payload.getPayloadCameraFOVStatus(camera_type_t.CAMERA_IR)
        time.sleep(0.5)  # 500ms, 2Hz

if __name__ == "__main__":
    main()