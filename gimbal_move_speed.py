#!/usr/bin/env python
import os
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

import time
import signal
import sys
from pymavlink import mavutil
from libs.payload_sdk import PayloadSdkInterface, payload_status_event_t, input_mode_t
from libs.payload_define import *

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
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_GB_ATTITUDE:
        # param[0]: pitch
        # param[1]: roll
        # param[2]: yaw
        print(f"Pitch: {param[0]:.2f} - Roll: {param[1]:.2f} - Yaw: {param[2]:.2f}")

def main():
    global my_payload

    print("Starting Set gimbal mode example...\n")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!\n")

    # Register callback function
    my_payload.regPayloadStatusChanged(onPayloadStatusChanged)

    # Check connection
    my_payload.checkPayloadConnection()
    time.sleep(0.1)  

    # Set gimbal RC mode to STANDARD 
    print("Set gimbal RC mode")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, payload_camera_rc_mode.PAYLOAD_CAMERA_RC_MODE_STANDARD, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(0.1)  

    # Move gimbal yaw to the right 20 deg/s
    print("Move gimbal yaw to the right 20 deg/s, delay in 5secs")
    my_payload.setGimbalSpeed(0, 0, 20, input_mode_t.INPUT_SPEED)
    time.sleep(5) 

    # Move gimbal yaw to the left 20 deg/s
    print("Move gimbal yaw to the left 20 deg/s, delay in 5secs")
    my_payload.setGimbalSpeed(0, 0, -20, input_mode_t.INPUT_SPEED)
    time.sleep(5) 

    # Stop gimbal movement
    print("Keep gimbal stop, delay in 5secs")
    my_payload.setGimbalSpeed(0, 0, 0, input_mode_t.INPUT_SPEED)
    time.sleep(0.5) 

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

if __name__ == "__main__":
    main()