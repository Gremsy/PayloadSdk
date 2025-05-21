#!/usr/bin/env python
import os
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

import time
import signal
import sys
from pymavlink import mavutil
from libs.payload_sdk import PayloadSdkInterface
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

def main():
    global my_payload

    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!\n")

    # Check connection
    my_payload.checkPayloadConnection()

    # Set view source to IR
    print("Set view source to IR!")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, payload_camera_view_src.PAYLOAD_CAMERA_VIEW_IR, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(1) 

    # Enable IR isotherms with high gain
    print("Enable IR Isotherms with high GAIN, sleep 5s ...")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERMS, payload_camera_ir_isotherms.PAYLOAD_CAMERA_IR_ISOTHERMS_ENABLE, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(0.1) 
    
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERMS_GAIN, payload_camera_ir_isotherms_gain.PAYLOAD_CAMERA_IR_ISOTHERMS_HIGH_GAIN, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(5)  

    # Switch to low gain
    print("Switch low GAIN, sleep 5s ...")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERMS_GAIN, payload_camera_ir_isotherms_gain.PAYLOAD_CAMERA_IR_ISOTHERMS_LOW_GAIN, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(5) 

    # Disable IR Isotherms
    print("Disable IR Isotherms.")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERMS, payload_camera_ir_isotherms.PAYLOAD_CAMERA_IR_ISOTHERMS_DISABLE, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(0.1) 

    # Close payload interface
    try:
        my_payload.sdkQuit()
        print("Payload connection closed successfully.")
    except Exception as e:
        print(f"Error while quitting payload: {e}") 

if __name__ == "__main__":
    main()