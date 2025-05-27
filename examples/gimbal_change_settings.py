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
from pymavlink import mavutil
from payload_sdk import PayloadSdkInterface, payload_status_event_t
from payload_define import *

my_payload = None
STIFF_TILT_VALUE = 80.0
step_num = 0

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

# Callback function for payload parameter changes
def onPayloadParamChanged(event: int, param_char: str, param: list):
    global STIFF_TILT_VALUE, step_num

    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_PARAMS:
        # param[0]: param_index
        # param[1]: value
        print(f" --> Payload_param: {param[0]}, value: {param[1]:.2f}")

    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_GB_PARAMS:
        # param[0]: param_index
		# param[1]: value
        print(f"--> Gimbal_param: index: {param[0]}, id: {param_char}, value: {param[1]}")
        if param_char == "STIFF_TILT" and step_num == 0:
            STIFF_TILT_VALUE = param[1]

def main():
    global my_payload, step_num
    
    print("Starting SetPayloadGimbalSettings example...\n")
    print("This sample will:")
    print(" 1. Download the current value of param STIFF_TILT")
    print(" 2. Change the value of param STIFF_TILT to 50")
    print(" 3. Download the value of param STIFF_TILT to verify the changed")
    print(" 4. Change the value for STIFF_TILT back to the original")
    print(" 5. Download the value of param STIFF_TILT to verify the changed\n")

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
    
    while True:
        if step_num == 0:
            # Step 1
            my_payload.getPayloadGimbalSettingByID("STIFF_TILT")
            time.sleep(1)
            step_num = 1

        elif step_num == 1:
            # Step 2
            my_payload.setPayloadGimbalParamByID("STIFF_TILT", 50.0)
            time.sleep(1)
            step_num = 2

        elif step_num == 2:
            # Step 3
            my_payload.getPayloadGimbalSettingByID("STIFF_TILT")
            time.sleep(1)
            step_num = 3

        elif step_num == 3:
            # Step 4
            my_payload.setPayloadGimbalParamByID("STIFF_TILT", STIFF_TILT_VALUE)
            time.sleep(1)
            step_num = 4

        elif step_num == 4:
            # Step 5
            my_payload.getPayloadGimbalSettingByID("STIFF_TILT")
            time.sleep(1)
            step_num = 5

        elif step_num >=5:
            break
    
    print("Done. Exit")
    time.sleep(1)
    
    # Close payload interface
    try:
        my_payload.sdkQuit()
        print("Payload connection closed successfully.")
    except Exception as e:
        print(f"Error while quitting payload: {e}")   

if __name__ == "__main__":
    main()