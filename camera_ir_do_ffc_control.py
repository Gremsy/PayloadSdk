#!/usr/bin/env python
import os
os.environ['MAVLINK20'] = "1"
os.environ['MAVLINK_DIALECT'] = "ardupilotmega"

import time
import signal
import sys
from libs.payload_sdk import PayloadSdkInterface, ffc_mode_t
from libs.payload_define import *

my_payload = None
time_to_exit = False

# Counter for FFC trigger
ffc_trigger_cnt_max = 5
ffc_trigger_cnt = 0

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

def main():
    global my_payload, ffc_trigger_cnt, time_to_exit

    print("Starting IR FFC Control example...\n")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!\n")

    # Check connection
    my_payload.checkPayloadConnection()

    # Change FFC mode to Auto
    my_payload.setPayloadCameraFFCMode(ffc_mode_t.FFC_MODE_AUTO)
    print("Change FFC to Auto, waiting for 5secs.")
    time.sleep(5)

    # Change FFC mode to Manual
    my_payload.setPayloadCameraFFCMode(ffc_mode_t.FFC_MODE_MANUAL)
    print("Change FFC to Manual, waiting for 5secs.")
    time.sleep(5)  

    # Perform FFC trigger operations in a loop
    while not time_to_exit:
        if ffc_trigger_cnt < ffc_trigger_cnt_max:
            ffc_trigger_cnt += 1
            print(f"Trigger FFC, {ffc_trigger_cnt}")
            my_payload.setPayloadCameraFFCTrigg()
            time.sleep(3)
        else:
            break

    print("Done. Exit...")

    # Close payload interface
    try:
        my_payload.sdkQuit()
        print("Payload connection closed successfully.")
    except Exception as e:
        print(f"Error while quitting payload: {e}")    

if __name__ == "__main__":
    main()