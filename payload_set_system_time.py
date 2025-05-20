#!/usr/bin/env python
import os
os.environ['MAVLINK20'] = "1"
os.environ['MAVLINK_DIALECT'] = "ardupilotmega"

import time
import signal
import sys
from libs.payload_sdk import PayloadSdkInterface, mavlink_system_time_t
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

# Function to get the epoch time
def get_epoch_time_in_microseconds() -> int:
    # Get the current time in seconds since the epoch
    return int(time.time() * 1e6)

def main():
    global my_payload

    print("Starting SendSystemTime example...")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!")

    # Check connection
    my_payload.checkPayloadConnection()

    msg_cnt = 0
    boot_time_ms = 0

    while True:

        sys_time = mavlink_system_time_t()
        sys_time.time_boot_ms = boot_time_ms
        boot_time_ms += 100  # Need to add boot time milliseconds of your system here
        sys_time.time_unix_usec = get_epoch_time_in_microseconds()  # Get the current time in epoch time in microseconds

        # Send system time to payload
        my_payload.sendPayloadSystemTime(sys_time)
        print(f"Send System Time to payload: {msg_cnt}, {sys_time.time_unix_usec}")
        msg_cnt += 1

        time.sleep(1)  # 1000ms ~ 1Hz

if __name__ == "__main__":
    main()