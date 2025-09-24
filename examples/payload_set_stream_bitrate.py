#!/usr/bin/env python3
import sys
import os

# Add the libs directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'libs'))

# Import config first to setup environment automatically
from config import config

import time
import signal
from payload_sdk import PayloadSdkInterface, mavlink_system_time_t
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

    while True:
        bitrate = 1000000 # bit per second
        my_payload.setPayloadStreamBitrate(bitrate)
        print(f"The stream bitrate was set to", bitrate)
        time.sleep(1)  # 1000ms ~ 1Hz

        sys.exit(0)

if __name__ == "__main__":
    main()