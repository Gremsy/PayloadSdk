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
from payload_sdk import PayloadSdkInterface
from payload_define import *

my_payload = None
time_to_exit = False

# Signal handler for quitting
def quit_handler(sig, frame):
    global my_payload, time_to_exit
    print("\nTERMINATING AT USER REQUEST")
    time_to_exit = True

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

    # End program    
    sys.exit(0)

def main():
    global my_payload, time_to_exit

    print("Starting ConnectPayload example...")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!")

    # Check connection
    connection_success = my_payload.checkPayloadConnection()
    
    if connection_success:
        print("✅ Connection test completed successfully!")
        print("Payload is connected and responding.")
        
        # Wait a moment to ensure connection is stable
        time.sleep(1)
        
        # Close payload interface
        try:
            my_payload.sdkQuit()
            print("Connection closed gracefully.")
        except Exception as e:
            print(f"Error while closing connection: {e}")
        
        print("Check connection example finished.")
    else:
        print("❌ Connection test failed!")
        print("Could not establish connection to payload.")
        
        # Close payload interface
        try:
            my_payload.sdkQuit()
        except Exception as e:
            print(f"Error while closing connection: {e}")
        
        sys.exit(1)

if __name__ == "__main__":
    main()