import time
import signal
import sys
import os

from libs.payload_sdk import PayloadSdkInterface, mavlink_global_position_int_t
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

    print("Starting SendGPS example...")
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

        # Create GPS data
        gps = mavlink_global_position_int_t()
        gps.time_boot_ms = boot_time_ms
        boot_time_ms += 100               # Need to add boot_time_ms of your system here
        gps.lat = int(40.730610 * 1e7)    # The location of New York City
        gps.lon = int(-73.935242 * 1e7)   
        gps.alt = int(50 * 1e3)           
        gps.relative_alt = 0              
        gps.vx = 0                        # Unused in this example
        gps.vy = 0                        # Unused in this example
        gps.vz = 0                        # Unused in this example
        gps.hdg = 90                      # GPS heading

        # Send GPS data
        my_payload.sendPayloadGPSPosition(gps)
        
        print(f"Send GPS data to payload: {msg_cnt}")
        msg_cnt += 1

        time.sleep(0.1)

if __name__ == "__main__":
    main()