import time
import signal
import sys
from pymavlink import mavutil
from libs.payload_sdk import PayloadSdkInterface, payload_status_event_t
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

# Callback function for payload param changes   
def onPayloadParamChanged(event: int, param_char: str, param: list):
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_PARAMS:
        # param[0]: param_index
		# param[1]: value
        print(f" --> Payload_param: {param_char}, value: {param[1]:.2f}")

    elif payload_status_event_t(event) == payload_status_event_t.PAYLOAD_GB_PARAMS:
        # param[0]: param_index
		# param[1]: value
        print(f"--> Gimbal_param: index: {param[0]:.0f}, id: {param_char}, value: {param[1]:.0f}")

# Callback function for payload status changes
def onPayloadStatusChanged(event: int, param: list):
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_PARAM_EXT_ACK:
        print(f" --> Got ack, result {param[0]:.2f}")

    elif payload_status_event_t(event) == payload_status_event_t.PAYLOAD_PARAMS:
        # param[0]: param_index
		# param[1]: value
        if param[0] == 0:  
            print(f"Payload EO_ZOOM_LEVEL: {param[1]:.2f}")

def main():
    global my_payload

    print("Starting LoadPayloadGimbalSettings example...\n")
    signal.signal(signal.SIGINT, quit_handler)
    
    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!\n")

    # Register callback functions
    my_payload.regPayloadParamChanged(onPayloadParamChanged)
    my_payload.regPayloadStatusChanged(onPayloadStatusChanged)

    # Check connection
    my_payload.checkPayloadConnection()

    # Request to read all settings from the gimbal
    my_payload.getPayloadGimbalSettingList()
    
    # Request to read a specific param by index
    # my_payload.getPayloadGimbalSettingByIndex(0)

    # Request to read a specific param by ID
    # my_payload.getPayloadGimbalSettingByID("VERSION_X")

    while True:
        time.sleep(10)
        break

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

if __name__ == "__main__":
    main()