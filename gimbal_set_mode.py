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

    print("Starting Set gimbal mode example...\n")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!\n")

    # Check connection
    my_payload.checkPayloadConnection()

    # Set gimbal to LOCK mode
    print("Gimbal set LOCK mode, delay in 5 secs")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, payload_camera_gimbal_mode.PAYLOAD_CAMERA_GIMBAL_MODE_LOCK, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(5) 

    # Set gimbal to FOLLOW mode
    print("Gimbal set FOLLOW mode, delay in 5 secs")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, payload_camera_gimbal_mode.PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(5)

    # Set gimbal to MAPPING mode
    print("Gimbal set MAPPING mode, delay in 5 secs")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, payload_camera_gimbal_mode.PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(5)

    # Set gimbal to OFF mode
    print("Gimbal set OFF mode, delay in 5 secs")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, payload_camera_gimbal_mode.PAYLOAD_CAMERA_GIMBAL_MODE_OFF, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(5)

    # Set gimbal to RESET mode
    print("Gimbal set RESET mode, delay in 5 secs")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, payload_camera_gimbal_mode.PAYLOAD_CAMERA_GIMBAL_MODE_RESET, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(5)

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

if __name__ == "__main__":
    main()