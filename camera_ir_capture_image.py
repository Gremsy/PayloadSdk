#!/usr/bin/env python
import os
os.environ['MAVLINK20'] = "1"
os.environ['MAVLINK_DIALECT'] = "ardupilotmega"

import time
import signal
import sys
from pymavlink import mavutil
from libs.payload_sdk import PayloadSdkInterface, payload_status_event_t, capture_sequence_t
from libs.payload_define import *   

my_payload = None
image_to_capture = 3
time_to_exit = False

my_capture = capture_sequence_t.CHECK_STORAGE

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

# Callback function for payload status changes
def onPayloadStatusChanged(event: int, param: list):
    global my_capture, image_to_capture, time_to_exit

    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_CAPTURE_STATUS:      
        # param[0]: image_status
		# param[1]: video_status
		# param[2]: image_count
		# param[3]: recording_time_ms
        
        if my_capture == capture_sequence_t.CHECK_CAPTURE_STATUS:
            print(f"Got payload capture status: image_status: {param[0]:.2f}, video_status: {param[1]:.2f}")

            # If image status is idle, do capture
            if param[0] == 0:
                my_capture = capture_sequence_t.CHECK_CAMERA_MODE
                print("   ---> Payload is idle, Check camera mode")
            else:
                print("   ---> Payload is busy")
                my_capture = capture_sequence_t.IDLE

        elif my_capture == capture_sequence_t.WAIT_CAPTURE_DONE:
            if param[0] == 0:
                image_to_capture -= 1
                my_capture = capture_sequence_t.CHECK_STORAGE
                print(f"   ---> Payload is completed capture image, Do next sequence {image_to_capture}")

                if image_to_capture == 0:
                    # Can't call sys.exit(0) in callback function.
                    time_to_exit = True
            else:
                print("   ---> Payload is busy")
    
    elif payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_STORAGE_INFO:
        # param[0]: total_capacity
		# param[1]: used_capacity
		# param[2]: available_capacity
		# param[3]: status
        
        if my_capture == capture_sequence_t.CHECK_STORAGE:
            print(f"Got payload storage info: total: {param[0]:.2f} MB, used: {param[1]:.2f} MB, available: {param[2]:.2f} MB")

            # If payload have enough space, check capture status
            if param[2] >= 10.0:
                my_capture = capture_sequence_t.CHECK_CAPTURE_STATUS
                print("   ---> Storage ready, check capture status")
            else:
                my_capture = capture_sequence_t.IDLE
                print("   ---> Payload's storage is not ready")
    
    elif payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_SETTINGS:
        # param[0]: mode_id
		# param[1]: zoomLevel
		# param[2]: focusLevel
        
        if my_capture == capture_sequence_t.CHECK_CAMERA_MODE:
            print(f"Got camera mode: {param[0]:.2f}")
            if param[0] == 0:  
                my_capture = capture_sequence_t.DO_CAPTURE
                print("   ---> Payload in Image mode, do capture image")
            else:
                my_capture = capture_sequence_t.CHANGE_CAMERA_MODE
                print("   ---> Payload in Video mode, change camera mode")


def main():
    global my_payload, my_capture, time_to_exit

    print("Starting CaptureImage example...\n")
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
    
    # Set payload to IMAGE mode for testing
    my_payload.setPayloadCameraMode(mavutil.mavlink.CAMERA_MODE_IMAGE)  
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_RECORD_SRC, payload_camera_record_src.PAYLOAD_CAMERA_RECORD_IR, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)

    while not time_to_exit:

        # Capture IR image with payload following this sequence
        if my_capture == capture_sequence_t.IDLE:
            # Wait in idle state
            pass  
        elif my_capture == capture_sequence_t.CHECK_STORAGE:
            my_payload.getPayloadStorage()

        elif my_capture == capture_sequence_t.CHECK_CAPTURE_STATUS:
            my_payload.getPayloadCaptureStatus()

        elif my_capture == capture_sequence_t.CHECK_CAMERA_MODE:
            my_payload.getPayloadCameraMode()

        elif my_capture == capture_sequence_t.CHANGE_CAMERA_MODE:
            my_payload.setPayloadCameraMode(mavutil.mavlink.CAMERA_MODE_IMAGE)  
            my_capture = capture_sequence_t.CHECK_CAMERA_MODE

        elif my_capture == capture_sequence_t.DO_CAPTURE:
            my_payload.setPayloadCameraCaptureImage()
            my_capture = capture_sequence_t.WAIT_CAPTURE_DONE   

        elif my_capture == capture_sequence_t.WAIT_CAPTURE_DONE:
            my_payload.getPayloadCaptureStatus()

        time.sleep(0.3)  
        
    # Close payload interface
    try:
        my_payload.sdkQuit()
        print("Payload connection closed successfully.")
    except Exception as e:
        print(f"Error while quitting payload: {e}")    

if __name__ == "__main__":
    main()