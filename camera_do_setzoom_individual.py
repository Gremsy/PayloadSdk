#!/usr/bin/env python
import os
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

import time
import signal
import sys
from pymavlink import mavutil
from libs.payload_sdk import PayloadSdkInterface, payload_status_event_t, payload_param_t
from libs.payload_define import *

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

# Callback function for payload status changes
def onPayloadStatusChanged(event: int, param: list):
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_ACK:
        print(f" --> Got ack, from command: {param[0]:.0f} - result: {param[1]:.2f}")

    elif payload_status_event_t(event) == payload_status_event_t.PAYLOAD_PARAM_EXT_ACK:
        print(f" --> Got ext_ack, result {param[0]:.2f}")

    elif payload_status_event_t(event) == payload_status_event_t.PAYLOAD_PARAMS:
        # param[0]: param index
		# param[1]: value
        if payload_param_t(param[0]) == payload_param_t.PARAM_EO_ZOOM_LEVEL:  
            print(f"Payload EO_ZOOM_LEVEL: {param[1]:.2f}")

        elif payload_param_t(param[0]) == payload_param_t.PARAM_IR_ZOOM_LEVEL:  
            print(f"Payload IR_ZOOM_LEVEL: {param[1]:.2f}")


def main():
    global my_payload, time_to_exit

    print("Starting ConnectPayload example...\n")
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
    
    # Register to receive zoom status updates
    my_payload.setParamRate(payload_param_t.PARAM_EO_ZOOM_LEVEL, 1000)  
    my_payload.setParamRate(payload_param_t.PARAM_IR_ZOOM_LEVEL, 1000)  
    
    # Set view source to EO/IR
    print("Set view source to EO/IR!")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, payload_camera_view_src.PAYLOAD_CAMERA_VIEW_EOIR, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) 

    # Change EO zoom mode to Super Resolution
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, payload_camera_video_zoom_mode.PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) 

    # Perform zoom operations in a loop
    while not time_to_exit:

        # Zoom EO to 1x
        print("zoom EO to 1x")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, payload_camera_video_zoom_super_resolution_factor.ZOOM_SUPER_RESOLUTION_1X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)  
        time.sleep(3)
        
        # Zoom EO to 4x
        print("zoom EO to 4x")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, payload_camera_video_zoom_super_resolution_factor.ZOOM_SUPER_RESOLUTION_4X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(3) 
        
        # Zoom IR to 1x
        print("zoom IR to 1x")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, payload_camera_ir_zoom_factor.ZOOM_IR_1X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) 
        time.sleep(3)  
        
        # Zoom IR to 4x
        print("zoom IR to 4x")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, payload_camera_ir_zoom_factor.ZOOM_IR_4X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) 
        time.sleep(3)  
        
        # Short delay to prevent high CPU usage
        time.sleep(0.001)

if __name__ == "__main__":
    main()