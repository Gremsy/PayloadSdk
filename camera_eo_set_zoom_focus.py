#!/usr/bin/env python
import os
os.environ['MAVLINK20'] = "1"
os.environ['MAVLINK_DIALECT'] = "ardupilotmega"

import time
import signal
import sys
from pymavlink import mavutil
from libs.payload_sdk import PayloadSdkInterface, payload_status_event_t, PAYLOAD_TYPE
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

# Callback function for payload status changes
def onPayloadStatusChanged(event: int, param: list):
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_ACK:
        print(f" --> Got ack, from command: {param[0]:.0f} - result: {param[1]:.2f}")

def main():
    global my_payload

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

    # Set view source to EO
    print("Set view source to EO!")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, payload_camera_view_src.PAYLOAD_CAMERA_VIEW_EO, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) 
    time.sleep(1) 
    
    print("Set zoom level to 1x!")
    if PAYLOAD_TYPE == "GHADRON":
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_FACTOR, payload_camera_video_zoom_factor.ZOOM_EO_1X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) 
    elif PAYLOAD_TYPE in ["VIO", "ZIO"]:
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, payload_camera_video_zoom_super_resolution_factor.ZOOM_SUPER_RESOLUTION_1X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(3)
    
    # Zoom step
    print("Zoom In 4 times!")
    for _ in range(4):
        my_payload.setCameraZoom(mavutil.mavlink.ZOOM_TYPE_STEP, camera_zoom_value.ZOOM_IN)    # Zoom in 
        time.sleep(1)  
    
    print("Zoom Out 2 times!")
    for _ in range(2):
        my_payload.setCameraZoom(mavutil.mavlink.ZOOM_TYPE_STEP, camera_zoom_value.ZOOM_OUT)   # Zoom out
        time.sleep(1)   

    # Zoom continuous
    print("Start Zoom In!")
    my_payload.setCameraZoom(mavutil.mavlink.ZOOM_TYPE_CONTINUOUS, camera_zoom_value.ZOOM_IN)   # Zoom in 
    time.sleep(5)   
    
    print("Stop Zoom!")
    my_payload.setCameraZoom(mavutil.mavlink.ZOOM_TYPE_CONTINUOUS, camera_zoom_value.ZOOM_STOP)    # Stop zoom  
    time.sleep(2)   
    
    print("Start Zoom Out!")
    my_payload.setCameraZoom(mavutil.mavlink.ZOOM_TYPE_CONTINUOUS, camera_zoom_value.ZOOM_OUT)     # Zoom out
    time.sleep(7)  
    print("Stop Zoom!")
    my_payload.setCameraZoom(mavutil.mavlink.ZOOM_TYPE_CONTINUOUS, camera_zoom_value.ZOOM_STOP)    # Stop zoom 
    time.sleep(2)   

    # Zoom range
    print("Zoom Range 50%!")
    my_payload.setCameraZoom(mavutil.mavlink.ZOOM_TYPE_RANGE, 50.0)  # Zoom 50%
    time.sleep(3)  
    
    print("Zoom Range 70%!")
    my_payload.setCameraZoom(mavutil.mavlink.ZOOM_TYPE_RANGE, 70.0)  # Zoom 70%
    time.sleep(3)    
    
    print("Zoom Range 100%!")
    my_payload.setCameraZoom(mavutil.mavlink.ZOOM_TYPE_RANGE, 100.0) # Zoom 100%
    time.sleep(3)   
    
    print("Zoom Range 0%!")
    my_payload.setCameraZoom(mavutil.mavlink.ZOOM_TYPE_RANGE, 0.0)   # Zoom 0%
    time.sleep(5)  
    
    if PAYLOAD_TYPE in ["VIO", "ZIO"]:
        # Focus continuous
        print("Start Focus In!")
        my_payload.setCameraFocus(mavutil.mavlink.ZOOM_TYPE_CONTINUOUS, camera_focus_value.FOCUS_IN)   # Focus in
        time.sleep(4)  
        print("Stop Focus!")
        my_payload.setCameraFocus(mavutil.mavlink.ZOOM_TYPE_CONTINUOUS, camera_focus_value.FOCUS_STOP)  # Stop focus
        time.sleep(2)   
        
        print("Start Focus Out!")
        my_payload.setCameraFocus(mavutil.mavlink.ZOOM_TYPE_CONTINUOUS, camera_focus_value.FOCUS_OUT)   # Focus out
        time.sleep(4)   
        print("Stop Focus!")
        my_payload.setCameraFocus(mavutil.mavlink.ZOOM_TYPE_CONTINUOUS, camera_focus_value.FOCUS_STOP)  # Stop focus
        time.sleep(2)   
        
        # Auto focus
        print("Auto Focus!")
        my_payload.setCameraFocus(mavutil.mavlink.FOCUS_TYPE_AUTO)
    
    print("!--------------------!\n")
    
    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

if __name__ == "__main__":
    main()