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
from payload_sdk import PayloadSdkInterface, payload_status_event_t, payload_param_t
from payload_define import *

my_payload = None
time_to_exit = False

# Handle SIGINT so the example can close cleanly
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
def onPayloadStatusChanged(event, param):
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_ACK:
        print(f" --> Got ack, from command: {param[0]:.0f} - result: {param[1]:.2f}")

    elif payload_status_event_t(event) == payload_status_event_t.PAYLOAD_PARAM_EXT_ACK:
        print(f" --> Got ext_ack, result {param[0]:.2f}")

    elif payload_status_event_t(event) == payload_status_event_t.PAYLOAD_PARAMS:
        # param[0]: payload_param_t enum value
        # param[1]: parameter value
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

    # Perform zoom operations in a loop
    while not time_to_exit:

        # Change EO zoom mode to Super Resolution
        print("\n[EO] Switching zoom mode to Super Resolution...\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, payload_camera_video_zoom_mode.PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(1)

        # Set view source to EO/IR
        print("[EO] Forcing view source to EO over IR composite...\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, payload_camera_view_src.PAYLOAD_CAMERA_VIEW_EOIR, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(1)  

        # Zoom EO to 1x
        print("[EO] Setting Super Resolution zoom to 1x.\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, payload_camera_video_zoom_super_resolution_factor.ZOOM_SUPER_RESOLUTION_1X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(4)

        # Zoom EO to 4x
        print("[EO] Stepping Super Resolution zoom up to 4x.\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, payload_camera_video_zoom_super_resolution_factor.ZOOM_SUPER_RESOLUTION_4X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(4)

        # Zoom EO to 30x
        print("[EO] Driving Super Resolution zoom to 30x.\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, payload_camera_video_zoom_super_resolution_factor.ZOOM_SUPER_RESOLUTION_30X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(4)

        # Change EO zoom mode to Combine mode
        print("[EO] Switching to Combine zoom mode for extended range...\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, payload_camera_video_zoom_mode.PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_COMBINE, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)

        # Zoom EO to 1x
        print("[EO] Combine zoom: resetting to 1x.\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_FACTOR, payload_camera_video_zoom_combine_factor.ZOOM_COMBINE_1X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(4)

        # Zoom EO to 40x
        print("[EO] Combine zoom: jumping to 40x.\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_FACTOR, payload_camera_video_zoom_combine_factor.ZOOM_COMBINE_40X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(6)

        # Zoom EO to 240x
        print("[EO] Combine zoom: pushing to 240x.\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_FACTOR, payload_camera_video_zoom_combine_factor.ZOOM_COMBINE_240X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(5)

        # Set view source to IR/EO
        print("[IR] Switching view source to IR over EO composite...\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, payload_camera_view_src.PAYLOAD_CAMERA_VIEW_IREO, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)

        # Zoom IR to 1x
        print("[IR] Setting zoom to 1x.\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, payload_camera_ir_zoom_factor.ZOOM_IR_1X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(4)

        # Zoom IR to 4x
        print("[IR] Stepping zoom to 4x.\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, payload_camera_ir_zoom_factor.ZOOM_IR_4X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(4)

        # Zoom IR to 8x
        print("[IR] Increasing zoom to 8x for maximum magnification.\n")
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, payload_camera_ir_zoom_factor.ZOOM_IR_8X, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        time.sleep(4)

        # Short delay to prevent high CPU usage
        time.sleep(0.001)

if __name__ == "__main__":
    main()
