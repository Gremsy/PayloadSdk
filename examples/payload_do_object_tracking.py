'''
 * This example wil show you how to execute the object tracking feature on the Payload
 * Only run this example when connect with the Vio payload for now
 * This example will:
 * 1. Change the view mode to EO
 * 2. Change the Tracking Mode to Object Tracking
 * 3. Turn the OSD mode to Status
 * 4. Send some position for the bounding box
 * 5. Trigger the Object tracking feature Start/Stop
'''

#!/usr/bin/env python3
import sys
import os

# Add the libs directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'libs'))

# Import config first to setup environment automatically
from config import config

import time
import signal
import sys
import threading
import random
from pymavlink import mavutil
from payload_sdk import PayloadSdkInterface, payload_status_event_t, payload_param_t, tracking_status_t, tracking_mode_t, PAYLOAD_TYPE
from payload_define import *

# Global variables
my_payload = None
time_to_exit = False

# Tracking parameters
track_pos_x = 0
track_pos_y = 0
track_pos_w = 0
track_pos_h = 0
track_status = 0
 
# Signal handler for quitting
def quit_handler(sig, frame):
    global my_payload, time_to_exit
    print("\nTERMINATING AT USER REQUEST\n")
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
    global track_pos_x, track_pos_y, track_pos_w, track_pos_h, track_status

    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_PARAMS:
        # param[0]: param index
        # param[1]: value
        if payload_param_t(param[0]) == payload_param_t.PARAM_TRACK_POS_X:
            track_pos_x = param[1]

        elif payload_param_t(param[0]) == payload_param_t.PARAM_TRACK_POS_Y:
            track_pos_y = param[1]

        elif payload_param_t(param[0]) == payload_param_t.PARAM_TRACK_POS_W:
            track_pos_w = param[1]

        elif payload_param_t(param[0]) == payload_param_t.PARAM_TRACK_POS_H:
            track_pos_h = param[1]

        elif payload_param_t(param[0]) == payload_param_t.PARAM_TRACK_STATUS:
            track_status = float(int(param[1]) & 0xff)

        else:
            return

        print(f"onPayloadStatusChanged, status: {track_status:.2f}, x: {track_pos_x:.2f}, y: {track_pos_y:.2f}, w: {track_pos_w:.2f}, h: {track_pos_h:.2f}")

# Handle tracking 
def handle_tracking():
    global time_to_exit, track_status, my_payload

    while not time_to_exit:
        # Random tracking bounding box
        random_x = random.randint(0, 1920 - 20)
        random_y = random.randint(0, 1080 - 20)

        print("Active the tracker")
        my_payload.setPayloadObjectTrackingMode(tracking_mode_t.TRACK_ACTIVE)

        print("Start tracking new object")
        # if you send the postion while the tracker is not actived, the payload will move by the EagleEyes feature (only move, without tracking)
		# Zio Payload only use random_x and random_y
        my_payload.setPayloadObjectTrackingPosition(random_x, random_y, 128, 128)

        # if you want to track the object at the center of the screen, just use
		# my_payload.setPayloadObjectTrackingPosition()

        if PAYLOAD_TYPE == "ZIO":
            time.sleep(2)
        else:
            time.sleep(0.2)

        # Check tracking status
        if track_status == tracking_status_t.TRACK_TRACKED:
            print("Object was tracked. Keep this object for 5 seconds...")
            time.sleep(5)  

            # Release object
            my_payload.setPayloadObjectTrackingMode(tracking_mode_t.TRACK_STOP)
            print("Object was released. Wait 3 seconds...")
            time.sleep(3)

        elif track_status == tracking_status_t.TRACK_IDLE:
            print("Tracker in IDLE mode.")
            time.sleep(1)

        elif track_status == tracking_status_t.TRACK_LOST:
            print("Lost object. Release the tracker then Try catch another object")
            my_payload.setPayloadObjectTrackingMode(tracking_mode_t.TRACK_STOP)
            time.sleep(1) 

def all_threads_init():
    tracking_thread = threading.Thread(target=handle_tracking)
    tracking_thread.daemon = True 
    tracking_thread.start()
    print("Thread created\n")

def main():
    global my_payload, time_to_exit

    print("Starting Do Object Tracking example...\n")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!\n")

    # Register callback function
    my_payload.regPayloadStatusChanged(onPayloadStatusChanged)

    # Check payload connection
    my_payload.checkPayloadConnection()

    # Initialize environment for object tracking

    if PAYLOAD_TYPE != "ZIO":
	# Change view mode to EO
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, payload_camera_view_src.PAYLOAD_CAMERA_VIEW_EO, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)

    if PAYLOAD_TYPE == "VIO":
        # Change tracking mode to Object tracking
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE, payload_camera_tracking_mode.PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    elif PAYLOAD_TYPE in ["MB1", "ZIO"]:
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_OBJECT_DETECTION, payload_camera_tracking_mode.PAYLOAD_CAMERA_OBJECT_DETECTION_DISABLE, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)

    # Change OSD mode to Status
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_OSD_MODE, payload_camera_osd_mode.PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)

	# Set status message rate
	# If you do not want to receive the message anymore, need to set rate to 0
    my_payload.setParamRate(payload_param_t.PARAM_TRACK_POS_X, 100)
    my_payload.setParamRate(payload_param_t.PARAM_TRACK_POS_Y, 100)
    my_payload.setParamRate(payload_param_t.PARAM_TRACK_POS_W, 100)
    my_payload.setParamRate(payload_param_t.PARAM_TRACK_POS_H, 100)
    my_payload.setParamRate(payload_param_t.PARAM_TRACK_STATUS, 100)

    # Init threads
    all_threads_init()

    # Initialize random seed
    random.seed(time.time())

    # Keep program running to process tracking
    while not time_to_exit:
        # Short delay to prevent high CPU usage
        time.sleep(0.001)

if __name__ == "__main__":
    main()