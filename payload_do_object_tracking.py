import time
import signal
import sys
import threading
import random
from pymavlink import mavutil
from libs.payload_sdk import PayloadSdkInterface, payload_status_event_t, payload_param_t, tracking_cmd_t, PAYLOAD_TYPE
from libs.payload_define import *

# Global variables
my_payload = None
time_to_exit = False

# Tracking parameters
track_pos_x = track_pos_y = track_pos_w = track_pos_h = track_status = 0.0

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
    global track_pos_x, track_pos_y, track_status, track_pos_w, track_pos_h
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
            track_status = param[1]

        print(f"onPayloadStatusChanged, status: {track_status:.2f}, x: {track_pos_x:.2f}, y: {track_pos_y:.2f}, w: {track_pos_w:.2f}, h: {track_pos_h:.2f}")

# Handle tracking 
def handle_tracking():
    global time_to_exit

    while not time_to_exit:
        # Random tracking bounding box
        random_w = random.randint(20, 1920)     
        random_h = random.randint(20, 1080)    

        print("Start tracking new object")
        my_payload.setPayloadObjectTrackingParams(tracking_cmd_t.TRACK_ACT, random_w, random_h)
        time.sleep(0.2) 

        # Check tracking status
        if track_status:
            print("Object was tracked. Keep this object for 5 seconds...")
            time.sleep(5)  
            # Release object
            my_payload.setPayloadObjectTrackingParams(tracking_cmd_t.TRACK_IDLE, random_w, random_h)
            print("Object was released. Wait 3 seconds...")
            time.sleep(3)

        else:
            print("Lost object. Try catch another object")
            my_payload.setPayloadObjectTrackingParams(tracking_cmd_t.TRACK_IDLE, random_w, random_h)
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
	# Change view mode to EO
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, payload_camera_view_src.PAYLOAD_CAMERA_VIEW_EO, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)

    if PAYLOAD_TYPE in ["VIO", "ZIO"]:
        # Change tracking mode to Object tracking
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE, payload_camera_tracking_mode.PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)

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

    # Keep program running to process tracking
    while not time_to_exit:
        # Short delay to prevent high CPU usage
        time.sleep(0.001)

if __name__ == "__main__":
    main()