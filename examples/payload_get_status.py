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
from payload_sdk import PayloadSdkInterface, payload_status_event_t, payload_param_t, PAYLOAD_TYPE
from payload_define import *

my_payload = None
time_to_exit = False

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
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_GB_ATTITUDE:
        # param[0]: pitch
        # param[1]: roll
        # param[2]: yaw
        print(f"Pitch: {param[0]:.2f} - Roll: {param[1]:.2f} - Yaw: {param[2]:.2f}")

    elif payload_status_event_t(event) == payload_status_event_t.PAYLOAD_PARAMS:
        # param[0]: param index
        # param[1]: value
        if payload_param_t(param[0]) == payload_param_t.PARAM_EO_ZOOM_LEVEL:
            print(f"Payload EO_ZOOM_LEVEL: {param[1]:.2f}")

        elif payload_param_t(param[0]) == payload_param_t.PARAM_IR_ZOOM_LEVEL:
            print(f"Payload IR_ZOOM_LEVEL: {param[1]:.2f}")

        elif PAYLOAD_TYPE == "VIO":
            if payload_param_t(param[0]) == payload_param_t.PARAM_LRF_RANGE:
                print(f"Payload LRF_RANGE: {param[1]:.2f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_LRF_OFSET_X:
                print(f"Payload PARAM_LRF_OFSET_X: {param[1]:.2f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_LRF_OFSET_Y:
                print(f"Payload PARAM_LRF_OFSET_Y: {param[1]:.2f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_TARGET_COOR_LON:
                print(f"Payload PARAM_TARGET_COOR_LON: {param[1]:.6f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_TARGET_COOR_LAT:
                print(f"Payload PARAM_TARGET_COOR_LAT: {param[1]:.6f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_TARGET_COOR_ALT:
                print(f"Payload PARAM_TARGET_COOR_ALT: {param[1]:.6f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_CAM_VIEW_MODE:
                print(f"Payload PARAM_CAM_VIEW_MODE: {param[1]:.2f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_CAM_REC_SOURCE:
                print(f"Payload PARAM_CAM_REC_SOURCE: {param[1]:.2f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_CAM_IR_TYPE:
                print(f"Payload PARAM_CAM_IR_TYPE: {param[1]:.2f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_CAM_IR_PALETTE_ID:
                print(f"Payload PARAM_CAM_IR_PALETTE_ID: {param[1]:.2f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_GIMBAL_MODE:
                print(f"Payload PARAM_GIMBAL_MODE: {param[1]:.2f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_PAYLOAD_GPS_LON:
                print(f"Payload PARAM_PAYLOAD_GPS_LON: {param[1]:.6f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_PAYLOAD_GPS_LAT:
                print(f"Payload PARAM_PAYLOAD_GPS_LAT: {param[1]:.6f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_PAYLOAD_GPS_ALT:
                print(f"Payload PARAM_PAYLOAD_GPS_ALT: {param[1]:.6f}")

            elif payload_param_t(param[0]) == payload_param_t.PARAM_CAM_IR_FFC_MODE:
                print(f"Payload PARAM_CAM_IR_FFC_MODE: {param[1]:.2f}")

def main():
    global my_payload, time_to_exit

    print("Starting Set gimbal mode example...")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!")

    # Register callback function
    my_payload.regPayloadStatusChanged(onPayloadStatusChanged)

    # Check connection
    my_payload.checkPayloadConnection()

    my_payload.setParamRate(payload_param_t.PARAM_EO_ZOOM_LEVEL, 1000)
    my_payload.setParamRate(payload_param_t.PARAM_IR_ZOOM_LEVEL, 1000)

    if PAYLOAD_TYPE == "VIO":
        my_payload.setParamRate(payload_param_t.PARAM_LRF_RANGE, 100)
        my_payload.setParamRate(payload_param_t.PARAM_LRF_OFSET_X, 100)
        my_payload.setParamRate(payload_param_t.PARAM_LRF_OFSET_Y, 100)

        my_payload.setParamRate(payload_param_t.PARAM_TARGET_COOR_LON, 1000)
        my_payload.setParamRate(payload_param_t.PARAM_TARGET_COOR_LAT, 1000)
        my_payload.setParamRate(payload_param_t.PARAM_TARGET_COOR_ALT, 1000)

        my_payload.setParamRate(payload_param_t.PARAM_CAM_VIEW_MODE, 1000)
        my_payload.setParamRate(payload_param_t.PARAM_CAM_REC_SOURCE, 1000)
        my_payload.setParamRate(payload_param_t.PARAM_CAM_IR_TYPE, 1000)
        my_payload.setParamRate(payload_param_t.PARAM_CAM_IR_PALETTE_ID, 1000)
        my_payload.setParamRate(payload_param_t.PARAM_GIMBAL_MODE, 1000)

        my_payload.setParamRate(payload_param_t.PARAM_PAYLOAD_GPS_LON, 1000)
        my_payload.setParamRate(payload_param_t.PARAM_PAYLOAD_GPS_LAT, 1000)
        my_payload.setParamRate(payload_param_t.PARAM_PAYLOAD_GPS_ALT, 1000)

        my_payload.setParamRate(payload_param_t.PARAM_CAM_IR_FFC_MODE, 1000)

    while not time_to_exit:
        time.sleep(0.01) 

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

if __name__ == "__main__":
    main()