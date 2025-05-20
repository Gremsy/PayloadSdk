#!/usr/bin/env python
import os
os.environ['MAVLINK20'] = "1"
os.environ['MAVLINK_DIALECT'] = "ardupilotmega"

import time
import signal
import sys
from pymavlink import mavutil
from libs.payload_sdk import PayloadSdkInterface
from libs.payload_define import *
import pretty_errors

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
    print("Starting set palette example...")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!\n")

    # Check connection
    my_payload.checkPayloadConnection()

    # Set view source to IR
    print("Set view source to IR!")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, payload_camera_view_src.PAYLOAD_CAMERA_VIEW_IR, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) 
    
    # Set IR palette
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, payload_camera_ir_palette.PAYLOAD_CAMERA_IR_PALETTE_1, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    print(" --> SET:      F1: WhiteHot        |       G1: WhiteHot")
    time.sleep(2)

    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, payload_camera_ir_palette.PAYLOAD_CAMERA_IR_PALETTE_2, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    print(" --> SET:      F1: BlackHot        |       G1: Fulgurite")
    time.sleep(2)

    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, payload_camera_ir_palette.PAYLOAD_CAMERA_IR_PALETTE_3, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    print(" --> SET:      F1: Rainbow         |       G1: IronRed")
    time.sleep(2)

    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, payload_camera_ir_palette.PAYLOAD_CAMERA_IR_PALETTE_4, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    print(" --> SET:      F1: RainbowHC       |       G1: HotIron")
    time.sleep(2)

    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, payload_camera_ir_palette.PAYLOAD_CAMERA_IR_PALETTE_5, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    print(" --> SET:      F1: Ironbow         |       G1: Medical")
    time.sleep(2)

    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, payload_camera_ir_palette.PAYLOAD_CAMERA_IR_PALETTE_6, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    print(" --> SET:      F1: Lava            |       G1: Arctic")
    time.sleep(2)

    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, payload_camera_ir_palette.PAYLOAD_CAMERA_IR_PALETTE_7, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    print(" --> SET:      F1: Arctic          |       G1: Rainbow1")
    time.sleep(2)

    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, payload_camera_ir_palette.PAYLOAD_CAMERA_IR_PALETTE_8, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    print(" --> SET:      F1: Globow          |       G1: Rainbow2")
    time.sleep(2)

    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, payload_camera_ir_palette.PAYLOAD_CAMERA_IR_PALETTE_9, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    print(" --> SET:      F1: Gradedfire      |       G1: Tint")
    time.sleep(2)

    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, payload_camera_ir_palette.PAYLOAD_CAMERA_IR_PALETTE_10, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    print(" --> SET:      F1: Hottest         |       G1: BlackHot")
    time.sleep(2)

    # Close payload interface
    try:
        my_payload.sdkQuit()
        print("Payload connection closed successfully.")
    except Exception as e:
        print(f"Error while quitting payload: {e}")  

if __name__ == "__main__":
    main()