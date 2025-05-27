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
from pymavlink import mavutil
from payload_sdk import PayloadSdkInterface, payload_status_event_t, calib_type_t
from payload_define import *

my_payload = None
is_calibration_running = False
is_exit = False
start_time = time.time() * 1000000 

# SDK log function
def sdk_log(func_name, message):
    elapsed_time = int((time.time() * 1000000) - start_time)
    print(f"[{elapsed_time}] SDK {func_name}(): {message}")

# Set the calibration 
my_calib = calib_type_t.CALIB_GYRO

# Signal handler for quitting
def quit_handler(sig, frame):
    global my_payload, is_exit
    print("\nTERMINATING AT USER REQUEST")
    is_exit = True

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

    # End program    
    sys.exit(0)

# Callback function for payload status changes
def onPayloadStatusChanged(event: int, param: list):
    global is_calibration_running, is_exit, my_calib
    
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_ACK:

        cmd_id, result, progress = param[0], param[1], param[2]
        # sdk_log("onPayloadStatusChanged", f"Got ack from {cmd_id:.0f}, result {result:.0f}, progress: {progress:.0f}")

        if my_calib == calib_type_t.CALIB_GYRO:
            if cmd_id == mavutil.mavlink.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:

                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    is_calibration_running = True

                if progress == mavutil.mavlink.MAV_RESULT_ACCEPTED and is_calibration_running:
                    sdk_log("onPayloadStatusChanged", "The gyro calibration done!")
                    is_exit = True

                elif progress == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    is_calibration_running = True
                    sdk_log("onPayloadStatusChanged", "The gyro calibration is processing")

        elif my_calib == calib_type_t.CALIB_ACCEL:
            if cmd_id == mavutil.mavlink.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:

                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    is_calibration_running = True

                if progress == mavutil.mavlink.MAV_RESULT_ACCEPTED and is_calibration_running:
                    sdk_log("onPayloadStatusChanged", "The accel calibration done!")
                    is_exit = True

                elif progress == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    is_calibration_running = True
                    sdk_log("onPayloadStatusChanged", "The accel calibration is processing")

        elif my_calib == calib_type_t.CALIB_MOTOR:
            if cmd_id == mavutil.mavlink.MAV_CMD_DO_SET_HOME:

                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    is_calibration_running = True
                    time.sleep(1)

                if progress == mavutil.mavlink.MAV_RESULT_ACCEPTED and is_calibration_running:
                    sdk_log("onPayloadStatusChanged", "The motor calibration done!")
                    is_exit = True

                elif progress == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    is_calibration_running = True
                    sdk_log("onPayloadStatusChanged", "The motor calibration is processing")

        elif my_calib == calib_type_t.AUTO_TUNE:
            if cmd_id == mavutil.mavlink.MAV_CMD_USER_3:

                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    is_calibration_running = True
                    time.sleep(1)

                if progress == mavutil.mavlink.MAV_RESULT_ACCEPTED and is_calibration_running:
                    sdk_log("onPayloadStatusChanged", "The Auto tune done!")
                    time.sleep(1)
                    is_exit = True

                elif progress == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    is_calibration_running = True
                    sdk_log("onPayloadStatusChanged", "The Auto tune is processing")

        elif my_calib == calib_type_t.SEARCH_HOME:
            if cmd_id == mavutil.mavlink.MAV_CMD_DO_SET_HOME:

                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    is_calibration_running = True
                    time.sleep(1)

                if progress == mavutil.mavlink.MAV_RESULT_ACCEPTED and is_calibration_running:
                    sdk_log("onPayloadStatusChanged", "The SearchHome done!")
                    is_exit = True

                elif progress == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    is_calibration_running = True
                    sdk_log("onPayloadStatusChanged", "The SearchHome is processing")

# Callback function for payload param changes   
def onPayloadParamChanged(event: int, param_char: str, param: list):
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_PARAMS:
        # param[0]: param_index
		# param[1]: value
        sdk_log("onPayloadParamChanged", f"--> Payload_param: {param_char}, value: {param[1]:.2f}")

    elif payload_status_event_t(event) == payload_status_event_t.PAYLOAD_GB_PARAMS:
        # param[0]: param_index
		# param[1]: value
        sdk_log("onPayloadParamChanged", f"--> Gimbal_param: index: {param[0]:.0f}, id: {param_char}, value: {param[1]:.0f}")

def main():
    global my_payload, is_calibration_running, is_exit, my_calib

    print("Starting Do Calib gimbal example")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!")

    # Register callback function
    my_payload.regPayloadStatusChanged(onPayloadStatusChanged)
    my_payload.regPayloadParamChanged(onPayloadParamChanged)

    # Check connection
    my_payload.checkPayloadConnection()

    if my_calib == calib_type_t.CALIB_GYRO:
        is_calibration_running = False
        my_payload.sendPayloadGimbalCalibGyro()
        sdk_log("main", "Calib gyro command was sent. Waiting for the calibration done")

    elif my_calib == calib_type_t.CALIB_ACCEL:
        is_calibration_running = False
        my_payload.sendPayloadGimbalCalibAccel()
        sdk_log("main", "Calib accel command was sent. Waiting for the calibration done")

    elif my_calib == calib_type_t.CALIB_MOTOR:
        is_calibration_running = False
        my_payload.sendPayloadGimbalCalibMotor()
        sdk_log("main", "Calib motor command was sent. Waiting for the calibration done")

    elif my_calib == calib_type_t.AUTO_TUNE:
        is_calibration_running = False
        my_payload.sendPayloadGimbalAutoTune(True)
        sdk_log("main", "Auto tune command was sent. Waiting for the process done")

    elif my_calib == calib_type_t.SEARCH_HOME:
        is_calibration_running = False
        my_payload.sendPayloadGimbalSearchHome()
        sdk_log("main", "Searching Home command was sent. Waiting for the calibration done")

    # Wait for the calibration process to complete
    while not is_exit:
        time.sleep(1)  

    # Load parameters to verify calibration
    if my_calib == calib_type_t.CALIB_GYRO:
        sdk_log("main", "Load the Gyro offset values")

        my_payload.getPayloadGimbalSettingByID("GYRO_OFFSETX")
        my_payload.getPayloadGimbalSettingByID("GYRO_OFFSETY")
        my_payload.getPayloadGimbalSettingByID("GYRO_OFFSETZ")

    elif my_calib == calib_type_t.CALIB_ACCEL:
        sdk_log("main", "Load the accel offset values")

        my_payload.getPayloadGimbalSettingByID("ACC_OFFSETX")
        my_payload.getPayloadGimbalSettingByID("ACC_OFFSETY")
        my_payload.getPayloadGimbalSettingByID("ACC_OFFSETZ")

    elif my_calib == calib_type_t.AUTO_TUNE:
        sdk_log("main", "Waiting for the gimbal rebooted 20s")
        time.sleep(20) 
        sdk_log("main", "Load the Stiffness/Holdstrength values")

        my_payload.getPayloadGimbalSettingByID("STIFF_TILT")
        my_payload.getPayloadGimbalSettingByID("STIFF_ROLL")
        my_payload.getPayloadGimbalSettingByID("STIFF_PAN")
        my_payload.getPayloadGimbalSettingByID("PWR_TILT")
        my_payload.getPayloadGimbalSettingByID("PWR_ROLL")
        my_payload.getPayloadGimbalSettingByID("PWR_PAN")

    elif my_calib == calib_type_t.SEARCH_HOME:
        pass
    
    elif my_calib == calib_type_t.CALIB_MOTOR:
        pass

    time.sleep(1)  
    print("Exit.")

if __name__ == "__main__":
    main()