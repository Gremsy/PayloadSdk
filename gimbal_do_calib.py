import time
import signal
import sys
from enum import Enum
import os
from libs.payload_sdk import PayloadSdkInterface, payload_status_event_t, calib_type_t
from libs.payload_define import *

# Define constants for MAVLink commands
MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION = 42503  
MAV_CMD_DO_SET_HOME = 179                     
MAV_CMD_USER_3 = 300                       
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_IN_PROGRESS = 5

my_payload = None
is_calibration_running = False
is_exit = False
start_time = time.time() * 1000000 

# Set the calibration type
my_calib = calib_type_t.CALIB_GYRO

# SDK log function
def sdk_log(func_name, message):
    elapsed_time = int((time.time() * 1000000) - start_time)
    print(f"[{elapsed_time}] SDK {func_name}(): {message}")

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
def on_payload_status_changed(event: int, param: list):
    global is_calibration_running, is_exit
    
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_ACK:

        cmd_id, result, progress = param[0], param[1], param[2]
        # sdk_log("onPayloadStatusChanged", f"Got ack from {cmd_id:.0f}, result {result:.0f}, progress: {progress:.0f}")

        if my_calib == calib_type_t.CALIB_GYRO:
            if cmd_id == MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:

                if result == MAV_RESULT_ACCEPTED:
                    is_calibration_running = True

                if progress == MAV_RESULT_ACCEPTED and is_calibration_running:
                    sdk_log("onPayloadStatusChanged", "The gyro calibration done!")
                    is_exit = True

                elif progress == MAV_RESULT_IN_PROGRESS:
                    is_calibration_running = True
                    sdk_log("onPayloadStatusChanged", "The gyro calibration is processing...")

        elif my_calib == calib_type_t.CALIB_ACCEL:
            if cmd_id == MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:

                if result == MAV_RESULT_ACCEPTED:
                    is_calibration_running = True

                if progress == MAV_RESULT_ACCEPTED and is_calibration_running:
                    sdk_log("onPayloadStatusChanged", "The accel calibration done!")
                    is_exit = True

                elif progress == MAV_RESULT_IN_PROGRESS:
                    is_calibration_running = True
                    sdk_log("onPayloadStatusChanged", "The accel calibration is processing...")

        elif my_calib == calib_type_t.CALIB_MOTOR:
            if cmd_id == MAV_CMD_DO_SET_HOME:

                if result == MAV_RESULT_ACCEPTED:
                    is_calibration_running = True
                    time.sleep(1)

                if progress == MAV_RESULT_ACCEPTED and is_calibration_running:
                    sdk_log("onPayloadStatusChanged", "The motor calibration done!")
                    is_exit = True

                elif progress == MAV_RESULT_IN_PROGRESS:
                    is_calibration_running = True
                    sdk_log("onPayloadStatusChanged", "The motor calibration is processing...")

        elif my_calib == calib_type_t.AUTO_TUNE:
            if cmd_id == MAV_CMD_USER_3:

                if result == MAV_RESULT_ACCEPTED:
                    is_calibration_running = True
                    time.sleep(1)

                if progress == MAV_RESULT_ACCEPTED and is_calibration_running:
                    sdk_log("onPayloadStatusChanged", "The Auto tune done!")
                    time.sleep(1)
                    is_exit = True

                elif progress == MAV_RESULT_IN_PROGRESS:
                    is_calibration_running = True
                    sdk_log("onPayloadStatusChanged", "The Auto tune is processing...")

        elif my_calib == calib_type_t.SEARCH_HOME:
            if cmd_id == MAV_CMD_DO_SET_HOME:

                if result == MAV_RESULT_ACCEPTED:
                    is_calibration_running = True
                    time.sleep(1)

                if progress == MAV_RESULT_ACCEPTED and is_calibration_running:
                    sdk_log("onPayloadStatusChanged", "The SearchHome done!")
                    is_exit = True

                elif progress == MAV_RESULT_IN_PROGRESS:
                    is_calibration_running = True
                    sdk_log("onPayloadStatusChanged", "The SearchHome is processing...")

# Callback function for payload param changes
def on_payload_param_changed(event: int, param_char: str, param: list):
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_PARAMS:
        sdk_log("onPayloadParamChanged", f"--> Payload_param: {param_char}, value: {param[1]:.2f}")
    
    elif payload_status_event_t(event) == payload_status_event_t.PAYLOAD_GB_PARAMS:
        sdk_log("onPayloadParamChanged", f"--> Gimbal_param: index: {param[0]:.0f}, id: {param_char}, value: {param[1]:.0f}")

def main():
    global my_payload, is_calibration_running, is_exit, my_calib

    print("Starting Do Calib gimbal example...")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!")

    # Register callback function
    my_payload.regPayloadStatusChanged(on_payload_status_changed)

    # Check connection
    my_payload.checkPayloadConnection()

    if my_calib == calib_type_t.CALIB_GYRO:
        is_calibration_running = False
        my_payload.sendPayloadGimbalCalibGyro()
        sdk_log("main", "Calib gyro command was sent. Waiting for the calibration done...")

    elif my_calib == calib_type_t.CALIB_ACCEL:
        is_calibration_running = False
        my_payload.sendPayloadGimbalCalibAccel()
        sdk_log("main", "Calib accel command was sent. Waiting for the calibration done...")

    elif my_calib == calib_type_t.CALIB_MOTOR:
        is_calibration_running = False
        my_payload.sendPayloadGimbalCalibMotor()
        sdk_log("main", "Calib motor command was sent. Waiting for the calibration done...")

    elif my_calib == calib_type_t.AUTO_TUNE:
        is_calibration_running = False
        my_payload.sendPayloadGimbalAutoTune(True)
        sdk_log("main", "Auto tune command was sent. Waiting for the process done...")

    elif my_calib == calib_type_t.SEARCH_HOME:
        is_calibration_running = False
        my_payload.sendPayloadGimbalSearchHome()
        sdk_log("main", "Searching Home command was sent. Waiting for the calibration done...")

    # Wait for the calibration process to complete
    while not is_exit:
        time.sleep(1)  

    # Load parameters to verify calibration
    if my_calib == calib_type_t.CALIB_GYRO:
        sdk_log("main", "Load the Gyro offset values...")
        my_payload.getPayloadGimbalSettingByID("GYROX_OFFSET")
        my_payload.getPayloadGimbalSettingByID("GYROY_OFFSET")
        my_payload.getPayloadGimbalSettingByID("GYROZ_OFFSET")

    elif my_calib == calib_type_t.CALIB_ACCEL:
        sdk_log("main", "Load the accel offset values...")
        my_payload.getPayloadGimbalSettingByID("ACCELX_OFFSET")
        my_payload.getPayloadGimbalSettingByID("ACCELY_OFFSET")
        my_payload.getPayloadGimbalSettingByID("ACCELZ_OFFSET")

    elif my_calib == calib_type_t.AUTO_TUNE:
        sdk_log("main", "Waiting for the gimbal rebooted...20s")
        time.sleep(20) 
        sdk_log("main", "Load the Stiffness/Holdstrength values...")
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
    sdk_log("main", "Exit.")

if __name__ == "__main__":
    main()