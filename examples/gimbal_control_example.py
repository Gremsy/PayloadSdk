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
from payload_sdk import PayloadSdkInterface, payload_status_event_t, input_mode_t
from payload_define import *

my_payload = None

# Gimbal control class
class GimbalSetModeExample(PayloadSdkInterface):
    def __init__(self):
        super().__init__()
        self.currentPitch: float = 0.0
        self.currentRoll:  float = 0.0
        self.currentYaw:   float = 0.0

    # Set gimbal stop and re-center the yaw angle.
    def setRecenterYaw(self) -> None:
        # Keep gimbal stopped, delay 1.5s
        self.setGimbalSpeed(0, 0, 0, input_mode_t.INPUT_SPEED)
        time.sleep(1.5)

        # Re-center the yaw angle
        self.setGimbalSpeed(self.currentPitch, self.currentRoll, 0, input_mode_t.INPUT_ANGLE)

        # Wait until yaw nearly zero (busy-wait like C++)
        while abs(self.currentYaw) >= 0.2:
            pass  

    # Set gimbal stop and re-center the pitch angle.
    def setRecenterPitch(self) -> None:
        # Keep gimbal stopped, delay 1.5s
        self.setGimbalSpeed(0, 0, 0, input_mode_t.INPUT_SPEED)
        time.sleep(1.5)

        # Re-center the pitch angle
        self.setGimbalSpeed(0, self.currentRoll, self.currentYaw, input_mode_t.INPUT_ANGLE)

        # Wait until pitch nearly zero (busy-wait like C++)
        while abs(self.currentPitch) >= 0.2:
            pass  

    # Update all current angles of the gimbal (no print to match C++)
    def updateAllCurrentAngle(self, currentPitch: float, currentRoll: float, currentYaw: float) -> None:
        self.currentPitch = currentPitch
        self.currentRoll = currentRoll
        self.currentYaw = currentYaw

# Signal handler for quitting
def quit_handler(sig, frame):
    global my_payload
    print("\nTERMINATING AT USER REQUEST\n")
    try:
        my_payload.sdkQuit()
    except Exception:
        pass
    sys.exit(0)

# Callback function for payload status changes
def onPayloadStatusChanged(event: int, param: list):
    global my_payload
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_GB_ATTITUDE:
        pitch = param[0]
        roll  = param[1]
        yaw   = param[2]
        my_payload.updateAllCurrentAngle(pitch, roll, yaw)

# Callback function for payload param changes   
def onPayloadParamChanged(event: int, param_char: str, param: list):
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_GB_ATTITUDE:
        pass  

def main():
    global my_payload
    print("Starting Set gimbal mode example...")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = GimbalSetModeExample()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!")

    # Register callback function
    my_payload.regPayloadStatusChanged(onPayloadStatusChanged)
    my_payload.regPayloadParamChanged(onPayloadParamChanged)

    # Check connection
    my_payload.checkPayloadConnection()

    # Set FOLLOW mode
    print("Gimbal set mode FOLLOW, delay in 3 secs")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, payload_camera_gimbal_mode.PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(3.0)

    print("Move gimbal angular velocity: pitch = -20 deg/s, yaw = 50 deg/s, delay in 3 secs")
    my_payload.setGimbalSpeed(-20, 0, 50, input_mode_t.INPUT_SPEED)
    time.sleep(3.0)

    print("Gimbal set re-center YAW ------")
    my_payload.setRecenterYaw()

    print("Gimbal set re-center PITCH ------")
    my_payload.setRecenterPitch()

    print("Move gimbal angular position: pitch = -40 deg, yaw = 90 deg, delay in 3 secs")
    my_payload.setGimbalSpeed(-40, 0, 90, input_mode_t.INPUT_ANGLE)
    time.sleep(3.0)

    print("Gimbal set re-center PITCH ------")
    my_payload.setRecenterPitch()

    print("Gimbal set re-center YAW ------")
    my_payload.setRecenterYaw()

    print("Delay in 3 secs before changing to LOCK MODE")
    time.sleep(3.0)

    # Set LOCK mode
    print("Gimbal set mode LOCK, delay in 3 secs")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, payload_camera_gimbal_mode.PAYLOAD_CAMERA_GIMBAL_MODE_LOCK, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(3.0)

    print("Move gimbal angular velocity: pitch = 20 deg/s, yaw = -50 deg/s, delay in 3 secs")
    my_payload.setGimbalSpeed(20, 0, -50, input_mode_t.INPUT_SPEED)
    time.sleep(3.0)

    print("Gimbal set re-center PITCH ------")
    my_payload.setRecenterPitch()

    print("Gimbal set re-center YAW ------")
    my_payload.setRecenterYaw()

    print("Move gimbal angular position: pitch = 40 deg, yaw = -90 deg, delay in 3 secs")
    my_payload.setGimbalSpeed(40, 0, -90, input_mode_t.INPUT_ANGLE)
    time.sleep(3.0)

    print("Gimbal set re-center PITCH ------")
    my_payload.setRecenterPitch()

    print("Gimbal set re-center YAW ------")
    my_payload.setRecenterYaw()

    print("Delay in 3 secs before changing to FOLLOW MODE")
    time.sleep(3.0)

    # Set FOLLOW mode again
    print("Gimbal set mode FOLLOW, delay in 3 secs")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, payload_camera_gimbal_mode.PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(3.0)

    print("Move gimbal angular velocity: pitch = -20 deg/s, yaw = 50 deg/s, delay in 3 secs")
    my_payload.setGimbalSpeed(-20, 0, 50, input_mode_t.INPUT_SPEED)
    time.sleep(3.0)

    print("Gimbal set re-center YAW ------")
    my_payload.setRecenterYaw()

    print("Gimbal set re-center PITCH ------")
    my_payload.setRecenterPitch()

    print("Move gimbal angular position: pitch = -40 deg, yaw = 90 deg, delay in 3 secs")
    my_payload.setGimbalSpeed(-40, 0, 90, input_mode_t.INPUT_ANGLE)
    time.sleep(3.0)

    print("Gimbal set re-center PITCH ------")
    my_payload.setRecenterPitch()

    print("Gimbal set re-center YAW ------")
    my_payload.setRecenterYaw()

    print("Delay in 3 secs before changing to MAPPING MODE")
    time.sleep(3.0)

    # Set MAPPING mode
    print("Gimbal set mode MAPPING, delay in 5 secs")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, payload_camera_gimbal_mode.PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(5.0)

    # Set OFF mode
    print("Gimbal set mode OFF, delay in 5 secs")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, payload_camera_gimbal_mode.PAYLOAD_CAMERA_GIMBAL_MODE_OFF, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(5.0)

    # Set RESET mode
    print("Gimbal set mode RESET, delay in 5 secs")
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, payload_camera_gimbal_mode.PAYLOAD_CAMERA_GIMBAL_MODE_RESET, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(5.0)

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception:
        pass

if __name__ == "__main__":
    main()