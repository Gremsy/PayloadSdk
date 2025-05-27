#!/usr/bin/env python3
import sys
import os

# Add the libs directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'libs'))

# Import config first to setup environment automatically
from config import config

import time
import signal
import random
import ctypes
from pymavlink import mavutil
from payload_sdk import PayloadSdkInterface, mavlink_global_position_int_t, mavlink_gps_raw_int_t
from payload_define import *

my_payload = None

# Signal handler for quitting
def quit_handler(sig, frame):
    global my_payload
    print("\nTERMINATING AT USER REQUEST")

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

    # End program    
    sys.exit(0)

# Simulate GPS fix status - in real application, this would come from actual GPS hardware
current_gps_fix_type = mavutil.mavlink.GPS_FIX_TYPE_NO_FIX

# GPS coordinates for movement simulation
class GPSCoordinate(ctypes.Structure):
    _fields_ = [
        ("lat", ctypes.c_double),
        ("lon", ctypes.c_double),
        ("alt", ctypes.c_double),
        ("heading", ctypes.c_double)
    ]

# Start point: New York City
start_pos = GPSCoordinate(40.730610, -73.935242, 50.0, 225.0)  # Heading Southwest
# End point: Philadelphia  
end_pos = GPSCoordinate(39.952583, -75.165222, 80.0, 225.0)

def getCurrentGPSPosition(msg_cnt):
    movement_start = 100
    total_movement_time = 500  # 500 messages to complete the journey
    
    if msg_cnt < movement_start:
        return start_pos
    elif msg_cnt < movement_start + total_movement_time:
        progress = (msg_cnt - movement_start) / total_movement_time
        lat = start_pos.lat + (end_pos.lat - start_pos.lat) * progress
        lon = start_pos.lon + (end_pos.lon - start_pos.lon) * progress
        alt = start_pos.alt + (end_pos.alt - start_pos.alt) * progress
        heading = start_pos.heading
        # Add some random variation to make it more realistic
        variation = 0.0001
        lat += (random.random() - 0.5) * variation
        lon += (random.random() - 0.5) * variation
        alt += (random.random() - 0.5) * 0.5  # ±0.25m altitude variation
        return GPSCoordinate(lat, lon, alt, heading)
    else:
        return end_pos

def updateGPSFixStatus(msg_cnt):
    global current_gps_fix_type
    if msg_cnt < 20:
        current_gps_fix_type = mavutil.mavlink.GPS_FIX_TYPE_NO_FIX
    else:
        current_gps_fix_type = mavutil.mavlink.GPS_FIX_TYPE_DGPS

def main():
    global my_payload

    print("Starting SendGPS example with moving coordinates...")
    print("Route: New York City -> Philadelphia")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!")

    # Check connection
    my_payload.checkPayloadConnection()

    msg_cnt = 0
    boot_time_ms = 0

    while True:
        # Update GPS fix status simulation
        updateGPSFixStatus(msg_cnt)
        
        # Get current GPS position (changing continuously)
        current_gps = getCurrentGPSPosition(msg_cnt)
        
        fix_type_str = "NO_FIX" if current_gps_fix_type == mavutil.mavlink.GPS_FIX_TYPE_NO_FIX else "DGPS"
        print(f"GPS Fix Type: {current_gps_fix_type} ({fix_type_str}) | Lat: {current_gps.lat:.6f}, Lon: {current_gps.lon:.6f}, Alt: {current_gps.alt:.1f}m")

        # Always send GPS_RAW_INT to allow GPS to get fix
        gps_raw = mavlink_gps_raw_int_t()
        gps_raw.time_usec = int(time.time() * 1e6)  # Current time in microseconds
        gps_raw.fix_type = current_gps_fix_type
        gps_raw.lat = int(current_gps.lat * 1e7)
        gps_raw.lon = int(current_gps.lon * 1e7)
        gps_raw.alt = int(current_gps.alt * 1e3)
        gps_raw.eph = 100  # HDOP * 100
        gps_raw.epv = 150  # VDOP * 100
        gps_raw.vel = 500 if msg_cnt > 100 and msg_cnt < 600 else 0  # 5 m/s when moving
        gps_raw.cog = int(current_gps.heading * 100)  # Course over ground in cdeg
        gps_raw.satellites_visible = 8 if current_gps_fix_type == mavutil.mavlink.GPS_FIX_TYPE_DGPS else 3
        gps_raw.alt_ellipsoid = int(current_gps.alt * 1e3)
        gps_raw.h_acc = 2000  # Horizontal accuracy in mm
        gps_raw.v_acc = 3000  # Vertical accuracy in mm
        gps_raw.vel_acc = 100  # Velocity accuracy in mm/s
        gps_raw.hdg_acc = 500  # Heading accuracy in degE5
        gps_raw.yaw = int(current_gps.heading * 100)  # Yaw in cdeg

        my_payload.sendPayloadGPSRawInt(gps_raw)
        print(f"Send GPS_RAW_INT to payload: {msg_cnt} (Fix Type: {current_gps_fix_type})")

        # Only send GLOBAL_POSITION_INT when GPS fix is successful (DGPS)
        if current_gps_fix_type == mavutil.mavlink.GPS_FIX_TYPE_DGPS:
            gps = mavlink_global_position_int_t()
            gps.time_boot_ms = boot_time_ms
            boot_time_ms += 100
            gps.lat = int(current_gps.lat * 1e7)
            gps.lon = int(current_gps.lon * 1e7)
            gps.alt = int(current_gps.alt * 1e3)
            gps.relative_alt = 0
            gps.vx = 0  # Unused in this example
            gps.vy = 0  # Unused in this example
            gps.vz = 0  # Unused in this example
            gps.hdg = int(current_gps.heading)  # Heading in degrees

            my_payload.sendPayloadGPSPosition(gps)
            print(f"Send GLOBAL_POSITION_INT to payload: {msg_cnt} (DGPS Fix Available)")

        msg_cnt += 1
        time.sleep(0.1)  # 10Hz

if __name__ == "__main__":
    main()