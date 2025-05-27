#!/usr/bin/env python3
from config import config
from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import *
import math
import threading
import time
import struct
import ctypes
import sys
from typing import Callable, List, Optional
from payload_define import *
from enum_base import *
from mavlink_msg_param_ext_value import *

# Setup environment from config
config.setup_environment()

# ============================================================================
# Constants from config
# ============================================================================
from config import SDK_VERSION, PAYLOAD_TYPE

# Connection parameters from config
CONTROL_UART = config.connection.CONTROL_UART
CONTROL_UDP = config.connection.CONTROL_UDP
CONTROL_METHOD = config.connection.CONTROL_METHOD

udp_ip_target = config.connection.UDP_IP_TARGET
udp_port_target = config.connection.UDP_PORT_TARGET
payload_uart_port = config.connection.UART_PORT
payload_uart_baud = config.connection.UART_BAUDRATE

# MAVLink Component IDs from config
SYS_ID = config.mavlink.SYS_ID
COMP_ID = config.mavlink.COMP_ID

PAYLOAD_SYSTEM_ID = config.mavlink.PAYLOAD_SYSTEM_ID
PAYLOAD_COMPONENT_ID = config.mavlink.PAYLOAD_COMPONENT_ID

CAMERA_SYSTEM_ID = config.mavlink.CAMERA_SYSTEM_ID
CAMERA_COMPONENT_ID = config.mavlink.CAMERA_COMPONENT_ID

GIMBAL_SYSTEM_ID = config.mavlink.GIMBAL_SYSTEM_ID
GIMBAL_COMPONENT_ID = config.mavlink.GIMBAL_COMPONENT_ID

# Payload status event enum
class payload_status_event_t(IntEnumBase):
    PAYLOAD_CAM_CAPTURE_STATUS =                                           0
    PAYLOAD_CAM_STORAGE_INFO   =                                           1
    PAYLOAD_CAM_SETTINGS       =                                           2
    PAYLOAD_CAM_PARAMS         =                                           3
    PAYLOAD_GB_ATTITUDE        =                                           4
    PAYLOAD_GB_PARAMS          =                                           5
    PAYLOAD_ACK                =                                           6
    PAYLOAD_CAM_INFO           =                                           7
    PAYLOAD_CAM_STREAMINFO     =                                           8
    PAYLOAD_PARAMS             =                                           9
    PAYLOAD_PARAM_EXT_ACK      =                                           10

# Payload param enum
class payload_param_t(IntEnumBase):
    PARAM_EO_ZOOM_LEVEL     =                                              0
    PARAM_IR_ZOOM_LEVEL     =                                              1
    PARAM_LRF_RANGE         =                                              2
    PARAM_TRACK_POS_X       =                                              3
    PARAM_TRACK_POS_Y       =                                              4
    PARAM_TRACK_POS_W       =                                              5
    PARAM_TRACK_POS_H       =                                              6
    PARAM_TRACK_STATUS      =                                              7
    PARAM_LRF_OFSET_X       =                                              8
    PARAM_LRF_OFSET_Y       =                                              9
    PARAM_TARGET_COOR_LON   =                                              10
    PARAM_TARGET_COOR_LAT   =                                              11
    PARAM_TARGET_COOR_ALT   =                                              12
    PARAM_PAYLOAD_GPS_LON   =                                              13
    PARAM_PAYLOAD_GPS_LAT   =                                              14
    PARAM_PAYLOAD_GPS_ALT   =                                              15
    PARAM_PAYLOAD_APP_VER_X =                                              16
    PARAM_PAYLOAD_APP_VER_Y =                                              17
    PARAM_PAYLOAD_APP_VER_Z =                                              18
    PARAM_CAM_VIEW_MODE     =                                              19
    PARAM_CAM_REC_SOURCE    =                                              20
    PARAM_CAM_IR_TYPE       =                                              21
    PARAM_CAM_IR_PALETTE_ID =                                              22
    PARAM_CAM_IR_FFC_MODE   =                                              23
    PARAM_GIMBAL_MODE       =                                              24
    PARAM_COUNT             =                                              25

PAYLOAD_PARAMS = [
    {"index": payload_param_t.PARAM_EO_ZOOM_LEVEL,     "id": "EO_ZOOM",      "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_IR_ZOOM_LEVEL,     "id": "IR_ZOOM",      "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_LRF_RANGE,         "id": "LRF_RANGE",    "value": 0.0, "msg_rate": 0, "tick_ms": 0},

    {"index": payload_param_t.PARAM_TRACK_POS_X,       "id": "TRK_POS_X",    "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TRACK_POS_Y,       "id": "TRK_POS_Y",    "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TRACK_POS_W,       "id": "TRK_POS_W",    "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TRACK_POS_H,       "id": "TRK_POS_H",    "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TRACK_STATUS,      "id": "TRK_STATUS",   "value": 0.0, "msg_rate": 0, "tick_ms": 0},

    {"index": payload_param_t.PARAM_LRF_OFSET_X,       "id": "LRF_OFFSET_X", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_LRF_OFSET_Y,       "id": "LRF_OFFSET_Y", "value": 0.0, "msg_rate": 0, "tick_ms": 0},

    {"index": payload_param_t.PARAM_TARGET_COOR_LON,   "id": "TARGET_LON",   "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TARGET_COOR_LAT,   "id": "TARGET_LAT",   "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TARGET_COOR_ALT,   "id": "TARGET_ALT",   "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_PAYLOAD_GPS_LON,   "id": "PAY_LON",      "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_PAYLOAD_GPS_LAT,   "id": "PAY_LAT",      "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_PAYLOAD_GPS_ALT,   "id": "PAY_ALT",      "value": 0.0, "msg_rate": 0, "tick_ms": 0},

    {"index": payload_param_t.PARAM_PAYLOAD_APP_VER_X, "id": "APP_VER_X",    "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_PAYLOAD_APP_VER_Y, "id": "APP_VER_Y",    "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_PAYLOAD_APP_VER_Z, "id": "APP_VER_Z",    "value": 0.0, "msg_rate": 0, "tick_ms": 0},

    {"index": payload_param_t.PARAM_CAM_VIEW_MODE,     "id": "VIEW_MODE",    "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_CAM_REC_SOURCE,    "id": "REC_SRC",      "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_CAM_IR_TYPE,       "id": "IR_TYPE",      "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_CAM_IR_PALETTE_ID, "id": "PALETTE_ID",   "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_CAM_IR_FFC_MODE,   "id": "FFC_MODE",     "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    
    {"index": payload_param_t.PARAM_GIMBAL_MODE,       "id": "GB_MODE",      "value": 0.0, "msg_rate": 0, "tick_ms": 0},
]

# Input mode enum
class input_mode_t(IntEnumBase):
    INPUT_ANGLE =                                                          1
    INPUT_SPEED =                                                          2

# FFC mode enum
class ffc_mode_t(IntEnumBase):
    FFC_MODE_MANUAL =                                                      0
    FFC_MODE_AUTO   =                                                      1
    FFC_MODE_END    =                                                      2

# Capture sequence enum
class capture_sequence_t(IntEnumBase):
    IDLE                 =                                                 0
    CHECK_STORAGE        =                                                 1
    CHECK_CAPTURE_STATUS =                                                 2
    CHECK_CAMERA_MODE    =                                                 3
    CHANGE_CAMERA_MODE   =                                                 4
    DO_CAPTURE           =                                                 5
    WAIT_CAPTURE_DONE    =                                                 6

# Time lapse capture sequence enum
class time_lapse_capture_sequence_t(IntEnumBase):
    IDLE                 =                                                 0
    CHECK_STORAGE        =                                                 1
    CHECK_CAPTURE_STATUS =                                                 2
    CHECK_CAMERA_MODE    =                                                 3
    CHANGE_CAMERA_MODE   =                                                 4
    DO_CAPTURE           =                                                 5
    IMAGE_IN_CAPTURING   =                                                 6
    STOP_CAPTURING_IMAGE =                                                 7

# Record sequence enum
class record_sequence_t(IntEnumBase):
    IDLE                 =                                                 0
    CHECK_STORAGE        =                                                 1
    CHECK_CAPTURE_STATUS =                                                 2
    CHECK_CAMERA_MODE    =                                                 3
    CHANGE_CAMERA_MODE   =                                                 4
    DO_RECORD_VIDEO      =                                                 5
    VIDEO_IN_RECORDING   =                                                 6
    STOP_RECORD_VIDEO    =                                                 7
    WAIT_RECORD_DONE     =                                                 8

# Calib type enum
class calib_type_t(IntEnumBase):
    CALIB_GYRO  =                                                          0
    CALIB_ACCEL =                                                          1
    AUTO_TUNE   =                                                          2
    CALIB_MOTOR =                                                          3
    SEARCH_HOME =                                                          4

# Tracking cmd enum
class tracking_cmd_t(FloatEnumBase):
    TRACK_IDLE =                                                           0
    TRACK_ACT  =                                                           1
    TRACK_LOST =                                                           2

# Stream sequence enum
class get_stream_sequence_t(IntEnumBase):
    IDLE                =                                                  0
    CHECK_CAMERA_INFO   =                                                  1
    CHECK_STREAMING_URI =                                                  2
    START_PIPELINE      =                                                  3
    PIPELINE_RUNNING    =                                                  4

# MAVLink global position structure
class mavlink_global_position_int_t(ctypes.Structure):
    _fields_ = [
        ("time_boot_ms", ctypes.c_uint32), 
        ("lat", ctypes.c_int32),           
        ("lon", ctypes.c_int32),           
        ("alt", ctypes.c_int32),           
        ("relative_alt", ctypes.c_int32),  
        ("vx", ctypes.c_int16),           
        ("vy", ctypes.c_int16),            
        ("vz", ctypes.c_int16),           
        ("hdg", ctypes.c_uint16),     
    ]   

# MAVLink system time structure
class mavlink_system_time_t(ctypes.Structure):
    _fields_ = [
        ("time_unix_usec", ctypes.c_uint64),  
        ("time_boot_ms", ctypes.c_uint32),  
    ]

class PayloadSdkInterface:

    # Initialize the PayloadSdkInterface object. Set up default parameters for connection, system IDs, component IDs, and callbacks.
    def __init__(self):
        print(f"Starting Gremsy PayloadSdk {SDK_VERSION}")
        
        # Validate configuration
        if not config.validate_all():
            raise ValueError("Invalid configuration detected")

        self.connection_type = config.connection.CONTROL_METHOD
        self.ip              = config.connection.UDP_IP_TARGET
        self.port            = config.connection.UDP_PORT_TARGET
        self.serial_port     = config.connection.UART_PORT
        self.baudrate        = config.connection.UART_BAUDRATE

        self.master  = None

        self.sys_id               = config.mavlink.SYS_ID
        self.comp_id              = config.mavlink.COMP_ID

        self.payload_system_id    = config.mavlink.PAYLOAD_SYSTEM_ID
        self.payload_component_id = config.mavlink.PAYLOAD_COMPONENT_ID

        self.camera_system_id     = config.mavlink.CAMERA_SYSTEM_ID
        self.camera_component_id  = config.mavlink.CAMERA_COMPONENT_ID

        self.gimbal_system_id     = config.mavlink.GIMBAL_SYSTEM_ID
        self.gimbal_component_id  = config.mavlink.GIMBAL_COMPONENT_ID

        self._notifyPayloadStatusChanged: Optional[Callable[[payload_status_event_t, List[float]],      None]] = None
        self._notifyPayloadParamChanged : Optional[Callable[[payload_status_event_t, str, List[float]], None]] = None
        self._notifyPayloadStreamChanged: Optional[Callable[[payload_status_event_t, str, List[float]], None]] = None

        self.is_send_stream_request = False
        self.receive_thread         = None
        self.last_heartbeat_time    = 0  
        self.time_to_exit           = False
        self.ping_seq               = config.communication.PING_INITIAL_SEQ
        
        # Print configuration summary
        config.print_config_summary()

    ''' Connection methods '''
    # Initialize the connection to the payload via UDP or UART.
    def sdkInitConnection(self) -> bool:
        if self.connection_type == CONTROL_UDP:
            if not self.ip or not self.port:
                print("Error: IP and port must be provided for UDP connection")
                return False
            
            connection_str = f"udpout:{self.ip}:{self.port}"

        elif self.connection_type == CONTROL_UART:
            if not self.serial_port or not self.baudrate:
                print("Error: Serial port and baudrate must be provided for serial connection")

                return False
            
            connection_str = f"{self.serial_port}:{self.baudrate}"

        else:
            print("Error: Invalid connection type. Use 'udp' or 'serial'")

            return False

        try:
            print(f"{config.debug.INFO_PREFIX} Connecting to {connection_str}")

            self.master = mavutil.mavlink_connection(
                connection_str,
                source_system=self.sys_id,
                source_component=self.comp_id
            )

            # Sending initial ping
            self.master.mav.ping_send(
                int(time.time() * 1000),
                self.ping_seq,
                0,
                0
            )
            self.ping_seq += 1

            self.receive_thread = threading.Thread(target=self.payload_recv_handle)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            time.sleep(1)  # Allow some time for the thread to start

            return True

        except Exception as e:

            print(f"{config.debug.ERROR_PREFIX} SDK connection failed: {e}")
            self.master = None

            return False

    # Check the connection to the payload within a specified timeout period.
    def checkPayloadConnection(self, timeout: float = None) -> bool:
        if timeout is None:
            timeout = config.connection.CONNECTION_TIMEOUT
            
        result = False
        start_time = time.time()
        print(f"{config.debug.INFO_PREFIX} Checking payload connection")

        while not self.time_to_exit:
            if time.time() - start_time > timeout:
                print(f"{config.debug.ERROR_PREFIX} No payload detected after {timeout} seconds")
                self.sdkQuit()  
                sys.exit(1)  

            msg = self.master.recv_match(blocking=True, timeout=config.communication.MESSAGE_TIMEOUT)

            if msg is None:
                continue

            comp_id = msg.get_srcComponent()
            sys_id = msg.get_srcSystem()

            if (comp_id >= mavutil.mavlink.MAV_COMP_ID_CAMERA and
                comp_id <= mavutil.mavlink.MAV_COMP_ID_CAMERA6):

                self.camera_system_id = sys_id
                self.camera_component_id = comp_id
                result = True

            if (comp_id == mavutil.mavlink.MAV_COMP_ID_GIMBAL  or
                comp_id == mavutil.mavlink.MAV_COMP_ID_GIMBAL2 or
                comp_id == mavutil.mavlink.MAV_COMP_ID_GIMBAL3 or
                comp_id == mavutil.mavlink.MAV_COMP_ID_GIMBAL4 or
                comp_id == mavutil.mavlink.MAV_COMP_ID_GIMBAL5 or
                comp_id == mavutil.mavlink.MAV_COMP_ID_GIMBAL6):

                self.gimbal_system_id = sys_id
                self.gimbal_component_id = comp_id
                result = True

            if result:
                print(f"{config.debug.INFO_PREFIX} Payload connected!")
                return True

        if not result:
            print(f"{config.debug.ERROR_PREFIX} No payload detected!")
            self.sdkQuit()
            sys.exit(1)

        return False

    # Close the connection and stop the data receiving thread.
    def sdkQuit(self) -> None:
        self.time_to_exit = True

        if self.receive_thread:
            self.receive_thread.join()
            
        if self.master:
            self.master.close()
            self.master = None

        print(f"{config.debug.INFO_PREFIX} Connection closed.")

    ''' Camera Control methods '''
    # Set the parameter value for the payload's camera.
    def setPayloadCameraParam(self, param_id: str, param_value: int, param_type: int) -> None:
        msg = {
            'param_id': bytearray(16),  
            'param_value': bytearray(128),  
            'param_type': param_type,
            'target_system': self.camera_system_id,
            'target_component': self.camera_component_id
        }

        param_id_bytes = param_id.encode('utf-8')[:15]  

        msg['param_id'][:len(param_id_bytes)] = param_id_bytes

        struct.pack_into('<I', msg['param_value'], 0, param_value)  

        self.master.mav.param_ext_set_send(
            msg['target_system'],
            msg['target_component'],
            msg['param_id'],
            msg['param_value'],
            msg['param_type']
        )
    
    # Set the operating mode for the camera.
    def setPayloadCameraMode(self, mode: int) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_SET_CAMERA_MODE,
            1, 
            0, 
            mode, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Request the camera to capture an image.
    def setPayloadCameraCaptureImage(self, interval_s: int=0) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            1, 
            0, 
            interval_s, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Stop the interval image capturing process.
    def setPayloadCameraStopImage(self) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Start video recording.
    def setPayloadCameraRecordVideoStart(self) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Stop video recording.
    def setPayloadCameraRecordVideoStop(self) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Adjust the camera's zoom level.
    def setCameraZoom(self, zoom_type: float, zoom_value: float) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
            1, 
            zoom_type, 
            zoom_value, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Adjust the camera's focus.
    def setCameraFocus(self, focus_type: float, focus_value: float=0) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS,
            1, 
            focus_type, 
            focus_value, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Request the list of all camera parameters.
    def getPayloadCameraSettingList(self) -> None:
        self.master.mav.param_ext_request_list_send(
            self.camera_system_id ,
            self.camera_component_id
        )  
   
   # Request the value of a specific camera parameter by ID.
    def getPayloadCameraSettingByID(self, param_id: str) -> None:
        if not config.validator.validate_param_id(param_id):
            return
        
        self.master.mav.param_ext_request_read_send(
            self.camera_system_id,
            self.camera_component_id,
            param_id.encode(),
            -1
        )

    # Request the value of a camera parameter by index.
    def getPayloadCameraSettingByIndex(self, idx: int) -> None:
        if not config.validator.validate_param_index(idx):
            return
        
        self.master.mav.param_ext_request_read_send(
            self.camera_system_id,
            self.camera_component_id,
            b"",
            idx
        )

    # Request the current mode of the camera.
    def getPayloadCameraMode(self) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_SETTINGS,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Request detailed information about the camera.
    def getPayloadCameraInformation(self) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_INFORMATION,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Request information about the camera's video stream.
    def getPayloadCameraStreamingInformation(self) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Request information about the camera's storage (SD card).
    def getPayloadStorage(self) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_REQUEST_STORAGE_INFORMATION,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Request the current capture status of the camera.
    def getPayloadCaptureStatus(self) -> None:
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    # Set the message rate for a specific parameter.
    def setParamRate(self, param_index: int, time_ms: int) -> None:
        PAYLOAD_PARAMS[param_index]["msg_rate"] = time_ms
        self.sendPayloadRequestStreamRate(param_index, time_ms)

    ''' Gimbal Control methods '''
    # Set the value of a gimbal parameter by ID.
    def setPayloadGimbalParamByID(self, param_id: str, param_value: float) -> None:
        self.master.mav.param_set_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            param_id.encode(),
            param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    # Set the speed or angle for move gimbal.
    def setGimbalSpeed(self, spd_pitch: float, spd_roll: float, spd_yaw: float, mode: input_mode_t) -> None:
        if mode == input_mode_t.INPUT_ANGLE:
            # Use config validation instead of hardcoded limits
            if not config.validator.validate_gimbal_angles(spd_yaw, spd_pitch, spd_roll):
                return
            
            q = self.mavlink_euler_to_quaternion(self.to_rad(spd_roll), self.to_rad(spd_pitch), self.to_rad(spd_yaw))

            angular_velocity_x = float('nan')
            angular_velocity_y = float('nan')
            angular_velocity_z = float('nan')

        else:
            q = [float('nan')] * 4
            angular_velocity_x = self.to_rad(spd_roll)
            angular_velocity_y = self.to_rad(spd_pitch)
            angular_velocity_z = self.to_rad(spd_yaw)

        try:
            self.master.mav.gimbal_device_set_attitude_send(
                self.gimbal_system_id,
                self.gimbal_component_id,
                0,
                q,
                angular_velocity_x,
                angular_velocity_y,
                angular_velocity_z
            )

        except TypeError as e:
            print(f"{config.debug.ERROR_PREFIX} Error in gimbal_device_set_attitude_send: {e}")
            print("Ensure pymavlink is updated and supports GIMBAL_DEVICE_SET_ATTITUDE correctly.")

    # Request the list of all gimbal parameters.
    def getPayloadGimbalSettingList(self) -> None:
        self.master.mav.param_request_list_send(
            self.gimbal_system_id,
            self.gimbal_component_id
        )

    # Request the value of a specific gimbal parameter by ID.
    def getPayloadGimbalSettingByID(self, param_id: str) -> None:
        if not config.validator.validate_param_id(param_id):
            return
        
        self.master.mav.param_request_read_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            param_id.encode(),
            -1
        )

    # Request the value of a gimbal parameter by index.
    def getPayloadGimbalSettingByIndex(self, idx: int) -> None:
        if not config.validator.validate_param_index(idx):
            return

        self.master.mav.param_request_read_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            b"",
            idx
        )

    # Send command to calibrate the gimbal's gyro.
    def sendPayloadGimbalCalibGyro(self) -> None:
        self.master.mav.command_long_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            mavutil.mavlink.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            1
        )

    # Send command to calibrate the gimbal's accelerometer.
    def sendPayloadGimbalCalibAccel(self) -> None:
        self.master.mav.command_long_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            mavutil.mavlink.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            2 
        )

    # Send command to calibrate the gimbal's motor.
    def sendPayloadGimbalCalibMotor(self) -> None:
        self.master.mav.command_long_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1,  
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            1 
        )

    # Send command to search for the gimbal's home position.
    def sendPayloadGimbalSearchHome(self) -> None:
        self.master.mav.command_long_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1,  
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            2  
        )

    # Send command to auto-tune the gimbal.
    def sendPayloadGimbalAutoTune(self, status: bool) -> None:
        self.master.mav.command_long_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            mavutil.mavlink.MAV_CMD_USER_3,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            1 if status else 0  
        )

    ''' FFC Control methods '''
    # Set the FFC mode for the camera.
    def setPayloadCameraFFCMode(self, mode: int) -> None:
        if 0 > mode or mode >= ffc_mode_t.FFC_MODE_END:
            print("Invalid FFC mode")
            return
        
        self.master.mav.command_long_send(
            self.payload_system_id,
            self.payload_component_id,
            mavutil.mavlink.MAV_CMD_USER_4,
            1, 
            2, 
            6, 
            mode, 
            0, 
            0, 
            0, 
            0
        )

    # Trigger FFC calibration for the camera.
    def setPayloadCameraFFCTrigg(self) -> None:
        self.master.mav.command_long_send(
            self.payload_system_id,          
            self.payload_component_id,       
            mavutil.mavlink.MAV_CMD_USER_4,  
            1,                               
            2,                               
            7,                               
            0,                               
            0,                               
            0,                               
            0,                               
            0                                
        )

    ''' GPS methods '''
    # Send GPS position information to the payload.
    def sendPayloadGPSPosition(self, gps_data: mavlink_global_position_int_t) -> None:
        self.master.mav.global_position_int_send(
            gps_data.time_boot_ms,
            gps_data.lat,
            gps_data.lon,
            gps_data.alt,
            gps_data.relative_alt,
            gps_data.vx,
            gps_data.vy,
            gps_data.vz,
            gps_data.hdg
        )

    ''' System Time methods '''
    # Send system time information to the payload.
    def sendPayloadSystemTime(self, sys_time_data: mavlink_system_time_t) -> None:
        self.master.mav.system_time_send(
            sys_time_data.time_unix_usec,
            sys_time_data.time_boot_ms
        )

    ''' SD Card method '''
    # Format the camera's SD card.
    def setFormatSDCard(self) -> None:
        print("[INFO] Sending SD card format command")

        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_STORAGE_FORMAT,
            1, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0   
        )

    ''' Object Tracking method '''
    # Set parameters for object tracking functionality.
    def setPayloadObjectTrackingParams(self, cmd: float, pos_x: float=960, pos_y: float=540) -> None:
        self.master.mav.command_long_send(
            self.payload_system_id,
            self.payload_component_id,
            mavutil.mavlink.MAV_CMD_USER_4,
            1, 
            4, 
            0, 
            cmd, 
            pos_x, 
            pos_y, 
            0, 
            0
        )

    ''' Parameter Request methods '''
    # Request the value of a specific parameter.
    def requestParamValue(self, param_index: int) -> None:
        if not config.validator.validate_param_index(param_index):
            return
        
        try:
            param_id = PAYLOAD_PARAMS[param_index]["id"]
        except (IndexError, KeyError):
            print(f"{config.debug.ERROR_PREFIX} Invalid param_index {param_index} or missing 'id' in PAYLOAD_PARAMS")
            return
        
        if not config.validator.validate_param_id(param_id):
            return
        
        self.master.mav.param_request_read_send(
            self.payload_system_id,
            self.payload_component_id,
            param_id.encode(),
            -1
        )

    # Send a request for the message rate of a parameter.
    def sendPayloadRequestStreamRate(self, index: int, time_ms: int) -> None:
        self.master.mav.command_long_send(
            self.payload_system_id,
            self.payload_component_id,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, 
            index, 
            time_ms * 1000, 
            0, 
            0, 
            0, 
            0, 
            1
        )

    # Request the message rate for all parameters with a set msg_rate.
    def requestMessageStreamInterval(self) -> None:
        for param in PAYLOAD_PARAMS:
            if param["msg_rate"] >= 0:
                self.sendPayloadRequestStreamRate(param["index"], param["msg_rate"])

    ''' Message Handler methods '''
    # Handle the PARAM_EXT_VALUE message from MAVLink.
    def _handle_msg_param_ext_value(self, msg: MAVLink_param_ext_value_message) -> None:
        if self._notifyPayloadParamChanged:
            buf = msg.get_msgbuf()

            # get_msgbuf() returns the raw bytes of the message
            # MAVLink message structure:
            # typedef struct __mavlink_message {
            #     uint16_t checksum;      ///< sent at end of packet
            #     uint8_t magic;          ///< protocol magic marker
            #     uint8_t len;            ///< Length of payload
            #     uint8_t incompat_flags; ///< flags that must be understood
            #     uint8_t compat_flags;   ///< flags that can be ignored if not understood
            #     uint8_t seq;            ///< Sequence of packet
            #     uint8_t sysid;          ///< ID of message sender system/aircraft
            #     uint8_t compid;         ///< ID of the message sender component
            #     uint32_t msgid:24;      ///< ID of message in payload
            #     uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];
            #     uint8_t ck[2];          ///< incoming checksum bytes
            #     uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
            # } mavlink_message_t;

            # Header: 10 byte (magic, len, incompat_flags, compat_flags, seq, sysid, compid, msgid).
            # Payload: (149 byte).
            # Checksum và Signature:(2 byte checksum + 13 byte signature).

            # Check if the message buffer is long enough to contain the header and payload
            if len(buf) < 10 + MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN:
                print("Invalid message buffer length for PARAM_EXT_VALUE")
                return

            # Extract the payload from the message buffer
            payload = buf[10:10 + MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN]
            
            # Extract param_value from the payload
            param_value_bytes = payload[20:20 + 128]
            
            # Check if the param_value_bytes is long enough to contain the uint32 value
            if len(param_value_bytes) < 4:
                print("param_value_bytes too short")
                return
            
            # Unpack the first 4 bytes of param_value_bytes as a little-endian uint32
            param_uint32 = struct.unpack('<I', param_value_bytes[:4])[0]
            
            params = [float(msg.param_index), float(param_uint32)]
            param_id = msg.param_id
            self._notifyPayloadParamChanged(payload_status_event_t.PAYLOAD_CAM_PARAMS, param_id, params)

    # Handle the COMMAND_ACK message from MAVLink.
    def _handle_msg_command_ack(self, msg: MAVLink_command_ack_message) -> None:
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_ACK
            param = [msg.command, msg.result, msg.progress]
            self._notifyPayloadStatusChanged(event, param)

    # Handle the CAMERA_INFORMATION message from MAVLink.
    def _handle_msg_camera_information(self, msg: MAVLink_camera_information_message) -> None:
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_CAM_INFO
            param = [msg.flags]
            self._notifyPayloadStatusChanged(event, param)

    # Handle the CAMERA_STREAM_INFORMATION message from MAVLink.
    def _handle_msg_camera_stream_information(self, msg: MAVLink_video_stream_information_message) -> None:
        if self._notifyPayloadStreamChanged:
            event = payload_status_event_t.PAYLOAD_CAM_STREAMINFO
            param = [msg.type, msg.resolution_v, msg.resolution_h]
            param_char = msg.uri.rstrip('\0')
            self._notifyPayloadStreamChanged(event, param_char, param)

    # Handle the STORAGE_INFORMATION message from MAVLink.
    def _handle_msg_storage_information(self, msg: MAVLink_storage_information_message) -> None:
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_CAM_STORAGE_INFO
            param = [msg.total_capacity, msg.used_capacity, msg.available_capacity, msg.status]
            self._notifyPayloadStatusChanged(event, param)

    # Handle the CAMERA_CAPTURE_STATUS message from MAVLink.
    def _handle_msg_camera_capture_status(self, msg: MAVLink_camera_capture_status_message) -> None:
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_CAM_CAPTURE_STATUS
            param = [msg.image_status, msg.video_status, msg.image_count, msg.recording_time_ms]
            self._notifyPayloadStatusChanged(event, param)

    # Handle the CAMERA_SETTINGS message from MAVLink.
    def _handle_msg_camera_settings(self, msg: MAVLink_camera_settings_message) -> None:
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_CAM_SETTINGS
            param = [msg.mode_id, msg.zoomLevel, msg.focusLevel]
            self._notifyPayloadStatusChanged(event, param)

    # Handle the MOUNT_ORIENTATION message from MAVLink.
    def _handle_msg_mount_orientation(self, msg: MAVLink_mount_orientation_message) -> None:
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_GB_ATTITUDE
            param = [msg.pitch, msg.roll, msg.yaw]
            self._notifyPayloadStatusChanged(event, param)

    # Handle the PARAM_VALUE message from MAVLink.
    def _handle_msg_param_value(self, msg: MAVLink_param_value_message) -> None:

        if msg.get_srcComponent() == self.gimbal_component_id and self._notifyPayloadParamChanged:
            event = payload_status_event_t.PAYLOAD_GB_PARAMS
            param = [msg.param_index, msg.param_value]
            param_char = msg.param_id.rstrip('\0')
            self._notifyPayloadParamChanged(event, param_char, param)

        elif msg.get_srcComponent() == self.payload_component_id and self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_PARAMS
            param = [msg.param_index, msg.param_value]
            self._notifyPayloadStatusChanged(event, param)
    
    # Handle the COMMAND_EXT_ACK message from MAVLink.
    def _handle_msg_command_ext_ack(self, msg: MAVLink_param_ext_ack_message) -> None:
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_PARAM_EXT_ACK
            param = [msg.param_result]
            self._notifyPayloadStatusChanged(event, param)

    # Main loop to receive and process MAVLink messages.
    def payload_recv_handle(self) -> None:
        self.last_heartbeat_time = time.time()
        
        while self.time_to_exit == False:
            current_time = time.time()

            if current_time - self.last_heartbeat_time >= config.communication.HEARTBEAT_INTERVAL:
                self.master.mav.ping_send(
                    int(time.time() * 1000),
                    self.ping_seq,
                    0,
                    0  
                )
                self.ping_seq += 1

            msg = self.master.recv_match(blocking=True, timeout=config.communication.MESSAGE_TIMEOUT)

            if msg is None:
                continue

            msgid = msg.get_msgId()
            sysid = msg.get_srcSystem()
            compid = msg.get_srcComponent()

            if compid == self.payload_component_id:
                self.payload_system_id = sysid

                if not self.is_send_stream_request:
                    # Only need to send 1 time, after get the sys_id of the payload
                    self.requestMessageStreamInterval()
                    self.is_send_stream_request = True

            # Update gimbal id
            if compid == mavutil.mavlink.MAV_COMP_ID_GIMBAL  or \
               compid == mavutil.mavlink.MAV_COMP_ID_GIMBAL2 or \
               compid == mavutil.mavlink.MAV_COMP_ID_GIMBAL3 or \
               compid == mavutil.mavlink.MAV_COMP_ID_GIMBAL4 or \
               compid == mavutil.mavlink.MAV_COMP_ID_GIMBAL5 or \
               compid == mavutil.mavlink.MAV_COMP_ID_GIMBAL6:
                
                self.gimbal_system_id = sysid
                self.gimbal_component_id = compid
            
            # Update camera id
            if compid >= mavutil.mavlink.MAV_COMP_ID_CAMERA and \
               compid <= mavutil.mavlink.MAV_COMP_ID_CAMERA6:
                
                self.camera_system_id = sysid
                self.camera_component_id = compid

            if msgid ==  mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:
                self.last_heartbeat_time = current_time  

            elif msgid == mavutil.mavlink.MAVLINK_MSG_ID_PARAM_EXT_VALUE:
                self._handle_msg_param_ext_value(msg)

            elif msgid == mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_ACK:
                self._handle_msg_command_ack(msg)

            elif msgid == mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_INFORMATION:
                self._handle_msg_camera_information(msg)

            elif msgid == mavutil.mavlink.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION:
                self._handle_msg_camera_stream_information(msg)

            elif msgid == mavutil.mavlink.MAVLINK_MSG_ID_STORAGE_INFORMATION:
                self._handle_msg_storage_information(msg)

            elif msgid == mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
                self._handle_msg_camera_capture_status(msg)
            
            elif msgid == mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_SETTINGS:
                self._handle_msg_camera_settings(msg)

            elif msgid == mavutil.mavlink.MAVLINK_MSG_ID_MOUNT_ORIENTATION:
                self._handle_msg_mount_orientation(msg)

            elif msgid == mavutil.mavlink.MAVLINK_MSG_ID_PARAM_VALUE:
                self._handle_msg_param_value(msg)

            elif msgid == mavutil.mavlink.MAVLINK_MSG_ID_PARAM_EXT_ACK:
                self._handle_msg_command_ext_ack(msg)

            time.sleep(config.communication.RECV_THREAD_SLEEP)

    ''' Helper functions '''
    # Convert angle from degrees to radians.
    def to_rad(self, deg: float) -> float:
        return deg * math.pi / 180.0

    # Convert angle from radians to degrees.
    def to_deg(self, rad: float) -> float:
        return rad * 180.0 / math.pi

    # Convert Euler angles to quaternion.
    def mavlink_euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> list:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4

        q[0] = cy * cp * cr + sy * sp * sr  # w
        q[1] = cy * cp * sr - sy * sp * cr  # x
        q[2] = sy * cp * sr + cy * sp * cr  # y
        q[3] = sy * cp * cr - cy * sp * sr  # z

        return q

    ''' Callback registration methods '''
    # Register a callback to receive notifications when the payload status changes.
    def regPayloadStatusChanged(self, callback: Callable[[payload_status_event_t, List[float]], None]) -> None:
        if not callable(callback):
            raise ValueError("Callback must be a callable object")
        
        self._notifyPayloadStatusChanged = callback

    # Register a callback to receive notifications when a payload parameter changes.
    def regPayloadParamChanged(self, callback: Callable[[payload_status_event_t, str, List[float]], None]) -> None:
        if not callable(callback):
            raise ValueError("Callback must be a callable object")
        
        self._notifyPayloadParamChanged = callback

    # Register a callback to receive notifications when the payload's video stream information changes.
    def regPayloadStreamChanged(self, callback: Callable[[payload_status_event_t, str, List[float]], None]) -> None:
        if not callable(callback):
            raise ValueError("Callback must be a callable object")
        
        self._notifyPayloadStreamChanged = callback