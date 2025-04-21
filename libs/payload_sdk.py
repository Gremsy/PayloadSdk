from pymavlink import mavutil
import math
import threading
import time
import struct
import ctypes
from enum import IntEnum
from .payload_define import *
from .enum_base import *

# Default connection parameters
CONNECTION_TYPE = "udp"
IP = "192.168.12.248"
PORT = 14566
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

SDK_VERSION = "3.0.0_build.04022025"
PAYLOAD_TYPE = "VIO"

# Define Mavlink Component
SYS_ID = 1
MAV_COMP_ID_ONBOARD_COMPUTER3 = 193

PAYLOAD_SYSTEM_ID = 1
MAV_COMP_ID_USER2 = 26

CAMERA_SYSTEM_ID = 1
MAV_COMP_ID_CAMERA = 100

GIMBAL_SYSTEM_ID = 1
MAV_COMP_ID_GIMBAL = 154

# Connection info structures
class T_ConnInfo_Uart(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char_p),
                ("baudrate", ctypes.c_int)]

class T_ConnInfo_UDP(ctypes.Structure):
    _fields_ = [("ip", ctypes.c_char_p),
                ("port", ctypes.c_int)]

class T_ConnInfo(ctypes.Union):
    _fields_ = [("uart", T_ConnInfo_Uart),
                ("udp", T_ConnInfo_UDP)]

class T_ConnInfoStruct(ctypes.Structure):
    _anonymous_ = ("device",)
    _fields_ = [("type", ctypes.c_uint8),
                ("device", T_ConnInfo)]

# Param type enum
class param_type(IntEnumBase):
    PARAM_TYPE_UINT8 = 1
    PARAM_TYPE_INT8 = 2
    PARAM_TYPE_UINT16 = 3
    PARAM_TYPE_INT16 = 4
    PARAM_TYPE_UINT32 = 5
    PARAM_TYPE_INT32 = 6
    PARAM_TYPE_UINT64 = 7
    PARAM_TYPE_INT64 = 8
    PARAM_TYPE_REAL32 = 9
    PARAM_TYPE_REAL64 = 10

# Payload status event enum
class payload_status_event_t(IntEnumBase):
    PAYLOAD_CAM_CAPTURE_STATUS = 0
    PAYLOAD_CAM_STORAGE_INFO = 1
    PAYLOAD_CAM_SETTINGS = 2
    PAYLOAD_CAM_PARAMS = 3
    PAYLOAD_GB_ATTITUDE = 4
    PAYLOAD_GB_PARAMS = 5
    PAYLOAD_ACK = 6
    PAYLOAD_CAM_INFO = 7
    PAYLOAD_CAM_STREAMINFO = 8
    PAYLOAD_PARAMS = 9
    PAYLOAD_PARAM_EXT_ACK = 10

# Payload param enum
class payload_param_t(IntEnumBase):
    PARAM_EO_ZOOM_LEVEL = 0
    PARAM_IR_ZOOM_LEVEL = 1
    PARAM_LRF_RANGE = 2
    PARAM_TRACK_POS_X = 3
    PARAM_TRACK_POS_Y = 4
    PARAM_TRACK_POS_W = 5
    PARAM_TRACK_POS_H = 6
    PARAM_TRACK_STATUS = 7
    PARAM_LRF_OFSET_X = 8
    PARAM_LRF_OFSET_Y = 9
    PARAM_TARGET_COOR_LON = 10
    PARAM_TARGET_COOR_LAT = 11
    PARAM_TARGET_COOR_ALT = 12
    PARAM_PAYLOAD_GPS_LON = 13
    PARAM_PAYLOAD_GPS_LAT = 14
    PARAM_PAYLOAD_GPS_ALT = 15
    PARAM_PAYLOAD_APP_VER_X = 16
    PARAM_PAYLOAD_APP_VER_Y = 17
    PARAM_PAYLOAD_APP_VER_Z = 18
    PARAM_CAM_VIEW_MODE = 19
    PARAM_CAM_REC_SOURCE = 20
    PARAM_CAM_IR_TYPE = 21
    PARAM_CAM_IR_PALETTE_ID = 22
    PARAM_CAM_IR_FFC_MODE = 23
    PARAM_GIMBAL_MODE = 24
    PARAM_COUNT = 25

PAYLOAD_PARAMS = [
    {"index": payload_param_t.PARAM_EO_ZOOM_LEVEL, "id": "EO_ZOOM", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_IR_ZOOM_LEVEL, "id": "IR_ZOOM", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_LRF_RANGE, "id": "LRF_RANGE", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TRACK_POS_X, "id": "TRK_POS_X", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TRACK_POS_Y, "id": "TRK_POS_Y", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TRACK_POS_W, "id": "TRK_POS_W", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TRACK_POS_H, "id": "TRK_POS_H", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TRACK_STATUS, "id": "TRK_STATUS", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_LRF_OFSET_X, "id": "LRF_OFFSET_X", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_LRF_OFSET_Y, "id": "LRF_OFFSET_Y", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TARGET_COOR_LON, "id": "TARGET_LON", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TARGET_COOR_LAT, "id": "TARGET_LAT", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_TARGET_COOR_ALT, "id": "TARGET_ALT", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_PAYLOAD_GPS_LON, "id": "PAY_LON", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_PAYLOAD_GPS_LAT, "id": "PAY_LAT", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_PAYLOAD_GPS_ALT, "id": "PAY_ALT", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_PAYLOAD_APP_VER_X, "id": "APP_VER_X", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_PAYLOAD_APP_VER_Y, "id": "APP_VER_Y", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_PAYLOAD_APP_VER_Z, "id": "APP_VER_Z", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_CAM_VIEW_MODE, "id": "VIEW_MODE", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_CAM_REC_SOURCE, "id": "REC_SRC", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_CAM_IR_TYPE, "id": "IR_TYPE", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_CAM_IR_PALETTE_ID, "id": "PALETTE_ID", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_CAM_IR_FFC_MODE, "id": "FFC_MODE", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
    {"index": payload_param_t.PARAM_GIMBAL_MODE, "id": "GB_MODE", "value": 0.0, "msg_rate": 0, "tick_ms": 0},
]

# Input mode enum
class input_mode_t(IntEnumBase):
    INPUT_ANGLE = 1
    INPUT_SPEED = 2

# FFC mode enum
class ffc_mode_t(IntEnumBase):
    FFC_MODE_MANUAL = 0
    FFC_MODE_AUTO = 1
    FFC_MODE_END = 2

# Capture sequence enum
class capture_sequence_t(IntEnumBase):
    IDLE = 0
    CHECK_STORAGE = 1
    CHECK_CAPTURE_STATUS = 2
    CHECK_CAMERA_MODE = 3
    CHANGE_CAMERA_MODE = 4
    DO_CAPTURE = 5
    WAIT_CAPTURE_DONE = 6

# Time lapse capture sequence enum
class time_lapse_capture_sequence_t(IntEnumBase):
    IDLE = 0
    CHECK_STORAGE = 1
    CHECK_CAPTURE_STATUS = 2
    CHECK_CAMERA_MODE = 3
    CHANGE_CAMERA_MODE = 4
    DO_CAPTURE = 5
    IMAGE_IN_CAPTURING = 6
    STOP_CAPTURING_IMAGE = 7

# Record sequence enum
class record_sequence_t(IntEnumBase):
    IDLE = 0
    CHECK_STORAGE = 1
    CHECK_CAPTURE_STATUS = 2
    CHECK_CAMERA_MODE = 3
    CHANGE_CAMERA_MODE = 4
    DO_RECORD_VIDEO = 5
    VIDEO_IN_RECORDING = 6
    STOP_RECORD_VIDEO = 7
    WAIT_RECORD_DONE = 8

# Calib type enum
class calib_type_t(IntEnumBase):
    CALIB_GYRO = 0
    CALIB_ACCEL = 1
    AUTO_TUNE = 2
    CALIB_MOTOR = 3
    SEARCH_HOME = 4

# Tracking cmd enum
class tracking_cmd_t(FloatEnumBase):
    TRACK_IDLE = 0
    TRACK_ACT = 1
    TRACK_LOST = 2

# Stream sequence enum
class get_stream_sequence_t(IntEnumBase):
    IDLE = 0
    CHECK_CAMERA_INFO = 1
    CHECK_STREAMING_URI = 2
    START_PIPELINE = 3
    PIPELINE_RUNNING = 4

class camera_zoom_type(FloatEnumBase):
   ZOOM_TYPE_STEP            =                                             0 
   ZOOM_TYPE_CONTINUOUS      =                                             1 
   ZOOM_TYPE_RANGE           =                                             2 
   ZOOM_TYPE_FOCAL_LENGTH    =                                             3
   ZOOM_TYPE_HORIZONTAL_FOV  =                                             4 
   CAMERA_ZOOM_TYPE_ENUM_END =                                             5

class camera_mode(IntEnumBase):
   CAMERA_MODE_IMAGE        =                                              0
   CAMERA_MODE_VIDEO        =                                              1
   CAMERA_MODE_IMAGE_SURVEY =                                              2
   CAMERA_MODE_ENUM_END     =                                              3

class camera_cap_flags(IntEnumBase):
   CAMERA_CAP_FLAGS_CAPTURE_VIDEO                   =                      1
   CAMERA_CAP_FLAGS_CAPTURE_IMAGE                   =                      2
   CAMERA_CAP_FLAGS_HAS_MODES                       =                      4
   CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE =                      8
   CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE =                      16
   CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE           =                      32
   CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM                  =                      64
   CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS                 =                      128
   CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM                =                      256
   CAMERA_CAP_FLAGS_HAS_TRACKING_POINT              =                      512
   CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE          =                      1024
   CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS         =                      2048
   CAMERA_CAP_FLAGS_HAS_THERMAL_RANGE               =                      4096
   CAMERA_CAP_FLAGS_ENUM_END                        =                      4097

class video_stream_type(IntEnumBase):
   VIDEO_STREAM_TYPE_RTSP     =                                            0
   VIDEO_STREAM_TYPE_RTPUDP   =                                            1
   VIDEO_STREAM_TYPE_TCP_MPEG =                                            2
   VIDEO_STREAM_TYPE_MPEG_TS  =                                            3
   VIDEO_STREAM_TYPE_ENUM_END =                                            4 

# MAVLink message structures
class mavlink_message_t(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("checksum", ctypes.c_uint16),
        ("magic", ctypes.c_uint8),
        ("len", ctypes.c_uint8),
        ("incompat_flags", ctypes.c_uint8),
        ("compat_flags", ctypes.c_uint8),
        ("seq", ctypes.c_uint8),
        ("sysid", ctypes.c_uint8),
        ("compid", ctypes.c_uint8),
        ("msgid", ctypes.c_uint8 * 3), 
        ("payload64", ctypes.c_uint64 * 33),
        ("ck", ctypes.c_uint8 * 2),
        ("signature", ctypes.c_uint8 * 13),
    ]

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
    def __init__(self, connection_type=None, ip=None, port=None, serial_port=None, baudrate=None, sys_id=1, comp_id=MAV_COMP_ID_ONBOARD_COMPUTER3):
        print(f"Starting Gremsy PayloadSdk {SDK_VERSION}")
        self.connection_type = connection_type.lower() if connection_type else CONNECTION_TYPE
        self.ip = ip if ip else IP
        self.port = port if port else PORT
        self.serial_port = serial_port if serial_port else SERIAL_PORT
        self.baudrate = baudrate if baudrate else BAUDRATE
        self.sys_id = sys_id
        self.comp_id = comp_id
        self.master = None
        self.payload_system_id = PAYLOAD_SYSTEM_ID
        self.payload_component_id = MAV_COMP_ID_USER2
        self.camera_system_id = CAMERA_SYSTEM_ID
        self.camera_component_id = MAV_COMP_ID_CAMERA
        self.gimbal_system_id = GIMBAL_SYSTEM_ID
        self.gimbal_component_id = MAV_COMP_ID_GIMBAL
        self._notifyPayloadStatusChanged = None
        self._notifyPayloadParamChanged = None
        self._notifyPayloadStreamChanged = None
        self._notifyCameraDetected = None  
        self.camera_detected = False  
        self.running = False
        self.is_stream_requested = False
        self.receive_thread = None
        self.last_ping_time = 0  
        self.ping_seq = 0

    # Helper functions
    def to_rad(self, deg):
        return deg * math.pi / 180.0

    def to_deg(self, rad):
        return rad * 180.0 / math.pi

    def _euler_to_quaternion(self, roll, pitch, yaw):
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

    # Callback registration methods
    def regPayloadStatusChanged(self, callback):
        self._notifyPayloadStatusChanged = callback

    def regPayloadParamChanged(self, callback):
        self._notifyPayloadParamChanged = callback

    def regPayloadStreamChanged(self, callback):
        self._notifyPayloadStreamChanged = callback

    # Message handling methods
    def _handle_msg_heartbeat(self, msg):
        comp_id = msg.get_srcComponent()
        if comp_id == mavutil.mavlink.MAV_COMP_ID_USER2:
            self.payload_system_id = msg.get_srcSystem()
            self.payload_component_id = comp_id
            if not self.is_stream_requested:
                self.requestMessageStreamInterval()
                self.is_stream_requested = True
        elif comp_id in range(mavutil.mavlink.MAV_COMP_ID_CAMERA, mavutil.mavlink.MAV_COMP_ID_CAMERA6 + 1):
            self.camera_system_id = msg.get_srcSystem()
            self.camera_component_id = comp_id
        elif comp_id in range(mavutil.mavlink.MAV_COMP_ID_GIMBAL, mavutil.mavlink.MAV_COMP_ID_GIMBAL6 + 1):
            self.gimbal_system_id = msg.get_srcSystem()
            self.gimbal_component_id = comp_id

    def _handle_msg_param_ext_value(self, msg):
        
        if self._notifyPayloadParamChanged:
            param_value_bytes = bytearray()
            for char in msg.param_value:
                if ord(char) < 256: 
                    param_value_bytes.append(ord(char))
                else:
                    param_value_bytes.append(0)  
            param_value_bytes = bytes(param_value_bytes[:128]) 

            if len(param_value_bytes) >= 4:
                param_value = struct.unpack('<I', param_value_bytes[:4])[0]
            else:
                param_value = 0 

            params = [float(msg.param_index), float(param_value)] 
            param_char = msg.param_id.rstrip('\0')
            self._notifyPayloadParamChanged(payload_status_event_t.PAYLOAD_CAM_PARAMS, param_char, params)

    def _handle_msg_param_ext_ack(self, msg):
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_PARAM_EXT_ACK
            param = [msg.param_result]
            self._notifyPayloadStatusChanged(event, param)

    def _handle_msg_command_ack(self, msg):
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_ACK
            param = [msg.command, msg.result, msg.progress]
            self._notifyPayloadStatusChanged(event, param)

    def _handle_msg_camera_information(self, msg):
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_CAM_INFO
            param = [msg.flags]
            self._notifyPayloadStatusChanged(event, param)

    def _handle_msg_video_stream_information(self, msg):
        if self._notifyPayloadStreamChanged:
            event = payload_status_event_t.PAYLOAD_CAM_STREAMINFO
            param = [msg.type, msg.resolution_v, msg.resolution_h]
            param_char = msg.uri.rstrip('\0')
            self._notifyPayloadStreamChanged(event, param_char, param)

    def _handle_msg_storage_information(self, msg):
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_CAM_STORAGE_INFO
            param = [msg.total_capacity, msg.used_capacity, msg.available_capacity, msg.status]
            self._notifyPayloadStatusChanged(event, param)

    def _handle_msg_camera_capture_status(self, msg):
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_CAM_CAPTURE_STATUS
            param = [msg.image_status, msg.video_status, msg.image_count, msg.recording_time_ms]
            self._notifyPayloadStatusChanged(event, param)

    def _handle_msg_camera_settings(self, msg):
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_CAM_SETTINGS
            param = [msg.mode_id, msg.zoomLevel, msg.focusLevel]
            self._notifyPayloadStatusChanged(event, param)

    def _handle_msg_mount_orientation(self, msg):
        if self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_GB_ATTITUDE
            param = [msg.pitch, msg.roll, msg.yaw]
            self._notifyPayloadStatusChanged(event, param)

    def _handle_msg_param_value(self, msg):
        if msg.get_srcComponent() == self.gimbal_component_id and self._notifyPayloadParamChanged:
            event = payload_status_event_t.PAYLOAD_GB_PARAMS
            param = [msg.param_index, msg.param_value]
            param_char = msg.param_id.rstrip('\0')
            self._notifyPayloadParamChanged(event, param_char, param)
        elif msg.get_srcComponent() == self.payload_component_id and self._notifyPayloadStatusChanged:
            event = payload_status_event_t.PAYLOAD_PARAMS
            param = [msg.param_index, msg.param_value]
            self._notifyPayloadStatusChanged(event, param)

    # Core message receiving loop
    def _receive_messages(self):
            while self.running:
                if not self.master:
                    time.sleep(0.1)
                    continue
                
                # Gửi PING định kỳ nếu chưa tìm thấy camera
                current_time = time.time()
                if not self.camera_detected and current_time - self.last_ping_time >= 1:
                    self.master.mav.ping_send(
                        int(current_time * 1000),
                        self.ping_seq,
                        self.sys_id,
                        1  # TARGET_COMP_ID
                    )
                    self.ping_seq += 1
                    self.last_ping_time = current_time

                msg = self.master.recv_match(blocking=True, timeout=0.1)
                if msg is None:
                    continue
                msg_type = msg.get_type()
                # print(f"Received message: {msg_type}, sysid: {msg.get_srcSystem()}, compid: {msg.get_srcComponent()}")

                if msg_type == 'HEARTBEAT':
                    self._handle_msg_heartbeat(msg)
                elif msg_type == 'PARAM_EXT_VALUE':
                    self._handle_msg_param_ext_value(msg)
                elif msg_type == 'PARAM_EXT_ACK':
                    self._handle_msg_param_ext_ack(msg)
                elif msg_type == 'COMMAND_ACK':
                    self._handle_msg_command_ack(msg)
                elif msg_type == 'CAMERA_INFORMATION':
                    self._handle_msg_camera_information(msg)
                elif msg_type == 'VIDEO_STREAM_INFORMATION':
                    self._handle_msg_video_stream_information(msg)
                elif msg_type == 'STORAGE_INFORMATION':
                    self._handle_msg_storage_information(msg)
                elif msg_type == 'CAMERA_CAPTURE_STATUS':
                    self._handle_msg_camera_capture_status(msg)
                elif msg_type == 'CAMERA_SETTINGS':
                    self._handle_msg_camera_settings(msg)
                elif msg_type == 'MOUNT_ORIENTATION':
                    self._handle_msg_mount_orientation(msg)
                elif msg_type == 'PARAM_VALUE':
                    self._handle_msg_param_value(msg)
                time.sleep(0.0001)

    # Init connection to payload
    def sdkInitConnection(self):
        if self.connection_type == 'udp':
            if not self.ip or not self.port:
                print("Error: IP and port must be provided for UDP connection")
                return False
            connection_str = f'udpout:{self.ip}:{self.port}'
        elif self.connection_type == 'serial':
            if not self.serial_port or not self.baudrate:
                print("Error: Serial port and baudrate must be provided for serial connection")
                return False
            connection_str = f'{self.serial_port}:{self.baudrate}'
        else:
            print("Error: Invalid connection type. Use 'udp' or 'serial'")
            return False

        try:
            print(f"[INFO] Connecting to {connection_str}...")
            self.master = mavutil.mavlink_connection(
                connection_str,
                source_system=self.sys_id,
                source_component=self.comp_id
            )
            time.sleep(1)
            print("[INFO] Waiting for heartbeat from payload...")
            self.master.wait_heartbeat(timeout=0)
            print(f"[INFO] Heartbeat received from system {self.master.target_system}, component {self.master.target_component}")
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_messages)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            time.sleep(1)
            return True
        except Exception as e:
            print(f"[ERROR] SDK connection failed: {e}")
            self.master = None
            return False

    def checkPayloadConnection(self, timeout: float = 10.0) -> bool:
        if not self.master:
            print("Error: Connection not initialized")
            return False

        start_time = time.time()
        result = False

        print("[INFO] Checking payload connection...")
        while self.running and (time.time() - start_time) < timeout:
            msg = self.master.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue

            comp_id = msg.get_srcComponent()
            sys_id = msg.get_srcSystem()

            # Check for camera
            if comp_id in range(mavutil.mavlink.MAV_COMP_ID_CAMERA, mavutil.mavlink.MAV_COMP_ID_CAMERA6 + 1):
                self.camera_system_id = sys_id
                self.camera_component_id = comp_id
                if not self.camera_detected:
                    self.camera_detected = True
                    print(f"[INFO] Camera detected with sys_id: {sys_id}, comp_id: {comp_id}")
                    if self._notifyCameraDetected:
                        self._notifyCameraDetected(True)
                result = True

            # Check for gimbal
            elif comp_id in range(mavutil.mavlink.MAV_COMP_ID_GIMBAL, mavutil.mavlink.MAV_COMP_ID_GIMBAL6 + 1):
                self.gimbal_system_id = sys_id
                self.gimbal_component_id = comp_id
                print(f"[INFO] Gimbal detected with sys_id: {sys_id}, comp_id: {comp_id}")
                result = True

            # Check for payload
            elif comp_id == mavutil.mavlink.MAV_COMP_ID_USER2:
                self.payload_system_id = sys_id
                self.payload_component_id = comp_id
                print(f"[INFO] Payload detected with sys_id: {sys_id}, comp_id: {comp_id}")
                if not self.is_stream_requested:
                    self.requestMessageStreamInterval()
                    self.is_stream_requested = True
                result = True

            if result:
                print("[INFO] Payload connected!")
                return True

            time.sleep(0.0001)

        print("[ERROR] No payload detected within timeout period")
        return False

    # Interface terminator
    def sdkQuit(self):
            self.running = False
            if self.receive_thread:
                self.receive_thread.join()
            if self.master:
                self.master.close()
                self.master = None
            if self.camera_detected and self._notifyCameraDetected:
                self.camera_detected = False
                self._notifyCameraDetected(False)
            print("[INFO] Connection closed.")

    # Camera methods
    def setPayloadCameraParam(self, param_id: str, param_value, param_type: param_type):
        if not self.master:
            print("Error: Connection not initialized")
            return
        param_value_bytes = bytearray(128)
        if param_type == param_type.PARAM_TYPE_UINT32:
            struct.pack_into('<I', param_value_bytes, 0, param_value)
        elif param_type == param_type.PARAM_TYPE_REAL32:
            struct.pack_into('<f', param_value_bytes, 0, param_value)
        try:
            self.master.mav.param_ext_set_send(
                self.camera_system_id,
                self.camera_component_id,
                param_id.encode(),
                param_value_bytes,
                param_type
            )
        except AttributeError:
            self.master.mav.param_set_send(
                self.camera_system_id,
                self.camera_component_id,
                param_id.encode(),
                param_value,
                param_type
            )

    def getPayloadCameraSettingList(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.param_ext_request_list_send(
            self.camera_system_id,
            self.camera_component_id
        )

    def getPayloadCameraSettingByID(self, param_id: str):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.param_request_read_send(
            self.camera_system_id,
            self.camera_component_id,
            param_id.encode(),
            -1
        )

    def getPayloadCameraSettingByIndex(self, idx: int):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.param_request_read_send(
            self.camera_system_id,
            self.camera_component_id,
            b"",
            idx
        )

    def getPayloadStorage(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_REQUEST_STORAGE_INFORMATION,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def getPayloadCaptureStatus(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def getPayloadCameraMode(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_SETTINGS,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def getPayloadCameraInformation(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_INFORMATION,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def getPayloadCameraStreamingInformation(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def setPayloadCameraMode(self, mode: int):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_SET_CAMERA_MODE,
            0, 0, mode, 0, 0, 0, 0, 0
        )

    def setPayloadCameraCaptureImage(self, interval_s: int=0):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            0, 0, interval_s, 1, 0, 0, 0, 0
        )

    def setPayloadCameraStopImage(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def setPayloadCameraRecordVideoStart(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def setPayloadCameraRecordVideoStop(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def setParamRate(self, param_index: int, time_ms: int):
        if not self.master:
            print("Error: Connection not initialized")
            return
        PAYLOAD_PARAMS[param_index]["msg_rate"] = time_ms
        self.sendPayloadRequestStreamRate(param_index, time_ms)

    def setCameraZoom(self, zoom_type: float, zoom_value: float):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
            0, zoom_type, zoom_value, 0, 0, 0, 0, 0
        )

    def setCameraFocus(self, focus_type: float, focus_value: float=0):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.camera_system_id,
            self.camera_component_id,
            mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS,
            0, focus_type, focus_value, 0, 0, 0, 0, 0
        )

    # Gimbal methods
    def setPayloadGimbalParamByID(self, param_id: str, param_value: float):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.param_set_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            param_id.encode(),
            param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def sendPayloadGimbalCalibGyro(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            mavutil.mavlink.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION,
            0, 0, 0, 0, 0, 0, 0, 1
        )

    def sendPayloadGimbalCalibAccel(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            mavutil.mavlink.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION,
            0, 0, 0, 0, 0, 0, 0, 2
        )

    def sendPayloadGimbalCalibMotor(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0, 0, 0, 0, 0, 0, 0, 1
        )

    def sendPayloadGimbalSearchHome(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0, 0, 0, 0, 0, 0, 0, 2
        )

    def sendPayloadGimbalAutoTune(self, status: bool):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            mavutil.mavlink.MAV_CMD_USER_3,
            0, 0, 0, 0, 0, 0, 0, 1 if status else 0
        )

    def getPayloadGimbalSettingList(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.param_request_list_send(
            self.gimbal_system_id,
            self.gimbal_component_id
        )

    def getPayloadGimbalSettingByID(self, param_id: str):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.param_request_read_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            param_id.encode(),
            -1
        )

    def getPayloadGimbalSettingByIndex(self, idx: int):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.param_request_read_send(
            self.gimbal_system_id,
            self.gimbal_component_id,
            b"",
            idx
        )

    def setGimbalSpeed(self, spd_pitch: float, spd_roll: float, spd_yaw: float, mode: input_mode_t):
        if not self.master:
            print("Error: Connection not initialized")
            return
        if mode == input_mode_t.INPUT_ANGLE:
            if spd_yaw > 180 or spd_yaw < -180:
                print("ERROR: Gimbal Protocol V2 only supports yaw axis from -180 degrees to 180 degrees!")
                return
            if spd_roll > 180 or spd_roll < -180:
                print("ERROR: Gimbal Protocol V2 only supports roll axis from -180 degrees to 180 degrees!")
                return
            if spd_pitch > 90 or spd_pitch < -90:
                print("ERROR: Gimbal Protocol V2 only supports pitch axis from -90 degrees to 90 degrees!")
                return
            q = self._euler_to_quaternion(self.to_rad(spd_roll), self.to_rad(spd_pitch), self.to_rad(spd_yaw))
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
            print(f"Error in gimbal_device_set_attitude_send: {e}")
            print("Ensure pymavlink is updated and supports GIMBAL_DEVICE_SET_ATTITUDE correctly.")

    # FFC methods
    def setPayloadCameraFFCMode(self, mode: int):
        if not self.master:
            print("Error: Connection not initialized")
            return
        if mode >= ffc_mode_t.FFC_MODE_END:
            print("Invalid FFC mode")
            return
        self.master.mav.command_long_send(
            self.payload_system_id,
            self.payload_component_id,
            mavutil.mavlink.MAV_CMD_USER_4,
            0, 2, 6, mode, 0, 0, 0, 0
        )

    def setPayloadCameraFFCTrigg(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.payload_system_id,
            self.payload_component_id,
            mavutil.mavlink.MAV_CMD_USER_4,
            0, 2, 7, 0, 0, 0, 0, 0
        )

    def getPayloadCameraFFCMode(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        raise NotImplementedError("getPayloadCameraFFCMode is not implemented yet.")

    # GPS and system time methods
    def sendPayloadGPSPosition(self, lat, lon, alt, alt_rel, heading, vx, vy, vz):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.global_position_int_send(
            int(time.time() * 1e3),
            lat,
            lon,
            alt,
            alt_rel,
            vx,
            vy,
            vz,
            heading
        )

    def sendPayloadSystemTime(self, unix_time_us):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.system_time_send(
            unix_time_us,
            int(time.time() * 1e3)
        )

    # Tracking method
    def setPayloadObjectTrackingParams(self, cmd: float, pos_x: float=960, pos_y: float=540):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.payload_system_id,
            self.payload_component_id,
            mavutil.mavlink.MAV_CMD_USER_4,
            0, 4, 0, cmd, pos_x, pos_y, 0, 0
        )

    def requestParamValue(self, param_index):
        if not self.master:
            print("Error: Connection not initialized")
            return
        param_id = PAYLOAD_PARAMS[param_index]["id"]
        self.master.mav.param_request_read_send(
            self.payload_system_id,
            self.payload_component_id,
            param_id.encode(),
            -1
        )

    def sendPayloadRequestStreamRate(self, index, time_ms):
        if not self.master:
            print("Error: Connection not initialized")
            return
        self.master.mav.command_long_send(
            self.payload_system_id,
            self.payload_component_id,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, index, time_ms * 1000, 0, 0, 0, 0, 1
        )

    def requestMessageStreamInterval(self):
        if not self.master:
            print("Error: Connection not initialized")
            return
        for param in PAYLOAD_PARAMS:
            if param["msg_rate"] > 0:
                self.sendPayloadRequestStreamRate(param["index"], param["msg_rate"])

    def regCameraDetected(self, callback):
        self._notifyCameraDetected = callback

    def _handle_msg_heartbeat(self, msg):
        comp_id = msg.get_srcComponent()
        if comp_id == mavutil.mavlink.MAV_COMP_ID_USER2:
            self.payload_system_id = msg.get_srcSystem()
            self.payload_component_id = comp_id
            if not self.is_stream_requested:
                self.requestMessageStreamInterval()
                self.is_stream_requested = True
        elif comp_id in [
            mavutil.mavlink.MAV_COMP_ID_CAMERA,
            mavutil.mavlink.MAV_COMP_ID_CAMERA2,
            mavutil.mavlink.MAV_COMP_ID_CAMERA3,
            mavutil.mavlink.MAV_COMP_ID_CAMERA4,
            mavutil.mavlink.MAV_COMP_ID_CAMERA5,
            mavutil.mavlink.MAV_COMP_ID_CAMERA6,
        ]:
            self.camera_system_id = msg.get_srcSystem()
            self.camera_component_id = comp_id
            if not self.camera_detected:
                self.camera_detected = True
                print(f"[INFO] Camera detected with component ID: {comp_id}")
                if self._notifyCameraDetected:
                    self._notifyCameraDetected(True)
        elif comp_id in range(mavutil.mavlink.MAV_COMP_ID_GIMBAL, mavutil.mavlink.MAV_COMP_ID_GIMBAL6 + 1):
            self.gimbal_system_id = msg.get_srcSystem()
            self.gimbal_component_id = comp_id