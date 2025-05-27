#!/usr/bin/env python3
"""
Configuration file for Gremsy Payload SDK
Contains all configurable parameters and settings
"""

import os
from typing import Dict, Any
from pymavlink import mavutil

# ============================================================================
# SDK Information
# ============================================================================
SDK_VERSION = "3.0.0_build.27052025"
PAYLOAD_TYPE = "VIO"

# ============================================================================
# Connection Configuration
# ============================================================================
class ConnectionConfig:
    """Connection configuration settings"""
    
    # Connection methods
    CONTROL_UART = 0
    CONTROL_UDP = 1
    
    # Default connection method
    CONTROL_METHOD = CONTROL_UDP
    
    # UDP Configuration
    UDP_IP_TARGET = "192.168.12.238"    # Payload IP address
    UDP_PORT_TARGET = 14566             # Do not change
    
    # UART Configuration
    UART_PORT = "/dev/ttyUSB0"
    UART_BAUDRATE = 115200
    
    # Connection timeout
    CONNECTION_TIMEOUT = 5.0  # seconds

# ============================================================================
# MAVLink System Configuration
# ============================================================================
class MAVLinkConfig:
    """MAVLink system and component ID configuration"""
    
    # Ground Control Station IDs
    SYS_ID = 1
    COMP_ID = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER3
    
    # Payload IDs
    PAYLOAD_SYSTEM_ID = 1
    PAYLOAD_COMPONENT_ID = mavutil.mavlink.MAV_COMP_ID_USER2     # Do not change
    
    # Camera IDs (auto-updated when receiving messages)
    CAMERA_SYSTEM_ID = 1
    CAMERA_COMPONENT_ID = mavutil.mavlink.MAV_COMP_ID_CAMERA
    
    # Gimbal IDs (auto-updated when receiving messages)
    GIMBAL_SYSTEM_ID = 1
    GIMBAL_COMPONENT_ID = mavutil.mavlink.MAV_COMP_ID_GIMBAL

# ============================================================================
# Camera Configuration
# ============================================================================
class CameraConfig:
    """Camera-related configuration"""
    
    # Camera modes
    CAMERA_MODE_IMAGE = 0
    CAMERA_MODE_VIDEO = 1
    
    # Zoom types
    ZOOM_TYPE_STEP = 0
    ZOOM_TYPE_CONTINUOUS = 1
    ZOOM_TYPE_RANGE = 2
    ZOOM_TYPE_FOCAL_LENGTH = 3
    
    # Focus types
    FOCUS_TYPE_STEP = 0
    FOCUS_TYPE_CONTINUOUS = 1
    FOCUS_TYPE_RANGE = 2
    FOCUS_TYPE_METERS = 3
    FOCUS_TYPE_AUTO = 4
    FOCUS_TYPE_AUTO_SINGLE = 5
    FOCUS_TYPE_AUTO_CONTINUOUS = 6
    
    # Default capture settings
    DEFAULT_CAPTURE_INTERVAL = 0  # seconds (0 = single shot)

# ============================================================================
# Gimbal Configuration
# ============================================================================
class GimbalConfig:
    """Gimbal-related configuration"""
    
    # Gimbal angle limits (degrees)
    YAW_MIN = -180
    YAW_MAX = 180
    PITCH_MIN = -90
    PITCH_MAX = 90
    ROLL_MIN = -180
    ROLL_MAX = 180
    
    # Default gimbal speeds (degrees/second)
    DEFAULT_YAW_SPEED = 30
    DEFAULT_PITCH_SPEED = 30
    DEFAULT_ROLL_SPEED = 30

# ============================================================================
# Tracking Configuration
# ============================================================================
class TrackingConfig:
    """Object tracking configuration"""
    
    # Default tracking position (center of 1920x1080 frame)
    DEFAULT_POS_X = 960
    DEFAULT_POS_Y = 540
    
    # Tracking box size limits
    MIN_BOX_SIZE = 10
    MAX_BOX_SIZE = 500

# ============================================================================
# Communication Configuration
# ============================================================================
class CommunicationConfig:
    """Communication and timing configuration"""
    
    # Heartbeat settings
    HEARTBEAT_INTERVAL = 1.0  # seconds
    
    # Message timeouts
    MESSAGE_TIMEOUT = 0.1  # seconds
    PARAM_REQUEST_TIMEOUT = 5.0  # seconds
    
    # Thread sleep intervals
    RECV_THREAD_SLEEP = 0.0001  # seconds
    CONNECTION_CHECK_SLEEP = 1.0  # seconds
    
    # Ping settings
    PING_INITIAL_SEQ = 0

# ============================================================================
# Parameter Configuration
# ============================================================================
class ParameterConfig:
    """Parameter-related configuration"""
    
    # Parameter limits
    MAX_PARAM_ID_LENGTH = 16
    MAX_PARAM_INDEX = 255
    MIN_PARAM_INDEX = 0
    
    # Default message rates (milliseconds, 0 = disabled)
    DEFAULT_PARAM_RATES = {
        "EO_ZOOM": 0,
        "IR_ZOOM": 0,
        "LRF_RANGE": 100,
        "TRK_STATUS": 100,
        "TARGET_LON": 0,
        "TARGET_LAT": 0,
        "TARGET_ALT": 0,
        "PAY_LON": 0,
        "PAY_LAT": 0,
        "PAY_ALT": 0,
        "VIEW_MODE": 0,
        "GB_MODE": 0,
    }

# ============================================================================
# Debug and Logging Configuration
# ============================================================================
class DebugConfig:
    """Debug and logging configuration"""
    
    # Enable/disable debug output
    ENABLE_DEBUG = True
    ENABLE_INFO = True
    ENABLE_WARNING = True
    ENABLE_ERROR = True
    
    # Log message prefixes
    DEBUG_PREFIX = "[DEBUG]"
    INFO_PREFIX = "[INFO]"
    WARNING_PREFIX = "[WARNING]"
    ERROR_PREFIX = "[ERROR]"

# ============================================================================
# Environment Configuration
# ============================================================================
class EnvironmentConfig:
    """Environment and system configuration"""
    
    @staticmethod
    def setup_mavlink_environment():
        """Setup MAVLink environment variables"""
        os.environ['MAVLINK20'] = '1'
        os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

# ============================================================================
# Configuration Validation
# ============================================================================
class ConfigValidator:
    """Validate configuration parameters"""
    
    @staticmethod
    def validate_connection_config() -> bool:
        """Validate connection configuration"""
        if ConnectionConfig.CONTROL_METHOD == ConnectionConfig.CONTROL_UDP:
            if not ConnectionConfig.UDP_IP_TARGET or not ConnectionConfig.UDP_PORT_TARGET:
                print(f"{DebugConfig.ERROR_PREFIX} UDP IP and port must be configured")
                return False
        elif ConnectionConfig.CONTROL_METHOD == ConnectionConfig.CONTROL_UART:
            if not ConnectionConfig.UART_PORT or not ConnectionConfig.UART_BAUDRATE:
                print(f"{DebugConfig.ERROR_PREFIX} UART port and baudrate must be configured")
                return False
        else:
            print(f"{DebugConfig.ERROR_PREFIX} Invalid connection method")
            return False
        return True
    
    @staticmethod
    def validate_gimbal_angles(yaw: float, pitch: float, roll: float) -> bool:
        """Validate gimbal angle limits"""
        if not (GimbalConfig.YAW_MIN <= yaw <= GimbalConfig.YAW_MAX):
            print(f"{DebugConfig.ERROR_PREFIX} Yaw angle {yaw} out of range [{GimbalConfig.YAW_MIN}, {GimbalConfig.YAW_MAX}]")
            return False
        if not (GimbalConfig.PITCH_MIN <= pitch <= GimbalConfig.PITCH_MAX):
            print(f"{DebugConfig.ERROR_PREFIX} Pitch angle {pitch} out of range [{GimbalConfig.PITCH_MIN}, {GimbalConfig.PITCH_MAX}]")
            return False
        if not (GimbalConfig.ROLL_MIN <= roll <= GimbalConfig.ROLL_MAX):
            print(f"{DebugConfig.ERROR_PREFIX} Roll angle {roll} out of range [{GimbalConfig.ROLL_MIN}, {GimbalConfig.ROLL_MAX}]")
            return False
        return True
    
    @staticmethod
    def validate_param_id(param_id: str) -> bool:
        """Validate parameter ID length"""
        if len(param_id) > ParameterConfig.MAX_PARAM_ID_LENGTH:
            print(f"{DebugConfig.ERROR_PREFIX} Parameter ID '{param_id}' exceeds {ParameterConfig.MAX_PARAM_ID_LENGTH} characters")
            return False
        return True
    
    @staticmethod
    def validate_param_index(index: int) -> bool:
        """Validate parameter index range"""
        if not (ParameterConfig.MIN_PARAM_INDEX <= index <= ParameterConfig.MAX_PARAM_INDEX):
            print(f"{DebugConfig.ERROR_PREFIX} Parameter index {index} out of range [{ParameterConfig.MIN_PARAM_INDEX}, {ParameterConfig.MAX_PARAM_INDEX}]")
            return False
        return True

# ============================================================================
# Configuration Manager
# ============================================================================
class ConfigManager:
    """Manage and access all configuration settings"""
    
    def __init__(self):
        self.connection = ConnectionConfig()
        self.mavlink = MAVLinkConfig()
        self.camera = CameraConfig()
        self.gimbal = GimbalConfig()
        self.tracking = TrackingConfig()
        self.communication = CommunicationConfig()
        self.parameter = ParameterConfig()
        self.debug = DebugConfig()
        self.environment = EnvironmentConfig()
        self.validator = ConfigValidator()
    
    def setup_environment(self):
        """Setup environment variables"""
        self.environment.setup_mavlink_environment()
    
    def validate_all(self) -> bool:
        """Validate all configuration settings"""
        return self.validator.validate_connection_config()
    
    def get_connection_string(self) -> str:
        """Get connection string based on configuration"""
        if self.connection.CONTROL_METHOD == self.connection.CONTROL_UDP:
            return f"udpout:{self.connection.UDP_IP_TARGET}:{self.connection.UDP_PORT_TARGET}"
        elif self.connection.CONTROL_METHOD == self.connection.CONTROL_UART:
            return f"{self.connection.UART_PORT}:{self.connection.UART_BAUDRATE}"
        else:
            raise ValueError("Invalid connection method")
    
    def print_config_summary(self):
        """Print configuration summary"""
        print(f"{self.debug.INFO_PREFIX} Gremsy Payload SDK Configuration")
        print(f"{self.debug.INFO_PREFIX} SDK Version: {SDK_VERSION}")
        print(f"{self.debug.INFO_PREFIX} Payload Type: {PAYLOAD_TYPE}")
        print(f"{self.debug.INFO_PREFIX} Connection: {self.get_connection_string()}")
        print(f"{self.debug.INFO_PREFIX} System ID: {self.mavlink.SYS_ID}")
        print(f"{self.debug.INFO_PREFIX} Component ID: {self.mavlink.COMP_ID}")

# ============================================================================
# Global Configuration Instance
# ============================================================================
# Create a global configuration instance
config = ConfigManager() 