#!/usr/bin/env python3
"""
Payload Settings Tab for Payload SDK UI Demo
Based on C++ PayloadSdk PayloadSettingsTab implementation
"""

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gtk, Gdk, GLib, Gst, GstVideo

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'libs'))

from config import ConnectionConfig

# Initialize GStreamer
Gst.init(None)


# UI Command indices (matching C++ enum)
class UICommand:
    # Camera commands
    CAM_CAPTURE = "CAM_CAPTURE"
    CAM_RECORD = "CAM_RECORD"
    CAM_VIEW_MODE = "CAM_VIEW_MODE"
    CAM_SOURCE_RECORD = "CAM_SOURCE_RECORD"
    CAM_ZOOM_CONTINIOUS = "CAM_ZOOM_CONTINIOUS"
    CAM_ZOOM_STEP = "CAM_ZOOM_STEP"
    CAM_ZOOM_RANGE = "CAM_ZOOM_RANGE"
    CAM_ZOOM_SPEED = "CAM_ZOOM_SPEED"
    CAM_FOCUS_CONTINIOUS = "CAM_FOCUS_CONTINIOUS"
    CAM_FOCUS_AUTO = "CAM_FOCUS_AUTO"
    CAM_FOCUS_SPEED = "CAM_FOCUS_SPEED"
    CAM_AE_MODE = "CAM_AE_MODE"
    CAM_SHUTTER = "CAM_SHUTTER"
    CAM_IRIS = "CAM_IRIS"
    CAM_GAIN = "CAM_GAIN"
    CAM_WHITE_BALANCE = "CAM_WHITE_BALANCE"
    CAM_WHITE_BALANCE_TRIGGER = "CAM_WHITE_BALANCE_TRIGGER"
    CAM_IR_PALETTE = "CAM_IR_PALETTE"
    CAM_IR_FFC_MODE = "CAM_IR_FFC_MODE"
    CAM_IR_FFC_TRIGGER = "CAM_IR_FFC_TRIGGER"
    CAM_LRF_MODE = "CAM_LRF_MODE"
    CAM_OSD_MODE = "CAM_OSD_MODE"
    CAM_IMAGE_FLIP = "CAM_IMAGE_FLIP"
    # Gimbal commands
    GIMBAL_MODE = "GIMBAL_MODE"
    GIMBAL_CONTROL_TILT = "GIMBAL_CONTROL_TILT"
    GIMBAL_CONTROL_PAN = "GIMBAL_CONTROL_PAN"
    GIMBAL_CONTROL_ANGLE = "GIMBAL_CONTROL_ANGLE"
    # Payload commands
    PAYLOAD_TOUCH = "PAYLOAD_TOUCH"
    PAYLOAD_TRACK = "PAYLOAD_TRACK"
    PAYLOAD_TRACK_MODE = "PAYLOAD_TRACK_MODE"
    # MB1-specific commands
    CAM_SETTING_TARGET = "CAM_SETTING_TARGET"
    CAM_RC_MODE = "CAM_RC_MODE"
    CAM_STORAGE_TYPE = "CAM_STORAGE_TYPE"
    CAM_EO_SCENE_MODE = "CAM_EO_SCENE_MODE"
    CAM_EO_AE_COMPENSATION = "CAM_EO_AE_COMPENSATION"
    CAM_EO_WHITE_BALANCE = "CAM_EO_WHITE_BALANCE"
    CAM_EO_ISO = "CAM_EO_ISO"
    CAM_EO_SHARPNESS = "CAM_EO_SHARPNESS"
    CAM_IR_GAIN_MODE = "CAM_IR_GAIN_MODE"
    CAM_IR_CONTRAST_MODE = "CAM_IR_CONTRAST_MODE"
    CAM_IR_AGC_MODE = "CAM_IR_AGC_MODE"
    CAM_IR_AGC_LINEAR_PERCENT = "CAM_IR_AGC_LINEAR_PERCENT"
    CAM_IR_SPOTMETER_MODE = "CAM_IR_SPOTMETER_MODE"
    CAM_IR_SPOTMETER_UNITS = "CAM_IR_SPOTMETER_UNITS"
    CAM_IR_SPOTMETER_SIZE = "CAM_IR_SPOTMETER_SIZE"
    CAM_IR_ISOTHERM_MODE = "CAM_IR_ISOTHERM_MODE"
    CAM_IR_ISOTHERM_UNITS = "CAM_IR_ISOTHERM_UNITS"
    CAM_IR_ISOTHERM_THRESHOLD = "CAM_IR_ISOTHERM_THRESHOLD"
    CAM_GIMBAL_FW_FLAG = "CAM_GIMBAL_FW_FLAG"
    CAM_OBJECT_DETECTION = "CAM_OBJECT_DETECTION"
    CAM_IR_ISOTHERMS_GAIN = "CAM_IR_ISOTHERMS_GAIN"


# ============================================================================
# UI OPTION LISTS
# These lists define (label, value) pairs for UI combo boxes
# Matching C++ PayloadSettingsTab.h structure
# ============================================================================

# Camera View Mode options
UI_CAM_VIEW_LIST_EOIR =                                                 0
UI_CAM_VIEW_LIST_EO =                                                   1
UI_CAM_VIEW_LIST_IR =                                                   2
UI_CAM_VIEW_LIST_IREO =                                                 3
UI_CAM_VIEW_LIST = [
    ("EO/IR",           UI_CAM_VIEW_LIST_EOIR),
    ("EO",              UI_CAM_VIEW_LIST_EO),
    ("IR",              UI_CAM_VIEW_LIST_IR),
    ("IR/EO",           UI_CAM_VIEW_LIST_IREO),
]

# Camera Record Source options
UI_CAM_RECORD_SRC_LIST_EO =                                             1
UI_CAM_RECORD_SRC_LIST_IR =                                             2
UI_CAM_RECORD_SRC_LIST_BOTH =                                           0
UI_CAM_RECORD_SRC_LIST_OSD =                                            5
UI_CAM_RECORD_SRC_LIST = [
    ("EO",              UI_CAM_RECORD_SRC_LIST_EO),
    ("IR",              UI_CAM_RECORD_SRC_LIST_IR),
    ("BOTH",            UI_CAM_RECORD_SRC_LIST_BOTH),
    ("OSD",             UI_CAM_RECORD_SRC_LIST_OSD),
]

# Gimbal Mode options
UI_GIMBAL_MODE_LIST_OFF =                                               0
UI_GIMBAL_MODE_LIST_LOCK =                                              1
UI_GIMBAL_MODE_LIST_FOLLOW =                                            2
UI_GIMBAL_MODE_LIST_MAPPING =                                           3
UI_GIMBAL_MODE_LIST = [
    ("OFF",             UI_GIMBAL_MODE_LIST_OFF),
    ("LOCK",            UI_GIMBAL_MODE_LIST_LOCK),
    ("FOLLOW",          UI_GIMBAL_MODE_LIST_FOLLOW),
    ("MAPPING",         UI_GIMBAL_MODE_LIST_MAPPING),
]

# OSD Mode options
UI_OSD_MODE_LIST_DISABLE =                                              0
UI_OSD_MODE_LIST_DEBUG =                                                1
UI_OSD_MODE_LIST_STATUS =                                               2
UI_OSD_MODE_LIST = [
    ("Disable",         UI_OSD_MODE_LIST_DISABLE),
    ("Debug",           UI_OSD_MODE_LIST_DEBUG),
    ("Status",          UI_OSD_MODE_LIST_STATUS),
]

# Image Flip options
UI_IMAGE_FLIP_LIST_OFF =                                                3
UI_IMAGE_FLIP_LIST_ON =                                                 2
UI_IMAGE_FLIP_LIST = [
    ("OFF",             UI_IMAGE_FLIP_LIST_OFF),
    ("ON",              UI_IMAGE_FLIP_LIST_ON),
]

# IR Palette options
UI_IR_PALETTE_LIST_1 =                                                  0
UI_IR_PALETTE_LIST_2 =                                                  1
UI_IR_PALETTE_LIST_3 =                                                  2
UI_IR_PALETTE_LIST_4 =                                                  3
UI_IR_PALETTE_LIST_5 =                                                  4
UI_IR_PALETTE_LIST_6 =                                                  5
UI_IR_PALETTE_LIST_7 =                                                  6
UI_IR_PALETTE_LIST_8 =                                                  7
UI_IR_PALETTE_LIST_9 =                                                  8
UI_IR_PALETTE_LIST_10 =                                                 9
UI_IR_PALETTE_LIST = [
    ("Palette 1",       UI_IR_PALETTE_LIST_1),
    ("Palette 2",       UI_IR_PALETTE_LIST_2),
    ("Palette 3",       UI_IR_PALETTE_LIST_3),
    ("Palette 4",       UI_IR_PALETTE_LIST_4),
    ("Palette 5",       UI_IR_PALETTE_LIST_5),
    ("Palette 6",       UI_IR_PALETTE_LIST_6),
    ("Palette 7",       UI_IR_PALETTE_LIST_7),
    ("Palette 8",       UI_IR_PALETTE_LIST_8),
    ("Palette 9",       UI_IR_PALETTE_LIST_9),
    ("Palette 10",      UI_IR_PALETTE_LIST_10),
]

# Tracking Mode options
UI_TRACK_MODE_LIST_OBJ_TRACKING =                                       0
UI_TRACK_MODE_LIST_OBJ_DETECTION =                                      1
UI_TRACK_MODE_LIST = [
    ("Tracking",        UI_TRACK_MODE_LIST_OBJ_TRACKING),
    ("Detection",       UI_TRACK_MODE_LIST_OBJ_DETECTION),
]

# ============================================================================
# MB1-SPECIFIC UI OPTION LISTS
# ============================================================================

# Setting Target options
UI_MB1_SETTING_TARGET_LIST_EO =                                         0
UI_MB1_SETTING_TARGET_LIST_IR =                                         1
UI_MB1_SETTING_TARGET_LIST_GIMBAL =                                     2
UI_MB1_SETTING_TARGET_LIST = [
    ("EO Camera",       UI_MB1_SETTING_TARGET_LIST_EO),
    ("IR Camera",       UI_MB1_SETTING_TARGET_LIST_IR),
    ("Gimbal Device",   UI_MB1_SETTING_TARGET_LIST_GIMBAL),
]

# RC Mode options
UI_MB1_RC_MODE_LIST_GREMSY =                                            0
UI_MB1_RC_MODE_LIST_STANDARD =                                          1
UI_MB1_RC_MODE_LIST = [
    ("Gremsy",          UI_MB1_RC_MODE_LIST_GREMSY),
    ("Standard",        UI_MB1_RC_MODE_LIST_STANDARD),
]

# Storage Type options
UI_MB1_STORAGE_TYPE_LIST_INTERNAL =                                     0
UI_MB1_STORAGE_TYPE_LIST_SDCARD =                                       1
UI_MB1_STORAGE_TYPE_LIST = [
    ("Internal",        UI_MB1_STORAGE_TYPE_LIST_INTERNAL),
    ("SD Card",         UI_MB1_STORAGE_TYPE_LIST_SDCARD),
]

# EO Scene Mode options
UI_MB1_EO_SCENE_MODE_LIST_DISABLED =                                    0
UI_MB1_EO_SCENE_MODE_LIST_FACE_PRIORITY =                               1
UI_MB1_EO_SCENE_MODE_LIST_ACTION =                                      2
UI_MB1_EO_SCENE_MODE_LIST_PORTRAIT =                                    3
UI_MB1_EO_SCENE_MODE_LIST_LANDSCAPE =                                   4
UI_MB1_EO_SCENE_MODE_LIST_NIGHT =                                       5
UI_MB1_EO_SCENE_MODE_LIST_NIGHT_PORTRAIT =                              6
UI_MB1_EO_SCENE_MODE_LIST_THEATRE =                                     7
UI_MB1_EO_SCENE_MODE_LIST_BEACH =                                       8
UI_MB1_EO_SCENE_MODE_LIST_SNOW =                                        9
UI_MB1_EO_SCENE_MODE_LIST_SUNSET =                                      10
UI_MB1_EO_SCENE_MODE_LIST_STEADY_PHOTO =                                11
UI_MB1_EO_SCENE_MODE_LIST_FIREWORKS =                                   12
UI_MB1_EO_SCENE_MODE_LIST_SPORTS =                                      13
UI_MB1_EO_SCENE_MODE_LIST_PARTY =                                       14
UI_MB1_EO_SCENE_MODE_LIST_CANDLELIGHT =                                 15
UI_MB1_EO_SCENE_MODE_LIST_HDR =                                         16
UI_MB1_EO_SCENE_MODE_LIST = [
    ("Disabled",        UI_MB1_EO_SCENE_MODE_LIST_DISABLED),
    ("Face-priority",   UI_MB1_EO_SCENE_MODE_LIST_FACE_PRIORITY),
    ("Action",          UI_MB1_EO_SCENE_MODE_LIST_ACTION),
    ("Portrait",        UI_MB1_EO_SCENE_MODE_LIST_PORTRAIT),
    ("Landscape",       UI_MB1_EO_SCENE_MODE_LIST_LANDSCAPE),
    ("Night",           UI_MB1_EO_SCENE_MODE_LIST_NIGHT),
    ("Night-portrait",  UI_MB1_EO_SCENE_MODE_LIST_NIGHT_PORTRAIT),
    ("Theatre",         UI_MB1_EO_SCENE_MODE_LIST_THEATRE),
    ("Beach",           UI_MB1_EO_SCENE_MODE_LIST_BEACH),
    ("Snow",            UI_MB1_EO_SCENE_MODE_LIST_SNOW),
    ("Sunset",          UI_MB1_EO_SCENE_MODE_LIST_SUNSET),
    ("Steady-photo",    UI_MB1_EO_SCENE_MODE_LIST_STEADY_PHOTO),
    ("Fireworks",       UI_MB1_EO_SCENE_MODE_LIST_FIREWORKS),
    ("Sports",          UI_MB1_EO_SCENE_MODE_LIST_SPORTS),
    ("Party",           UI_MB1_EO_SCENE_MODE_LIST_PARTY),
    ("Candlelight",     UI_MB1_EO_SCENE_MODE_LIST_CANDLELIGHT),
    ("HDR",             UI_MB1_EO_SCENE_MODE_LIST_HDR),
]

# EO White Balance options
UI_MB1_EO_WB_LIST_OFF =                                                 0
UI_MB1_EO_WB_LIST_MANUAL_CC_TEMP =                                      1
UI_MB1_EO_WB_LIST_MANUAL_RGB_GAINS =                                    2
UI_MB1_EO_WB_LIST_AUTO =                                                3
UI_MB1_EO_WB_LIST_SHADE =                                               4
UI_MB1_EO_WB_LIST_INCANDESCENT =                                        5
UI_MB1_EO_WB_LIST_FLUORESCENT =                                         6
UI_MB1_EO_WB_LIST_WARM_FLUORESCENT =                                    7
UI_MB1_EO_WB_LIST_DAYLIGHT =                                            8
UI_MB1_EO_WB_LIST_CLOUDY_DAYLIGHT =                                     9
UI_MB1_EO_WB_LIST_TWILIGHT =                                            10
UI_MB1_EO_WB_LIST = [
    ("Off",             UI_MB1_EO_WB_LIST_OFF),
    ("Manual CC Temp",  UI_MB1_EO_WB_LIST_MANUAL_CC_TEMP),
    ("Manual RGB Gains",UI_MB1_EO_WB_LIST_MANUAL_RGB_GAINS),
    ("Auto",            UI_MB1_EO_WB_LIST_AUTO),
    ("Shade",           UI_MB1_EO_WB_LIST_SHADE),
    ("Incandescent",    UI_MB1_EO_WB_LIST_INCANDESCENT),
    ("Fluorescent",     UI_MB1_EO_WB_LIST_FLUORESCENT),
    ("Warm Fluorescent",UI_MB1_EO_WB_LIST_WARM_FLUORESCENT),
    ("Daylight",        UI_MB1_EO_WB_LIST_DAYLIGHT),
    ("Cloudy Daylight", UI_MB1_EO_WB_LIST_CLOUDY_DAYLIGHT),
    ("Twilight",        UI_MB1_EO_WB_LIST_TWILIGHT),
]

# EO ISO options
UI_MB1_EO_ISO_LIST_AUTO =                                               0
UI_MB1_EO_ISO_LIST_DEBLUR =                                             1
UI_MB1_EO_ISO_LIST_100 =                                                2
UI_MB1_EO_ISO_LIST_200 =                                                3
UI_MB1_EO_ISO_LIST_400 =                                                4
UI_MB1_EO_ISO_LIST_800 =                                                5
UI_MB1_EO_ISO_LIST_1600 =                                               6
UI_MB1_EO_ISO_LIST_3200 =                                               7
UI_MB1_EO_ISO_LIST = [
    ("Auto",            UI_MB1_EO_ISO_LIST_AUTO),
    ("Deblur",          UI_MB1_EO_ISO_LIST_DEBLUR),
    ("100",             UI_MB1_EO_ISO_LIST_100),
    ("200",             UI_MB1_EO_ISO_LIST_200),
    ("400",             UI_MB1_EO_ISO_LIST_400),
    ("800",             UI_MB1_EO_ISO_LIST_800),
    ("1600",            UI_MB1_EO_ISO_LIST_1600),
    ("3200",            UI_MB1_EO_ISO_LIST_3200),
]

# IR Gain Mode options
UI_MB1_IR_GAIN_MODE_LIST_LOW =                                          0
UI_MB1_IR_GAIN_MODE_LIST_HIGH =                                         1
UI_MB1_IR_GAIN_MODE_LIST = [
    ("Low (-50~150C)",  UI_MB1_IR_GAIN_MODE_LIST_LOW),
    ("High (-50~550C)", UI_MB1_IR_GAIN_MODE_LIST_HIGH),
]

# IR Contrast Mode options
UI_MB1_IR_CONTRAST_MODE_LIST_DEFAULT =                                  0
UI_MB1_IR_CONTRAST_MODE_LIST_CUSTOM =                                   1
UI_MB1_IR_CONTRAST_MODE_LIST = [
    ("Default",         UI_MB1_IR_CONTRAST_MODE_LIST_DEFAULT),
    ("Custom",          UI_MB1_IR_CONTRAST_MODE_LIST_CUSTOM),
]

# IR AGC Mode options
UI_MB1_IR_AGC_MODE_LIST_NORMAL =                                        0
UI_MB1_IR_AGC_MODE_LIST_HOLD =                                          1
UI_MB1_IR_AGC_MODE_LIST_THRESHOLD =                                     2
UI_MB1_IR_AGC_MODE_LIST_BRIGHT =                                        3
UI_MB1_IR_AGC_MODE_LIST_LINEAR =                                        4
UI_MB1_IR_AGC_MODE_LIST_MANUAL =                                        5
UI_MB1_IR_AGC_MODE_LIST = [
    ("Normal",          UI_MB1_IR_AGC_MODE_LIST_NORMAL),
    ("Hold",            UI_MB1_IR_AGC_MODE_LIST_HOLD),
    ("Threshold",       UI_MB1_IR_AGC_MODE_LIST_THRESHOLD),
    ("Bright",          UI_MB1_IR_AGC_MODE_LIST_BRIGHT),
    ("Linear",          UI_MB1_IR_AGC_MODE_LIST_LINEAR),
    ("Manual",          UI_MB1_IR_AGC_MODE_LIST_MANUAL),
]

# IR SpotMeter Mode options
UI_MB1_IR_SPOTMETER_MODE_LIST_DISABLE =                                 0
UI_MB1_IR_SPOTMETER_MODE_LIST_ENABLE =                                  1
UI_MB1_IR_SPOTMETER_MODE_LIST = [
    ("Disable",         UI_MB1_IR_SPOTMETER_MODE_LIST_DISABLE),
    ("Enable",          UI_MB1_IR_SPOTMETER_MODE_LIST_ENABLE),
]

# IR SpotMeter Units options
UI_MB1_IR_SPOTMETER_UNITS_LIST_CELSIUS =                                0
UI_MB1_IR_SPOTMETER_UNITS_LIST_FAHRENHEIT =                             1
UI_MB1_IR_SPOTMETER_UNITS_LIST_KELVIN =                                 2
UI_MB1_IR_SPOTMETER_UNITS_LIST = [
    ("Celsius",         UI_MB1_IR_SPOTMETER_UNITS_LIST_CELSIUS),
    ("Fahrenheit",      UI_MB1_IR_SPOTMETER_UNITS_LIST_FAHRENHEIT),
    ("Kelvin",          UI_MB1_IR_SPOTMETER_UNITS_LIST_KELVIN),
]

# IR Isotherm Mode options
UI_MB1_IR_ISOTHERM_MODE_LIST_DISABLE =                                  0
UI_MB1_IR_ISOTHERM_MODE_LIST_ENABLE =                                   1
UI_MB1_IR_ISOTHERM_MODE_LIST = [
    ("Disable",         UI_MB1_IR_ISOTHERM_MODE_LIST_DISABLE),
    ("Enable",          UI_MB1_IR_ISOTHERM_MODE_LIST_ENABLE),
]

# IR Isotherm Units options
UI_MB1_IR_ISOTHERM_UNITS_LIST_KELVIN =                                  0
UI_MB1_IR_ISOTHERM_UNITS_LIST_CELSIUS =                                 1
UI_MB1_IR_ISOTHERM_UNITS_LIST_FAHRENHEIT =                              2
UI_MB1_IR_ISOTHERM_UNITS_LIST_PERCENT =                                 3
UI_MB1_IR_ISOTHERM_UNITS_LIST_COUNTS =                                  4
UI_MB1_IR_ISOTHERM_UNITS_LIST = [
    ("Kelvin",          UI_MB1_IR_ISOTHERM_UNITS_LIST_KELVIN),
    ("Celsius",         UI_MB1_IR_ISOTHERM_UNITS_LIST_CELSIUS),
    ("Fahrenheit",      UI_MB1_IR_ISOTHERM_UNITS_LIST_FAHRENHEIT),
    ("Percent",         UI_MB1_IR_ISOTHERM_UNITS_LIST_PERCENT),
    ("Counts (Raw)",    UI_MB1_IR_ISOTHERM_UNITS_LIST_COUNTS),
]

# Object Detection options
UI_MB1_OBJECT_DETECTION_LIST_DISABLE =                                  0
UI_MB1_OBJECT_DETECTION_LIST_ENABLE =                                   1
UI_MB1_OBJECT_DETECTION_LIST = [
    ("Disable",         UI_MB1_OBJECT_DETECTION_LIST_DISABLE),
    ("Enable",          UI_MB1_OBJECT_DETECTION_LIST_ENABLE),
]

# IR Isotherms Gain options
UI_MB1_IR_ISOTHERMS_GAIN_LIST_HIGH_GAIN =                               0
UI_MB1_IR_ISOTHERMS_GAIN_LIST_LOW_GAIN =                                1
UI_MB1_IR_ISOTHERMS_GAIN_LIST = [
    ("High Gain",       UI_MB1_IR_ISOTHERMS_GAIN_LIST_HIGH_GAIN),
    ("Low Gain",        UI_MB1_IR_ISOTHERMS_GAIN_LIST_LOW_GAIN),
]

# Gimbal Forward Flag options
UI_MB1_GIMBAL_FW_FLAG_LIST_OVERWRITE =                                  0
UI_MB1_GIMBAL_FW_FLAG_LIST_FORWARD =                                    1
UI_MB1_GIMBAL_FW_FLAG_LIST = [
    ("Overwrite",       UI_MB1_GIMBAL_FW_FLAG_LIST_OVERWRITE),
    ("Forward",         UI_MB1_GIMBAL_FW_FLAG_LIST_FORWARD),
]

# ============================================================================
# NON-MB1 OPTION LISTS (for other payloads)
# ============================================================================
AE_MODE_LIST = ["Auto", "Manual", "Shutter Priority", "Aperture Priority"]
SHUTTER_MODE_LIST = ["1/30", "1/60", "1/125", "1/250", "1/500", "1/1000", "1/2000", "1/4000", "1/8000"]
IRIS_MODE_LIST = ["F1.4", "F2", "F2.8", "F4", "F5.6", "F8", "F11", "F16"]
GAIN_MODE_LIST = ["0dB", "3dB", "6dB", "9dB", "12dB", "15dB", "18dB", "21dB", "24dB"]
WHITE_BALANCE_LIST = ["Auto", "Indoor", "Outdoor", "One Push", "Manual"]
FFC_MODE_LIST = ["Manual", "Auto"]
LRF_MODE_LIST = ["Off", "Single", "Continuous"]


class PayloadSettingsTab(Gtk.Box):
    """Payload settings tab with all controls"""

    def __init__(self, is_mb1=False):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        self.set_hexpand(True)

        # Payload type flag
        self.is_mb1 = is_mb1

        # Callback for button clicks
        self._button_clicked_callback = None

        # State variables
        self.rec_status = 0
        self.speed_gimbal = 20.0
        self.is_playing = False
        self.is_fullscreen = False
        self.is_touch = False
        self.pipeline = None
        self.video_window_handle = 0
        self.fullscreen_window = None
        self.fullscreen_video_area = None
        self.fullscreen_video_handle = 0

        # Info labels
        self.gimbal_mode_info = None
        self.pitch_angle_info = None
        self.roll_angle_info = None
        self.yaw_angle_info = None
        self.view_mode_info = None
        self.record_src_info = None
        self.eo_zoom_level_info = None
        self.ir_zoom_level_info = None
        self.ir_type_info = None
        self.ir_palette_info = None
        self.ir_ffc_mode_info = None
        self.ir_temp_max_info = None
        self.ir_temp_min_info = None
        self.ir_temp_mean_info = None
        self.lrf_offset_x_info = None
        self.lrf_offset_y_info = None
        self.lrf_range_info = None
        self.target_gps_lon_info = None
        self.target_gps_lat_info = None
        self.target_gps_alt_info = None
        self.payload_gps_lon_info = None
        self.payload_gps_lat_info = None
        self.payload_gps_alt_info = None
        self.storage_info = None
        self.capture_info = None
        self.record_info = None

        # Control widgets
        self.url_entry = None
        self.play_button = None
        self.stop_button = None
        self.fullscreen_button = None
        self.video_area = None
        self.touch_button = None
        self.track_button = None

        # Combo boxes
        self.view_mode_combo = None
        self.rec_src_combo = None
        self.ae_mode_combo = None
        self.shutter_combo = None
        self.iris_combo = None
        self.gain_combo = None
        self.wb_mode_combo = None
        self.ir_palette_combo = None
        self.ffc_mode_combo = None
        self.lrf_mode_combo = None
        self.osd_mode_combo = None
        self.image_flip_combo = None
        self.gimbal_mode_combo = None
        self.track_mode_combo = None

        # MB1-specific combo boxes
        self.setting_target_combo = None
        self.rc_mode_combo = None
        self.storage_type_combo = None
        self.eo_scene_mode_combo = None
        self.eo_wb_combo = None
        self.eo_iso_combo = None
        self.ir_gain_mode_combo = None
        self.ir_contrast_mode_combo = None
        self.ir_agc_mode_combo = None
        self.ir_spotmeter_mode_combo = None
        self.ir_spotmeter_units_combo = None
        self.ir_isotherm_mode_combo = None
        self.ir_isotherm_units_combo = None
        self.object_detection_combo = None
        self.ir_isotherms_gain_combo = None
        self.gimbal_fw_flag_combo = None

        # Sliders
        self.zoom_range = None
        self.eo_zoom_speed_range = None
        self.eo_focus_speed_range = None
        self.speed_gimbal_range = None
        self.pitch_angle_gimbal_range = None
        self.roll_angle_gimbal_range = None
        self.yaw_angle_gimbal_range = None

        # MB1-specific sliders
        self.eo_ae_compensation_range = None
        self.eo_sharpness_range = None
        self.ir_agc_linear_percent_range = None
        self.ir_spotmeter_size_range = None
        self.ir_isotherm_threshold_range = None

        # Create main layout
        self.pack_start(self._create_main_tab(), True, True, 0)

    def _create_main_tab(self):
        """Create main tab content"""
        frame = Gtk.Frame()
        frame.set_halign(Gtk.Align.FILL)
        frame.set_valign(Gtk.Align.START)
        frame.set_hexpand(True)
        frame.set_margin_start(10)
        frame.set_margin_end(10)

        mainbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        mainbox.set_margin_top(0)
        mainbox.set_margin_bottom(10)
        mainbox.set_margin_start(10)
        mainbox.set_margin_end(10)
        mainbox.set_hexpand(True)

        # Left column - Video and Payload Settings
        box_1 = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box_1.set_valign(Gtk.Align.START)
        box_1.pack_start(self._create_video_interface(), False, False, 0)
        box_1.pack_start(self._create_payload_setting_main_group(), False, False, 0)

        # Right columns - Camera, Gimbal, Info
        box_2 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        box_2.set_valign(Gtk.Align.START)
        box_2.pack_start(self._create_camera_setting_main_group(), False, False, 0)
        box_2.pack_start(self._create_gimbal_setting_main_group(), False, False, 0)
        box_2.pack_start(self._create_info_show_main_group(), False, False, 0)

        mainbox.pack_start(box_1, False, False, 0)
        mainbox.pack_start(box_2, True, True, 0)

        frame.add(mainbox)
        return frame

    # ===== Video Interface =====
    def _create_video_interface(self):
        """Create video interface with RTSP streaming"""
        frame = Gtk.Frame()
        frame.set_halign(Gtk.Align.FILL)
        frame.set_valign(Gtk.Align.START)
        frame.set_hexpand(True)
        frame.set_shadow_type(Gtk.ShadowType.NONE)

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        box.set_margin_top(0)
        box.set_margin_bottom(0)
        box.set_margin_start(10)
        box.set_margin_end(10)

        # URL input section
        url_frame = Gtk.Frame(label="RTSP URL")
        url_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        url_box.set_margin_top(5)
        url_box.set_margin_bottom(5)
        url_box.set_margin_start(10)
        url_box.set_margin_end(10)

        # URL entry
        self.url_entry = Gtk.Entry()
        self.url_entry.set_placeholder_text("rtsp://example.com:554/stream")
        # Default RTSP URL from config IP
        default_rtsp_url = f"rtsp://{ConnectionConfig.UDP_IP_TARGET}:8554/payload"
        self.url_entry.set_text(default_rtsp_url)
        self.url_entry.connect("activate", self._on_url_entry_activate)
        url_box.pack_start(self.url_entry, True, True, 0)

        # Control buttons
        self.play_button = Gtk.Button(label="Play")
        self.stop_button = Gtk.Button(label="Stop")
        self.fullscreen_button = Gtk.Button(label="Fullscreen")

        self.play_button.connect("clicked", self._on_play_button_clicked)
        self.stop_button.connect("clicked", self._on_stop_button_clicked)
        self.fullscreen_button.connect("clicked", self._on_fullscreen_clicked)

        self.stop_button.set_sensitive(False)
        self.fullscreen_button.set_sensitive(False)

        url_box.pack_start(self.play_button, False, False, 0)
        url_box.pack_start(self.stop_button, False, False, 0)
        url_box.pack_start(self.fullscreen_button, False, False, 0)

        url_frame.add(url_box)
        box.pack_start(url_frame, False, False, 0)

        # Video display area
        video_frame = Gtk.Frame()
        self.video_area = Gtk.DrawingArea()
        self.video_area.set_size_request(640, 360)
        self.video_area.set_double_buffered(False)
        self.video_area.set_vexpand(True)
        self.video_area.set_hexpand(True)
        self.video_area.connect("draw", self._on_video_area_draw)
        self.video_area.connect("realize", self._on_video_area_realize)
        self.video_area.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        self.video_area.connect("button-press-event", self._on_video_area_clicked)

        video_frame.add(self.video_area)
        box.pack_start(video_frame, True, True, 0)

        # Touch/Track buttons
        hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        hbox.set_margin_top(10)
        hbox.set_margin_bottom(10)
        hbox.set_margin_start(10)
        hbox.set_margin_end(10)

        self.touch_button = Gtk.ToggleButton(label="Touch")
        self.track_button = Gtk.ToggleButton(label="Track")

        # CSS styling
        css_provider = Gtk.CssProvider()
        css_provider.load_from_data(b"""
            .toggle-off {
                background: #444;
                color: white;
                border-radius: 6px;
            }
            .toggle-on {
                background: #3a9f3a;
                color: white;
                border-radius: 6px;
            }
        """)

        for btn in [self.touch_button, self.track_button]:
            context = btn.get_style_context()
            context.add_provider(css_provider, Gtk.STYLE_PROVIDER_PRIORITY_USER)
            context.add_class("toggle-off")

        self.touch_button.connect("toggled", self._on_touch_toggled)
        self.track_button.connect("toggled", self._on_track_toggled)

        hbox.pack_start(self.touch_button, True, True, 0)
        hbox.pack_start(self.track_button, True, True, 0)
        hbox.pack_start(self._create_combo_box("track_mode_combo", "", UI_TRACK_MODE_LIST, UICommand.PAYLOAD_TRACK_MODE), True, True, 0)

        box.pack_start(hbox, True, True, 0)

        frame.add(box)
        return frame

    # ===== Payload Setting Main Group =====
    def _create_payload_setting_main_group(self):
        """Create payload settings group"""
        frame = Gtk.Frame(label="Payload Settings")
        frame.set_halign(Gtk.Align.START)
        frame.set_valign(Gtk.Align.START)
        frame.set_margin_top(0)

        mainbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        mainbox.set_margin_top(0)
        mainbox.set_margin_bottom(10)
        mainbox.set_margin_start(10)
        mainbox.set_margin_end(10)

        box_1 = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        box_1.set_margin_top(0)
        box_1.set_margin_bottom(10)
        box_1.set_margin_start(10)
        box_1.set_margin_end(10)

        # Camera View & Record
        cam_frame = Gtk.Frame(label="Camera View & Record")
        hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        hbox.set_margin_top(10)
        hbox.set_margin_bottom(10)
        hbox.set_margin_start(10)
        hbox.set_margin_end(10)

        hbox.pack_start(self._create_combo_box("view_mode_combo", "View", UI_CAM_VIEW_LIST, UICommand.CAM_VIEW_MODE), True, True, 0)
        hbox.pack_start(self._create_combo_box("rec_src_combo", "Record", UI_CAM_RECORD_SRC_LIST, UICommand.CAM_SOURCE_RECORD), True, True, 0)

        cam_frame.add(hbox)
        box_1.pack_start(cam_frame, False, False, 0)
        box_1.pack_start(self._create_capture_record_group(), False, False, 0)

        box_2 = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        box_2.set_margin_top(10)
        box_2.set_margin_bottom(10)
        box_2.set_margin_start(10)
        box_2.set_margin_end(10)

        # Non-MB1 payloads have LRF mode
        if not self.is_mb1:
            box_2.pack_start(self._create_lrf_mode_group(), False, False, 0)
        box_2.pack_start(self._create_osd_mode_group(), False, False, 0)
        box_2.pack_start(self._create_image_flip_group(), False, False, 0)

        # MB1-specific general settings
        if self.is_mb1:
            box_2.pack_start(self._create_mb1_general_settings_group(), False, False, 0)

        mainbox.pack_start(box_1, False, False, 0)
        mainbox.pack_start(box_2, True, True, 0)

        frame.add(mainbox)
        return frame

    # ===== Camera Setting Main Group =====
    def _create_camera_setting_main_group(self):
        """Create camera settings group"""
        frame = Gtk.Frame(label="Camera Setting")
        frame.set_halign(Gtk.Align.START)
        frame.set_valign(Gtk.Align.START)

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        box.set_margin_top(0)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_zoom_controls_group(), False, False, 0)

        # Non-MB1 payloads have focus controls, exposure, and white balance
        if not self.is_mb1:
            box.pack_start(self._create_focus_controls_group(), False, False, 0)
            box.pack_start(self._create_exposure_group(), False, False, 0)
            box.pack_start(self._create_white_balance_group(), False, False, 0)

        # MB1-specific camera controls
        if self.is_mb1:
            box.pack_start(self._create_mb1_eo_advanced_group(), False, False, 0)
            box.pack_start(self._create_mb1_ir_advanced_group(), False, False, 0)
            box.pack_start(self._create_mb1_ir_spotmeter_group(), False, False, 0)
            box.pack_start(self._create_mb1_ir_isotherm_group(), False, False, 0)

        box.pack_start(self._create_ir_palette_group(), False, False, 0)

        frame.add(box)
        return frame

    # ===== Gimbal Setting Main Group =====
    def _create_gimbal_setting_main_group(self):
        """Create gimbal settings group"""
        frame = Gtk.Frame(label="Gimbal Setting")
        frame.set_halign(Gtk.Align.START)
        frame.set_valign(Gtk.Align.START)

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        box.set_margin_top(0)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_combo_box("gimbal_mode_combo", "Gimbal Mode", UI_GIMBAL_MODE_LIST, UICommand.GIMBAL_MODE), False, False, 0)
        box.pack_start(self._create_gimbal_control_speed_group(), False, False, 0)
        box.pack_start(self._create_gimbal_control_angle_group(), False, False, 0)

        frame.add(box)
        return frame

    # ===== Info Main Group =====
    def _create_info_show_main_group(self):
        """Create info display group"""
        frame = Gtk.Frame(label="Payload Info")
        frame.set_halign(Gtk.Align.END)
        frame.set_valign(Gtk.Align.START)

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        box.set_margin_top(0)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        self.gimbal_mode_info = self._create_info_row(box, "Gimbal Mode")
        self.pitch_angle_info = self._create_info_row(box, "Pitch")
        self.roll_angle_info = self._create_info_row(box, "Roll")
        self.yaw_angle_info = self._create_info_row(box, "Yaw")

        self.view_mode_info = self._create_info_row(box, "View Mode")
        self.record_src_info = self._create_info_row(box, "Record Source")

        self.eo_zoom_level_info = self._create_info_row(box, "EO Zoom Level")
        self.ir_zoom_level_info = self._create_info_row(box, "IR Zoom Level")

        self.ir_type_info = self._create_info_row(box, "IR Type")
        self.ir_palette_info = self._create_info_row(box, "IR Palette ID")
        self.ir_ffc_mode_info = self._create_info_row(box, "IR FFC Mode")
        self.ir_temp_max_info = self._create_info_row(box, "IR Temp Max")
        self.ir_temp_min_info = self._create_info_row(box, "IR Temp Min")
        self.ir_temp_mean_info = self._create_info_row(box, "IR Temp Mean")

        self.lrf_offset_x_info = self._create_info_row(box, "LRF OFFSET X")
        self.lrf_offset_y_info = self._create_info_row(box, "LRF OFFSET Y")
        self.lrf_range_info = self._create_info_row(box, "LRF Range")
        self.target_gps_lon_info = self._create_info_row(box, "Target GPS LON")
        self.target_gps_lat_info = self._create_info_row(box, "Target GPS LAT")
        self.target_gps_alt_info = self._create_info_row(box, "Target GPS ALT")

        self.payload_gps_lon_info = self._create_info_row(box, "Payload GPS LON")
        self.payload_gps_lat_info = self._create_info_row(box, "Payload GPS LAT")
        self.payload_gps_alt_info = self._create_info_row(box, "Payload GPS ALT")

        frame.add(box)
        return frame

    # ===== Capture/Record Group =====
    def _create_capture_record_group(self):
        """Create capture/record controls"""
        frame = Gtk.Frame(label="Capture / Record")

        main_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        main_box.set_margin_top(0)
        main_box.set_margin_bottom(10)
        main_box.set_margin_start(10)
        main_box.set_margin_end(10)

        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        # Capture group
        capture_group = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        btn_capture = Gtk.Button(label="Capture")
        btn_capture.connect("clicked", lambda b: self._on_button_clicked(UICommand.CAM_CAPTURE, [0.0]))
        capture_group.pack_start(btn_capture, True, True, 0)
        self.capture_info = Gtk.Label(label="0")
        capture_group.pack_start(self.capture_info, True, True, 0)

        # Record group
        record_group = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        btn_record = Gtk.Button(label="Record")
        btn_record.connect("clicked", lambda b: self._on_button_clicked(UICommand.CAM_RECORD, [float(self.rec_status)]))
        record_group.pack_start(btn_record, True, True, 0)
        self.record_info = Gtk.Label(label="00:00:00")
        record_group.pack_start(self.record_info, True, True, 0)

        box.pack_start(capture_group, True, True, 0)
        box.pack_start(record_group, True, True, 0)

        main_box.pack_start(box, True, True, 0)

        self.storage_info = Gtk.Label(label="No SD Card")
        main_box.pack_start(self.storage_info, True, True, 0)

        frame.add(main_box)
        return frame

    # ===== Zoom Controls Group =====
    def _create_zoom_controls_group(self):
        """Create zoom controls"""
        frame = Gtk.Frame(label="Zoom")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        # EO Zoom Speed slider
        self._create_slider_row(box, "EO Zoom Speed", 0, 7, 3, "eo_zoom_speed_range", UICommand.CAM_ZOOM_SPEED)

        # Continuous zoom
        cont_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        cont_box.pack_start(Gtk.Label(label="Continuous"), False, False, 0)
        self._add_button_to_box(cont_box, "Zoom In", UICommand.CAM_ZOOM_CONTINIOUS, 1.0)
        self._add_button_to_box(cont_box, "Stop", UICommand.CAM_ZOOM_CONTINIOUS, 0)
        self._add_button_to_box(cont_box, "Zoom Out", UICommand.CAM_ZOOM_CONTINIOUS, -1.0)
        box.pack_start(cont_box, False, False, 0)

        # Step zoom
        step_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        step_box.pack_start(Gtk.Label(label="Step           "), False, False, 0)
        self._add_button_to_box(step_box, "Zoom In", UICommand.CAM_ZOOM_STEP, 1)
        self._add_button_to_box(step_box, "Zoom Out", UICommand.CAM_ZOOM_STEP, -1)
        box.pack_start(step_box, False, False, 0)

        # Range zoom slider
        self._create_slider_row(box, "Range", 0, 100, 0, "zoom_range", UICommand.CAM_ZOOM_RANGE)

        frame.add(box)
        return frame

    # ===== Focus Controls Group =====
    def _create_focus_controls_group(self):
        """Create focus controls"""
        frame = Gtk.Frame(label="Focus")

        hbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        hbox.set_margin_top(10)
        hbox.set_margin_bottom(10)
        hbox.set_margin_start(10)
        hbox.set_margin_end(10)

        # EO Focus Speed slider
        self._create_slider_row(hbox, "EO Focus Speed", 0, 7, 3, "eo_focus_speed_range", UICommand.CAM_FOCUS_SPEED)

        # Continuous focus
        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        box.pack_start(Gtk.Label(label="Continuous"), False, False, 0)
        self._add_button_to_box(box, "Focus In", UICommand.CAM_FOCUS_CONTINIOUS, 1)
        self._add_button_to_box(box, "Stop", UICommand.CAM_FOCUS_CONTINIOUS, 0)
        self._add_button_to_box(box, "Focus Out", UICommand.CAM_FOCUS_CONTINIOUS, -1)
        hbox.pack_start(box, True, True, 0)

        # Auto focus
        self._add_button_to_box(hbox, "Auto", UICommand.CAM_FOCUS_AUTO, 1)

        frame.add(hbox)
        return frame

    # ===== Exposure Group =====
    def _create_exposure_group(self):
        """Create exposure controls"""
        frame = Gtk.Frame(label="Exposure")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_combo_box("ae_mode_combo", "Mode", AE_MODE_LIST, UICommand.CAM_AE_MODE), False, False, 0)

        hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        hbox.set_margin_top(10)
        hbox.set_margin_bottom(10)
        hbox.set_margin_start(10)
        hbox.set_margin_end(10)

        hbox.pack_start(self._create_combo_box("shutter_combo", "Shutter", SHUTTER_MODE_LIST, UICommand.CAM_SHUTTER), True, True, 0)
        hbox.pack_start(self._create_combo_box("iris_combo", "Iris", IRIS_MODE_LIST, UICommand.CAM_IRIS), True, True, 0)
        hbox.pack_start(self._create_combo_box("gain_combo", "Gain", GAIN_MODE_LIST, UICommand.CAM_GAIN), True, True, 0)

        box.pack_start(hbox, False, False, 0)

        frame.add(box)
        return frame

    # ===== White Balance Group =====
    def _create_white_balance_group(self):
        """Create white balance controls"""
        frame = Gtk.Frame(label="White Balance")

        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_combo_box("wb_mode_combo", "Mode", WHITE_BALANCE_LIST, UICommand.CAM_WHITE_BALANCE), True, True, 0)
        self._add_button_to_box(box, "WB Trigger", UICommand.CAM_WHITE_BALANCE_TRIGGER, 1.0)

        frame.add(box)
        return frame

    # ===== IR Palette Group =====
    def _create_ir_palette_group(self):
        """Create IR camera controls"""
        frame = Gtk.Frame(label="IR Camera")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_combo_box("ir_palette_combo", "Palette", UI_IR_PALETTE_LIST, UICommand.CAM_IR_PALETTE), True, True, 0)

        hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        hbox.set_margin_top(10)
        hbox.set_margin_bottom(10)
        hbox.set_margin_start(10)
        hbox.set_margin_end(10)

        hbox.pack_start(self._create_combo_box("ffc_mode_combo", "FFC Mode", FFC_MODE_LIST, UICommand.CAM_IR_FFC_MODE), True, True, 0)
        self._add_button_to_box(hbox, "FFC Trigger", UICommand.CAM_IR_FFC_TRIGGER, 1.0)

        box.pack_start(hbox, True, True, 0)

        frame.add(box)
        return frame

    # ===== LRF Mode Group =====
    def _create_lrf_mode_group(self):
        """Create LRF mode controls"""
        frame = Gtk.Frame(label="LRF")

        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_combo_box("lrf_mode_combo", "Frequency", LRF_MODE_LIST, UICommand.CAM_LRF_MODE), True, True, 0)

        frame.add(box)
        return frame

    # ===== OSD Mode Group =====
    def _create_osd_mode_group(self):
        """Create OSD mode controls"""
        frame = Gtk.Frame(label="OSD Mode")

        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_combo_box("osd_mode_combo", "Mode", UI_OSD_MODE_LIST, UICommand.CAM_OSD_MODE), True, True, 0)

        frame.add(box)
        return frame

    # ===== Image Flip Group =====
    def _create_image_flip_group(self):
        """Create image flip controls"""
        frame = Gtk.Frame(label="Image Flip")

        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_combo_box("image_flip_combo", "Mode", UI_IMAGE_FLIP_LIST, UICommand.CAM_IMAGE_FLIP), True, True, 0)

        frame.add(box)
        return frame

    # ===== MB1 General Settings Group =====
    def _create_mb1_general_settings_group(self):
        """Create MB1 general settings group"""
        frame = Gtk.Frame(label="MB1 Settings")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_aligned_combo("setting_target_combo", "Target", UI_MB1_SETTING_TARGET_LIST, UICommand.CAM_SETTING_TARGET), False, False, 0)
        box.pack_start(self._create_aligned_combo("rc_mode_combo", "RC Mode", UI_MB1_RC_MODE_LIST, UICommand.CAM_RC_MODE), False, False, 0)
        box.pack_start(self._create_aligned_combo("storage_type_combo", "Storage", UI_MB1_STORAGE_TYPE_LIST, UICommand.CAM_STORAGE_TYPE), False, False, 0)
        box.pack_start(self._create_aligned_combo("object_detection_combo", "Object Detect", UI_MB1_OBJECT_DETECTION_LIST, UICommand.CAM_OBJECT_DETECTION), False, False, 0)
        box.pack_start(self._create_aligned_combo("ir_isotherms_gain_combo", "Isotherms Gain", UI_MB1_IR_ISOTHERMS_GAIN_LIST, UICommand.CAM_IR_ISOTHERMS_GAIN), False, False, 0)
        box.pack_start(self._create_aligned_combo("gimbal_fw_flag_combo", "Gimbal FW Flag", UI_MB1_GIMBAL_FW_FLAG_LIST, UICommand.CAM_GIMBAL_FW_FLAG), False, False, 0)

        frame.add(box)
        return frame

    # ===== MB1 EO Advanced Group =====
    def _create_mb1_eo_advanced_group(self):
        """Create MB1 EO advanced settings group"""
        frame = Gtk.Frame(label="EO Advanced")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_aligned_combo("eo_scene_mode_combo", "Scene Mode", UI_MB1_EO_SCENE_MODE_LIST, UICommand.CAM_EO_SCENE_MODE), False, False, 0)

        # AE Compensation slider
        hbox1 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        label1 = Gtk.Label(label="AE Comp")
        label1.set_size_request(100, -1)
        label1.set_halign(Gtk.Align.START)
        self.eo_ae_compensation_range = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL)
        self.eo_ae_compensation_range.set_range(-12, 12)
        self.eo_ae_compensation_range.set_increments(1, 1)
        self.eo_ae_compensation_range.set_digits(0)
        self.eo_ae_compensation_range.set_value(2)
        self.eo_ae_compensation_range.connect("value-changed", lambda s: self._on_button_clicked(UICommand.CAM_EO_AE_COMPENSATION, [s.get_value()]))
        hbox1.pack_start(label1, False, False, 0)
        hbox1.pack_start(self.eo_ae_compensation_range, True, True, 0)
        box.pack_start(hbox1, False, False, 0)

        box.pack_start(self._create_aligned_combo("eo_wb_combo", "White Balance", UI_MB1_EO_WB_LIST, UICommand.CAM_EO_WHITE_BALANCE), False, False, 0)
        box.pack_start(self._create_aligned_combo("eo_iso_combo", "ISO", UI_MB1_EO_ISO_LIST, UICommand.CAM_EO_ISO), False, False, 0)

        # Sharpness slider
        hbox2 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        label2 = Gtk.Label(label="Sharpness")
        label2.set_size_request(100, -1)
        label2.set_halign(Gtk.Align.START)
        self.eo_sharpness_range = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL)
        self.eo_sharpness_range.set_range(0, 6)
        self.eo_sharpness_range.set_increments(1, 1)
        self.eo_sharpness_range.set_digits(0)
        self.eo_sharpness_range.set_value(2)
        self.eo_sharpness_range.connect("value-changed", lambda s: self._on_button_clicked(UICommand.CAM_EO_SHARPNESS, [s.get_value()]))
        hbox2.pack_start(label2, False, False, 0)
        hbox2.pack_start(self.eo_sharpness_range, True, True, 0)
        box.pack_start(hbox2, False, False, 0)

        frame.add(box)
        return frame

    # ===== MB1 IR Advanced Group =====
    def _create_mb1_ir_advanced_group(self):
        """Create MB1 IR advanced settings group"""
        frame = Gtk.Frame(label="IR Advanced")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_aligned_combo("ir_gain_mode_combo", "Gain Mode", UI_MB1_IR_GAIN_MODE_LIST, UICommand.CAM_IR_GAIN_MODE), False, False, 0)
        box.pack_start(self._create_aligned_combo("ir_contrast_mode_combo", "Contrast", UI_MB1_IR_CONTRAST_MODE_LIST, UICommand.CAM_IR_CONTRAST_MODE), False, False, 0)
        box.pack_start(self._create_aligned_combo("ir_agc_mode_combo", "AGC Mode", UI_MB1_IR_AGC_MODE_LIST, UICommand.CAM_IR_AGC_MODE), False, False, 0)

        # AGC Linear Percent slider
        hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        label = Gtk.Label(label="AGC Linear %")
        label.set_size_request(100, -1)
        label.set_halign(Gtk.Align.START)
        self.ir_agc_linear_percent_range = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL)
        self.ir_agc_linear_percent_range.set_range(0, 100)
        self.ir_agc_linear_percent_range.set_increments(10, 10)
        self.ir_agc_linear_percent_range.set_digits(0)
        self.ir_agc_linear_percent_range.set_value(0)
        self.ir_agc_linear_percent_range.connect("value-changed", lambda s: self._on_button_clicked(UICommand.CAM_IR_AGC_LINEAR_PERCENT, [s.get_value()]))
        hbox.pack_start(label, False, False, 0)
        hbox.pack_start(self.ir_agc_linear_percent_range, True, True, 0)
        box.pack_start(hbox, False, False, 0)

        frame.add(box)
        return frame

    # ===== MB1 IR SpotMeter Group =====
    def _create_mb1_ir_spotmeter_group(self):
        """Create MB1 IR SpotMeter settings group"""
        frame = Gtk.Frame(label="IR SpotMeter")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_aligned_combo("ir_spotmeter_mode_combo", "Mode", UI_MB1_IR_SPOTMETER_MODE_LIST, UICommand.CAM_IR_SPOTMETER_MODE), False, False, 0)
        box.pack_start(self._create_aligned_combo("ir_spotmeter_units_combo", "Units", UI_MB1_IR_SPOTMETER_UNITS_LIST, UICommand.CAM_IR_SPOTMETER_UNITS), False, False, 0)

        # SpotMeter Size slider
        hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        label = Gtk.Label(label="Size")
        label.set_size_request(80, -1)
        label.set_halign(Gtk.Align.START)
        self.ir_spotmeter_size_range = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL)
        self.ir_spotmeter_size_range.set_range(16, 128)
        self.ir_spotmeter_size_range.set_increments(4, 4)
        self.ir_spotmeter_size_range.set_digits(0)
        self.ir_spotmeter_size_range.set_value(16)
        self.ir_spotmeter_size_range.connect("value-changed", lambda s: self._on_button_clicked(UICommand.CAM_IR_SPOTMETER_SIZE, [s.get_value()]))
        hbox.pack_start(label, False, False, 0)
        hbox.pack_start(self.ir_spotmeter_size_range, True, True, 0)
        box.pack_start(hbox, False, False, 0)

        frame.add(box)
        return frame

    # ===== MB1 IR Isotherm Group =====
    def _create_mb1_ir_isotherm_group(self):
        """Create MB1 IR Isotherm settings group"""
        frame = Gtk.Frame(label="IR Isotherm")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        box.pack_start(self._create_aligned_combo("ir_isotherm_mode_combo", "Mode", UI_MB1_IR_ISOTHERM_MODE_LIST, UICommand.CAM_IR_ISOTHERM_MODE), False, False, 0)
        box.pack_start(self._create_aligned_combo("ir_isotherm_units_combo", "Units", UI_MB1_IR_ISOTHERM_UNITS_LIST, UICommand.CAM_IR_ISOTHERM_UNITS), False, False, 0)

        # Isotherm Threshold slider
        hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        label = Gtk.Label(label="Threshold")
        label.set_size_request(80, -1)
        label.set_halign(Gtk.Align.START)
        self.ir_isotherm_threshold_range = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL)
        self.ir_isotherm_threshold_range.set_range(0, 150)
        self.ir_isotherm_threshold_range.set_increments(5, 5)
        self.ir_isotherm_threshold_range.set_digits(0)
        self.ir_isotherm_threshold_range.set_value(50)
        self.ir_isotherm_threshold_range.connect("value-changed", lambda s: self._on_button_clicked(UICommand.CAM_IR_ISOTHERM_THRESHOLD, [s.get_value()]))
        hbox.pack_start(label, False, False, 0)
        hbox.pack_start(self.ir_isotherm_threshold_range, True, True, 0)
        box.pack_start(hbox, False, False, 0)

        frame.add(box)
        return frame

    # ===== Gimbal Control Speed Group =====
    def _create_gimbal_control_speed_group(self):
        """Create gimbal speed controls"""
        frame = Gtk.Frame(label="Control Speed")

        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        # Speed slider
        self.speed_gimbal_range = Gtk.Scale(orientation=Gtk.Orientation.VERTICAL)
        self.speed_gimbal_range.set_range(1, 180)
        self.speed_gimbal_range.set_inverted(True)
        self.speed_gimbal_range.set_value(20)
        self.speed_gimbal_range.connect("value-changed", self._on_speed_gimbal_changed)
        box.pack_start(self.speed_gimbal_range, False, False, 0)

        # Direction buttons grid
        grid = Gtk.Grid()
        grid.set_row_spacing(10)
        grid.set_column_spacing(10)
        grid.set_margin_top(10)
        grid.set_margin_bottom(10)
        grid.set_margin_start(10)
        grid.set_margin_end(10)

        btn_up = Gtk.Button(label="Up")
        btn_left = Gtk.Button(label="Left")
        btn_home = Gtk.Button(label="Home")
        btn_right = Gtk.Button(label="Right")
        btn_down = Gtk.Button(label="Down")

        grid.attach(btn_up, 1, 0, 1, 1)
        grid.attach(btn_left, 0, 1, 1, 1)
        grid.attach(btn_home, 1, 1, 1, 1)
        grid.attach(btn_right, 2, 1, 1, 1)
        grid.attach(btn_down, 1, 2, 1, 1)

        # Home button
        btn_home.connect("clicked", lambda b: self._on_button_clicked(UICommand.GIMBAL_MODE, [4.0]))

        # Direction buttons with press/release
        self._bind_control_gimbal_button(btn_up, UICommand.GIMBAL_CONTROL_TILT, 1.0)
        self._bind_control_gimbal_button(btn_down, UICommand.GIMBAL_CONTROL_TILT, -1.0)
        self._bind_control_gimbal_button(btn_left, UICommand.GIMBAL_CONTROL_PAN, -1.0)
        self._bind_control_gimbal_button(btn_right, UICommand.GIMBAL_CONTROL_PAN, 1.0)

        box.pack_start(grid, False, False, 0)

        frame.add(box)
        return frame

    # ===== Gimbal Control Angle Group =====
    def _create_gimbal_control_angle_group(self):
        """Create gimbal angle controls"""
        frame = Gtk.Frame(label="Control Angle")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)

        # Pitch
        hbox1 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        hbox1.pack_start(Gtk.Label(label="Pitch"), False, False, 0)
        self.pitch_angle_gimbal_range = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL)
        self.pitch_angle_gimbal_range.set_range(-90, 90)
        self.pitch_angle_gimbal_range.set_value(0)
        self.pitch_angle_gimbal_range.connect("value-changed", self._on_angle_gimbal_changed)
        hbox1.pack_start(self.pitch_angle_gimbal_range, True, True, 0)

        # Roll
        hbox2 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        hbox2.pack_start(Gtk.Label(label="Roll  "), False, False, 0)
        self.roll_angle_gimbal_range = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL)
        self.roll_angle_gimbal_range.set_range(-45, 45)
        self.roll_angle_gimbal_range.set_value(0)
        self.roll_angle_gimbal_range.connect("value-changed", self._on_angle_gimbal_changed)
        hbox2.pack_start(self.roll_angle_gimbal_range, True, True, 0)

        # Yaw
        hbox3 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        hbox3.pack_start(Gtk.Label(label="Yaw  "), False, False, 0)
        self.yaw_angle_gimbal_range = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL)
        self.yaw_angle_gimbal_range.set_range(-180, 180)
        self.yaw_angle_gimbal_range.set_value(0)
        self.yaw_angle_gimbal_range.connect("value-changed", self._on_angle_gimbal_changed)
        hbox3.pack_start(self.yaw_angle_gimbal_range, True, True, 0)

        box.pack_start(hbox1, False, False, 0)
        box.pack_start(hbox2, False, False, 0)
        box.pack_start(hbox3, False, False, 0)

        frame.add(box)
        return frame

    # ===== Helper Methods =====
    def _create_info_row(self, parent_box, title):
        """Create info row with label"""
        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)

        title_label = Gtk.Label(label=title)
        title_label.set_size_request(150, -1)
        title_label.set_halign(Gtk.Align.START)
        title_label.set_xalign(0.0)
        box.pack_start(title_label, False, False, 0)

        value_label = Gtk.Label(label="---")
        value_label.set_halign(Gtk.Align.START)
        box.pack_start(value_label, True, True, 0)

        parent_box.pack_start(box, False, False, 0)
        return value_label

    def _create_combo_box(self, attr_name, label_text, options, command):
        """Create combo box with label

        Args:
            attr_name: Attribute name to store combo box reference
            label_text: Label text (can be empty string)
            options: List of options - either strings or (label, value) tuples
            command: UICommand to send when selection changes
        """
        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)

        if label_text:
            label = Gtk.Label(label=label_text)
            box.pack_start(label, False, False, 0)

        combo = Gtk.ComboBoxText()

        # Build value mapping for options with (label, value) tuples
        value_map = []
        for opt in options:
            if isinstance(opt, tuple):
                combo.append_text(opt[0])  # label
                value_map.append(opt[1])    # value
            else:
                combo.append_text(opt)
                value_map.append(None)  # No specific value, use index

        combo.set_active(0)
        # Store value map on combo for later use
        combo._value_map = value_map
        combo.connect("changed", lambda c: self._on_combo_changed_with_value(c, command))

        setattr(self, attr_name, combo)
        box.pack_start(combo, True, True, 0)

        return box

    def _create_aligned_combo(self, attr_name, label_text, options, command, label_width=120):
        """Create combo box with fixed-width label (for MB1 settings)

        Args:
            attr_name: Attribute name to store combo box reference
            label_text: Label text
            options: List of options - either strings or (label, value) tuples
            command: UICommand to send when selection changes
            label_width: Fixed width for label
        """
        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)

        label = Gtk.Label(label=label_text)
        label.set_size_request(label_width, -1)
        label.set_halign(Gtk.Align.START)
        box.pack_start(label, False, False, 0)

        combo = Gtk.ComboBoxText()

        # Build value mapping for options with (label, value) tuples
        value_map = []
        for opt in options:
            if isinstance(opt, tuple):
                combo.append_text(opt[0])  # label
                value_map.append(opt[1])    # value
            else:
                combo.append_text(opt)
                value_map.append(None)  # No specific value, use index

        combo.set_active(0)
        # Store value map on combo for later use
        combo._value_map = value_map
        combo.connect("changed", lambda c: self._on_combo_changed_with_value(c, command))

        setattr(self, attr_name, combo)
        box.pack_start(combo, True, True, 0)

        return box

    def _create_slider_row(self, parent_box, label_text, min_val, max_val, default_val, attr_name, command):
        """Create slider row"""
        hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)

        label = Gtk.Label(label=label_text)
        hbox.pack_start(label, False, False, 0)

        scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL)
        scale.set_range(min_val, max_val)
        scale.set_value(default_val)
        scale.set_digits(0)
        scale.connect("value-changed", lambda s: self._on_slider_changed(s, command))

        setattr(self, attr_name, scale)
        hbox.pack_start(scale, True, True, 0)

        parent_box.pack_start(hbox, False, False, 0)

    def _add_button_to_box(self, box, label, command, value):
        """Add button to box"""
        btn = Gtk.Button(label=label)
        btn.connect("clicked", lambda b: self._on_button_clicked(command, [float(value)]))
        box.pack_start(btn, True, True, 0)

    def _bind_control_gimbal_button(self, button, command, direction):
        """Bind gimbal control button with press/release"""
        button.connect("pressed", lambda b: self._on_button_clicked(command, [direction * self.speed_gimbal]))
        button.connect("released", lambda b: self._on_button_clicked(command, [0.0]))

    # ===== Event Handlers =====
    def connect_button_clicked(self, callback):
        """Register button click callback"""
        self._button_clicked_callback = callback

    def _on_button_clicked(self, command, params):
        """Handle button click"""
        if self._button_clicked_callback:
            self._button_clicked_callback(command, params)

    def _on_combo_changed(self, combo, command):
        """Handle combo box change (legacy - uses index as value)"""
        value = combo.get_active()
        self._on_button_clicked(command, [float(value)])

    def _on_combo_changed_with_value(self, combo, command):
        """Handle combo box change with value mapping

        Uses the stored _value_map to get the actual value to send.
        If value_map entry is None, falls back to index.
        """
        index = combo.get_active()
        if hasattr(combo, '_value_map') and combo._value_map:
            value = combo._value_map[index]
            if value is None:
                value = index
        else:
            value = index
        self._on_button_clicked(command, [float(value)])

    def _on_slider_changed(self, slider, command):
        """Handle slider change"""
        value = slider.get_value()
        self._on_button_clicked(command, [value])

    def _on_speed_gimbal_changed(self, scale):
        """Handle gimbal speed slider change"""
        self.speed_gimbal = scale.get_value()

    def _on_angle_gimbal_changed(self, scale):
        """Handle gimbal angle slider change"""
        pitch = self.pitch_angle_gimbal_range.get_value() if self.pitch_angle_gimbal_range else 0
        roll = self.roll_angle_gimbal_range.get_value() if self.roll_angle_gimbal_range else 0
        yaw = self.yaw_angle_gimbal_range.get_value() if self.yaw_angle_gimbal_range else 0
        self._on_button_clicked(UICommand.GIMBAL_CONTROL_ANGLE, [pitch, roll, yaw])

    def _on_touch_toggled(self, button):
        """Handle touch button toggle"""
        ctx = button.get_style_context()
        if button.get_active():
            ctx.remove_class("toggle-off")
            ctx.add_class("toggle-on")
            self.is_touch = True
        else:
            ctx.remove_class("toggle-on")
            ctx.add_class("toggle-off")
            self.is_touch = False

    def _on_track_toggled(self, button):
        """Handle track button toggle"""
        ctx = button.get_style_context()
        if button.get_active():
            ctx.remove_class("toggle-off")
            ctx.add_class("toggle-on")
            self._on_button_clicked(UICommand.PAYLOAD_TRACK, [1.0])
        else:
            ctx.remove_class("toggle-on")
            ctx.add_class("toggle-off")
            self._on_button_clicked(UICommand.PAYLOAD_TRACK, [0.0])

    # ===== Video Controls =====
    def _on_video_area_draw(self, widget, cr):
        """Draw video area placeholder"""
        if not self.is_playing:
            allocation = widget.get_allocation()
            width = allocation.width
            height = allocation.height

            # Fill with black
            cr.set_source_rgb(0.0, 0.0, 0.0)
            cr.rectangle(0, 0, width, height)
            cr.fill()

            # Draw text
            cr.set_source_rgb(0.7, 0.7, 0.7)
            cr.select_font_face("Sans", 0, 0)
            cr.set_font_size(16)

            text = "No video stream"
            extents = cr.text_extents(text)
            cr.move_to((width - extents.width) / 2, (height + extents.height) / 2)
            cr.show_text(text)

        return True

    def _on_video_area_realize(self, widget):
        """Handle video area realize"""
        window = widget.get_window()
        if window:
            # Get X11 window ID for GStreamer
            if hasattr(window, 'get_xid'):
                self.video_window_handle = window.get_xid()

    def _on_video_area_clicked(self, widget, event):
        """Handle video area click for touch/track"""
        if self.is_touch and self.is_playing:
            x_screen = event.x
            y_screen = event.y
            width = widget.get_allocated_width()
            height = widget.get_allocated_height()

            x_send = x_screen / width * 1920
            y_send = y_screen / height * 1080

            self._on_button_clicked(UICommand.PAYLOAD_TOUCH, [x_send, y_send])
        return True

    def _on_url_entry_activate(self, entry):
        """Handle URL entry activate"""
        self._on_play_button_clicked(None)

    def _on_play_button_clicked(self, button):
        """Handle play button click"""
        url = self.url_entry.get_text()
        if not url:
            return
        self._play_stream(url)

    def _on_stop_button_clicked(self, button):
        """Handle stop button click"""
        self._stop_stream()

    def _on_fullscreen_clicked(self, button):
        """Handle fullscreen button click"""
        if not self.is_playing:
            return

        if self.is_fullscreen:
            self._exit_fullscreen()
        else:
            self._enter_fullscreen()

    def _enter_fullscreen(self):
        """Enter fullscreen mode"""
        if self.is_fullscreen or not self.is_playing:
            return

        # Create fullscreen window
        self.fullscreen_window = Gtk.Window(type=Gtk.WindowType.TOPLEVEL)
        self.fullscreen_window.set_title("Video Fullscreen")
        self.fullscreen_window.set_decorated(False)
        self.fullscreen_window.fullscreen()

        # Create video area for fullscreen
        self.fullscreen_video_area = Gtk.DrawingArea()
        self.fullscreen_video_area.set_double_buffered(False)
        self.fullscreen_video_area.connect("realize", self._on_fullscreen_video_realize)
        self.fullscreen_video_area.connect("draw", self._on_fullscreen_video_draw)

        # Handle key press for ESC to exit fullscreen
        self.fullscreen_window.connect("key-press-event", self._on_fullscreen_key_press)
        self.fullscreen_window.connect("destroy", self._on_fullscreen_window_destroy)

        # Add touch support for fullscreen
        self.fullscreen_video_area.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        self.fullscreen_video_area.connect("button-press-event", self._on_fullscreen_video_clicked)

        self.fullscreen_window.add(self.fullscreen_video_area)
        self.fullscreen_window.show_all()

        self.is_fullscreen = True
        if self.fullscreen_button:
            self.fullscreen_button.set_label("Exit Fullscreen")

    def _on_fullscreen_video_realize(self, widget):
        """Handle fullscreen video area realize"""
        window = widget.get_window()
        if window and hasattr(window, 'get_xid'):
            self.fullscreen_video_handle = window.get_xid()
            # Redirect video to fullscreen window
            if self.pipeline:
                vsink = self.pipeline.get_by_name('vsink')
                if vsink:
                    vsink.set_window_handle(self.fullscreen_video_handle)

    def _on_fullscreen_video_draw(self, widget, cr):
        """Draw fullscreen video area placeholder"""
        # Let GStreamer handle drawing
        return False

    def _on_fullscreen_key_press(self, widget, event):
        """Handle key press in fullscreen mode"""
        if event.keyval == Gdk.KEY_Escape or event.keyval == Gdk.KEY_F11:
            self._exit_fullscreen()
            return True
        return False

    def _on_fullscreen_window_destroy(self, widget):
        """Handle fullscreen window destroy"""
        self._exit_fullscreen()

    def _on_fullscreen_video_clicked(self, widget, event):
        """Handle click on fullscreen video for touch/track"""
        if self.is_touch and self.is_playing:
            x_screen = event.x
            y_screen = event.y
            width = widget.get_allocated_width()
            height = widget.get_allocated_height()

            x_send = x_screen / width * 1920
            y_send = y_screen / height * 1080

            self._on_button_clicked(UICommand.PAYLOAD_TOUCH, [x_send, y_send])
        return True

    def _exit_fullscreen(self):
        """Exit fullscreen mode"""
        if not self.is_fullscreen:
            return

        # Redirect video back to original window
        if self.pipeline and self.video_window_handle:
            vsink = self.pipeline.get_by_name('vsink')
            if vsink:
                vsink.set_window_handle(self.video_window_handle)

        # Destroy fullscreen window
        if self.fullscreen_window:
            self.fullscreen_window.destroy()
            self.fullscreen_window = None
            self.fullscreen_video_area = None
            self.fullscreen_video_handle = 0

        self.is_fullscreen = False
        if self.fullscreen_button:
            self.fullscreen_button.set_label("Fullscreen")

    def _play_stream(self, rtsp_url):
        """Start video stream"""
        if self.is_playing:
            self._stop_stream()

        # Create GStreamer pipeline with xvimagesink for X11 window embedding
        # Use ximagesink or xvimagesink instead of autovideosink for window handle support
        pipeline_str = f"rtspsrc location={rtsp_url} latency=200 ! decodebin ! videoconvert ! xvimagesink name=vsink sync=false"

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            if not self.pipeline:
                print("Failed to create pipeline")
                return

            # Get video sink and connect to bus for window handle
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.enable_sync_message_emission()
            bus.connect("sync-message::element", self._on_sync_message)

            # Start playing
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                print("Failed to start pipeline")
                self._cleanup_gstreamer()
                return

            self.is_playing = True
            self.play_button.set_sensitive(False)
            self.stop_button.set_sensitive(True)
            self.url_entry.set_sensitive(False)
            if self.fullscreen_button:
                self.fullscreen_button.set_sensitive(True)

        except Exception as e:
            print(f"Error starting stream: {e}")

    def _on_sync_message(self, bus, message):
        """Handle GStreamer sync messages for video overlay"""
        if message.get_structure() is None:
            return

        message_name = message.get_structure().get_name()
        if message_name == "prepare-window-handle":
            # Get the video sink element
            imagesink = message.src
            # Set the window handle
            if self.video_window_handle:
                imagesink.set_window_handle(self.video_window_handle)

    def _stop_stream(self):
        """Stop video stream"""
        # Exit fullscreen first if active
        if self.is_fullscreen:
            self._exit_fullscreen()

        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        self._cleanup_gstreamer()

        self.is_playing = False
        self.play_button.set_sensitive(True)
        self.stop_button.set_sensitive(False)
        self.url_entry.set_sensitive(True)
        if self.fullscreen_button:
            self.fullscreen_button.set_sensitive(False)
            self.fullscreen_button.set_label("Fullscreen")

        if self.video_area:
            self.video_area.queue_draw()

    def _cleanup_gstreamer(self):
        """Cleanup GStreamer resources"""
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None

    # ===== Update Methods =====
    def send_connected(self):
        """Handle connected state"""
        pass

    def send_disconnected(self):
        """Handle disconnected state"""
        self._stop_stream()

    def update_rtsp_url_from_ip(self, ip):
        """Update RTSP URL from IP"""
        if self.url_entry:
            self.url_entry.set_text(f"rtsp://{ip}:8554/payload")

    def update_storage_info(self, status, total, used, available):
        """Update storage info label

        Args:
            status: Storage status (0 = no card, other = card present)
            total: Total capacity in MB
            used: Used capacity in MB
            available: Available capacity in MB
        """
        if self.storage_info:
            if status == 0:
                self.storage_info.set_text("No SD Card")
            else:
                # Convert MB to GB (divide by 1024)
                available_gb = available / 1024.0
                total_gb = total / 1024.0
                self.storage_info.set_text(f"{available_gb:.2f} GB free of {total_gb:.2f} GB")

    def update_capture_info(self, img_status, video_status, img_count, rec_time_ms):
        """Update capture/record info"""
        if self.capture_info:
            self.capture_info.set_text(str(img_count))

        if self.record_info:
            hours = rec_time_ms // 3600000
            minutes = (rec_time_ms % 3600000) // 60000
            seconds = (rec_time_ms % 60000) // 1000
            self.record_info.set_text(f"{hours:02d}:{minutes:02d}:{seconds:02d}")

        self.rec_status = video_status

    def update_gimbal_attitude(self, pitch, roll, yaw):
        """Update gimbal attitude info"""
        if self.pitch_angle_info:
            self.pitch_angle_info.set_text(f"{pitch:.2f}")
        if self.roll_angle_info:
            self.roll_angle_info.set_text(f"{roll:.2f}")
        if self.yaw_angle_info:
            self.yaw_angle_info.set_text(f"{yaw:.2f}")

    def update_payload_status(self, params):
        """Update payload status from params list [index, value]

        This matches C++ update_payload_status(double* params) where:
        - params[0] = param index (PARAM_xxx enum)
        - params[1] = value
        """
        if not params or len(params) < 2:
            return

        index = int(params[0])
        value = params[1]

        # Import payload_param_t enum values
        # These match the C++ PARAM_xxx enum in payloadSdkInterface.h
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
        PARAM_CAM_VIEW_MODE = 19
        PARAM_CAM_REC_SOURCE = 20
        PARAM_CAM_IR_TYPE = 21
        PARAM_CAM_IR_PALETTE_ID = 22
        PARAM_CAM_IR_FFC_MODE = 23
        PARAM_GIMBAL_MODE = 24
        PARAM_IR_TEMP_MAX = 25
        PARAM_IR_TEMP_MIN = 26
        PARAM_IR_TEMP_MEAN = 27

        if index == PARAM_GIMBAL_MODE:
            if value == 4:
                return  # Skip invalid mode
            if self.gimbal_mode_info:
                mode_names = {0: "OFF", 1: "LOCK", 2: "FOLLOW", 3: "MAPPING"}
                self.gimbal_mode_info.set_text(mode_names.get(int(value), "---"))
            # Update combo box
            if self.gimbal_mode_combo:
                self._update_combo_by_value(self.gimbal_mode_combo, int(value))

        elif index == PARAM_CAM_VIEW_MODE:
            if self.view_mode_info:
                mode_names = {0: "EO/IR", 1: "EO", 2: "IR", 3: "IR/EO", 4: "SYNC", 6: "SIDE BY SIDE"}
                self.view_mode_info.set_text(mode_names.get(int(value), "---"))
            if self.view_mode_combo:
                self._update_combo_by_value(self.view_mode_combo, int(value))

        elif index == PARAM_CAM_REC_SOURCE:
            if self.record_src_info:
                src_names = {0: "Both EO/IR", 1: "EO", 2: "IR", 5: "OSD"}
                self.record_src_info.set_text(src_names.get(int(value), "---"))
            if self.rec_src_combo:
                self._update_combo_by_value(self.rec_src_combo, int(value))

        elif index == PARAM_EO_ZOOM_LEVEL:
            if self.eo_zoom_level_info:
                self.eo_zoom_level_info.set_text(f"{value:.2f}")

        elif index == PARAM_IR_ZOOM_LEVEL:
            if self.ir_zoom_level_info:
                self.ir_zoom_level_info.set_text(f"{value:.2f}")

        elif index == PARAM_CAM_IR_TYPE:
            if self.ir_type_info:
                type_names = {0: "NO THERMAL CAMERA", 1: "G1", 2: "F1"}
                self.ir_type_info.set_text(type_names.get(int(value), "---"))

        elif index == PARAM_CAM_IR_PALETTE_ID:
            if self.ir_palette_info:
                self.ir_palette_info.set_text(f"PALETTE {int(value) + 1}")
            if self.ir_palette_combo:
                self._update_combo_by_value(self.ir_palette_combo, int(value))

        elif index == PARAM_CAM_IR_FFC_MODE:
            if self.ir_ffc_mode_info:
                mode_names = {0: "Manual", 1: "Auto"}
                self.ir_ffc_mode_info.set_text(mode_names.get(int(value), "---"))
            if self.ffc_mode_combo:
                self._update_combo_by_value(self.ffc_mode_combo, int(value))

        elif index == PARAM_IR_TEMP_MAX:
            if self.ir_temp_max_info:
                self.ir_temp_max_info.set_text(f"{value:.2f}°C")

        elif index == PARAM_IR_TEMP_MIN:
            if self.ir_temp_min_info:
                self.ir_temp_min_info.set_text(f"{value:.2f}°C")

        elif index == PARAM_IR_TEMP_MEAN:
            if self.ir_temp_mean_info:
                self.ir_temp_mean_info.set_text(f"{value:.2f}°C")

        elif index == PARAM_LRF_RANGE:
            if self.lrf_range_info:
                self.lrf_range_info.set_text(f"{value:.2f}m")

        elif index == PARAM_LRF_OFSET_X:
            if self.lrf_offset_x_info:
                self.lrf_offset_x_info.set_text(f"{value:.2f}")

        elif index == PARAM_LRF_OFSET_Y:
            if self.lrf_offset_y_info:
                self.lrf_offset_y_info.set_text(f"{value:.2f}")

        elif index == PARAM_TARGET_COOR_LON:
            if self.target_gps_lon_info:
                self.target_gps_lon_info.set_text(f"{value:.6f}")

        elif index == PARAM_TARGET_COOR_LAT:
            if self.target_gps_lat_info:
                self.target_gps_lat_info.set_text(f"{value:.6f}")

        elif index == PARAM_TARGET_COOR_ALT:
            if self.target_gps_alt_info:
                self.target_gps_alt_info.set_text(f"{value:.2f}")

        elif index == PARAM_PAYLOAD_GPS_LON:
            if self.payload_gps_lon_info:
                self.payload_gps_lon_info.set_text(f"{value:.6f}")

        elif index == PARAM_PAYLOAD_GPS_LAT:
            if self.payload_gps_lat_info:
                self.payload_gps_lat_info.set_text(f"{value:.6f}")

        elif index == PARAM_PAYLOAD_GPS_ALT:
            if self.payload_gps_alt_info:
                self.payload_gps_alt_info.set_text(f"{value:.2f}")

        elif index == PARAM_TRACK_STATUS:
            is_track = (int(value) >> 8) & 0xff
            if self.track_button:
                ctx = self.track_button.get_style_context()
                if is_track == 1:
                    ctx.remove_class("toggle-off")
                    ctx.add_class("toggle-on")
                elif is_track == 0:
                    ctx.remove_class("toggle-on")
                    ctx.add_class("toggle-off")

    def _update_combo_by_value(self, combo, value):
        """Update combo box selection by matching value in _value_map"""
        if not combo or not hasattr(combo, '_value_map'):
            return
        try:
            # Find index where value matches
            for i, v in enumerate(combo._value_map):
                if v == value:
                    combo.set_active(i)
                    return
            # If no match found, try direct index
            if value < combo.get_model().iter_n_children(None):
                combo.set_active(int(value))
        except Exception:
            pass

    def update_payload_param(self, param_id, value):
        """Update payload parameter by param_id string

        This matches C++ update_payload_param(char* index, double value)
        Called when PAYLOAD_CAM_PARAMS event is received
        """
        if not param_id:
            return

        # Import param IDs from mb1_define or use string comparison
        param_id_str = param_id.rstrip('\0') if isinstance(param_id, str) else str(param_id)

        # Map param_id to update logic
        if param_id_str in ["C_V_ZOOM_SPEED", "ZOOM_SPEED", "C_V_Z_SPD"]:
            if self.eo_zoom_speed_range:
                self.eo_zoom_speed_range.set_value(float(value))

        elif param_id_str in ["C_V_FOCUS_SPEED", "FOCUS_SPEED", "C_V_F_SPD"]:
            if self.eo_focus_speed_range:
                self.eo_focus_speed_range.set_value(float(value))

        elif param_id_str in ["EO_ZOOM", "C_V_ZOOM"]:
            if self.eo_zoom_level_info:
                self.eo_zoom_level_info.set_text(f"{value:.2f}")

        elif param_id_str in ["IR_ZOOM", "C_T_ZOOM"]:
            if self.ir_zoom_level_info:
                self.ir_zoom_level_info.set_text(f"{value:.2f}")

        elif param_id_str == "LRF_RANGE":
            if self.lrf_range_info:
                self.lrf_range_info.set_text(f"{value:.2f}m")

        # Gimbal mode - param ID is "GB_MODE"
        elif param_id_str == "GB_MODE":
            int_value = int(value)
            if int_value == 4:
                return  # Skip RESET mode (temporary command)
            if self.gimbal_mode_info:
                mode_names = {0: "OFF", 1: "LOCK", 2: "FOLLOW", 3: "MAPPING"}
                self.gimbal_mode_info.set_text(mode_names.get(int_value, "---"))
            if self.gimbal_mode_combo:
                self._update_combo_by_value(self.gimbal_mode_combo, int_value)

        # View mode - param ID is "C_SOURCE"
        elif param_id_str == "C_SOURCE":
            int_value = int(value)
            if self.view_mode_info:
                mode_names = {0: "EO/IR", 1: "EO", 2: "IR", 3: "IR/EO", 4: "SYNC", 6: "SIDE BY SIDE"}
                self.view_mode_info.set_text(mode_names.get(int_value, "---"))
            if self.view_mode_combo:
                self._update_combo_by_value(self.view_mode_combo, int_value)

        # Record source - param ID is "C_V_REC"
        elif param_id_str == "C_V_REC":
            int_value = int(value)
            if self.record_src_info:
                src_names = {0: "Both EO/IR", 1: "EO", 2: "IR", 5: "OSD"}
                self.record_src_info.set_text(src_names.get(int_value, "---"))
            if self.rec_src_combo:
                self._update_combo_by_value(self.rec_src_combo, int_value)

        # IR Palette - param ID is "C_T_PALETTE"
        elif param_id_str == "C_T_PALETTE":
            int_value = int(value)
            if self.ir_palette_info:
                self.ir_palette_info.set_text(f"PALETTE {int_value + 1}")
            if self.ir_palette_combo:
                self._update_combo_by_value(self.ir_palette_combo, int_value)

        # IR FFC Mode - param ID is "FFC_MODE" or "IR_FFCMODE"
        elif param_id_str in ["FFC_MODE", "IR_FFCMODE"]:
            int_value = int(value)
            if self.ir_ffc_mode_info:
                mode_names = {0: "Manual", 1: "Auto"}
                self.ir_ffc_mode_info.set_text(mode_names.get(int_value, "---"))
            if self.ffc_mode_combo:
                self._update_combo_by_value(self.ffc_mode_combo, int_value)

        # OSD Mode - param ID is "OSD_MODE"
        elif param_id_str == "OSD_MODE":
            int_value = int(value)
            if self.osd_mode_combo:
                self._update_combo_by_value(self.osd_mode_combo, int_value)

        # Image Flip - param ID is "C_V_FLIP"
        elif param_id_str == "C_V_FLIP":
            int_value = int(value)
            if self.image_flip_combo:
                self._update_combo_by_value(self.image_flip_combo, int_value)

        # Tracking Mode - param ID is "TRACK_MODE"
        elif param_id_str == "TRACK_MODE":
            int_value = int(value)
            if self.track_mode_combo:
                self._update_combo_by_value(self.track_mode_combo, int_value)

        # LRF Mode - param ID is "LRF_MODE"
        elif param_id_str == "LRF_MODE":
            int_value = int(value)
            if self.lrf_mode_combo:
                self._update_combo_by_value(self.lrf_mode_combo, int_value)

        # LRF Offset X
        elif param_id_str == "LRF_OFFSET_X":
            if self.lrf_offset_x_info:
                self.lrf_offset_x_info.set_text(f"{value:.2f}")

        # LRF Offset Y
        elif param_id_str == "LRF_OFFSET_Y":
            if self.lrf_offset_y_info:
                self.lrf_offset_y_info.set_text(f"{value:.2f}")

        # Target GPS coordinates
        elif param_id_str == "TARGET_LON":
            if self.target_gps_lon_info:
                self.target_gps_lon_info.set_text(f"{value:.6f}")

        elif param_id_str == "TARGET_LAT":
            if self.target_gps_lat_info:
                self.target_gps_lat_info.set_text(f"{value:.6f}")

        elif param_id_str == "TARGET_ALT":
            if self.target_gps_alt_info:
                self.target_gps_alt_info.set_text(f"{value:.2f}")

        # Payload GPS coordinates
        elif param_id_str == "PAY_LON":
            if self.payload_gps_lon_info:
                self.payload_gps_lon_info.set_text(f"{value:.6f}")

        elif param_id_str == "PAY_LAT":
            if self.payload_gps_lat_info:
                self.payload_gps_lat_info.set_text(f"{value:.6f}")

        elif param_id_str == "PAY_ALT":
            if self.payload_gps_alt_info:
                self.payload_gps_alt_info.set_text(f"{value:.2f}")

        # IR Temperature values
        elif param_id_str == "IR_TEMP_MAX":
            if self.ir_temp_max_info:
                self.ir_temp_max_info.set_text(f"{value:.2f}°C")

        elif param_id_str == "IR_TEMP_MIN":
            if self.ir_temp_min_info:
                self.ir_temp_min_info.set_text(f"{value:.2f}°C")

        elif param_id_str == "IR_TEMP_MEAN":
            if self.ir_temp_mean_info:
                self.ir_temp_mean_info.set_text(f"{value:.2f}°C")

        # IR Type
        elif param_id_str == "IR_TYPE":
            int_value = int(value)
            if self.ir_type_info:
                type_names = {0: "NO THERMAL CAMERA", 1: "G1", 2: "F1"}
                self.ir_type_info.set_text(type_names.get(int_value, "---"))

    def update_url_streaming(self, url):
        """Update streaming URL"""
        if self.url_entry:
            self.url_entry.set_text(url)

    def update_gimbal_mode_from_string(self, mode_string):
        """Update gimbal mode from mode string

        Args:
            mode_string: Mode string from SDK like "LOCK_MODE", "FOLLOW_MODE", "OFF_MODE", "MAPPING_MODE", "RESET_MODE"
        """
        if not mode_string:
            return

        # Map mode string to display name and value
        mode_map = {
            "OFF_MODE": ("OFF", 0),
            "LOCK_MODE": ("LOCK", 1),
            "FOLLOW_MODE": ("FOLLOW", 2),
            "MAPPING_MODE": ("MAPPING", 3),
            "RESET_MODE": ("RESET", 4),
        }

        mode_str = mode_string.rstrip('\0') if isinstance(mode_string, str) else str(mode_string)

        if mode_str in mode_map:
            display_name, value = mode_map[mode_str]
            if value == 4:
                return  # Skip RESET mode (temporary command)
            if self.gimbal_mode_info:
                self.gimbal_mode_info.set_text(display_name)
            if self.gimbal_mode_combo:
                self._update_combo_by_value(self.gimbal_mode_combo, value)
