from enum_base import IntEnumBase, FloatEnumBase

PAYLOAD_CAMERA_TRACKING_MODE =                                          "TRACK_MODE"
class payload_camera_tracking_mode(IntEnumBase):
    PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING  =                             0
    PAYLOAD_CAMERA_TRACKING_OBJ_DETECTION =                             1

PAYLOAD_CAMERA_VIDEO_OSD_MODE =                                         "OSD_MODE"
class payload_camera_osd_mode(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_OSD_MODE_DISABLE =                             0
    PAYLOAD_CAMERA_VIDEO_OSD_MODE_DEBUG   =                             1
    PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS  =                             2

PAYLOAD_CAMERA_RC_MODE =                                                "RC_MODE"
class payload_camera_rc_mode(IntEnumBase):
    PAYLOAD_CAMERA_RC_MODE_GREMSY   =                                   0
    PAYLOAD_CAMERA_RC_MODE_STANDARD =                                   1

PAYLOAD_CAMERA_VIDEO_FLIP =                                             "C_V_FLIP"
class payload_camera_video_flip(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_FLIP_OFF =                                     3
    PAYLOAD_CAMERA_VIDEO_FLIP_ON  =                                     2

PAYLOAD_CAMERA_VIDEO_DEFOG =                                            "C_V_DEFOG"
class payload_camera_video_defog(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_DEFOG_OFF =                                    0
    PAYLOAD_CAMERA_VIDEO_DEFOG_ON  =                                    1

PAYLOAD_CAMERA_VIDEO_DEFOG_LEVEL =                                      "C_V_DEFOG_LV"
class payload_camera_video_defog_level(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_DEFOG_LOWEST =                                 0
    PAYLOAD_CAMERA_VIDEO_DEFOG_LOW    =                                 1
    PAYLOAD_CAMERA_VIDEO_DEFOG_MID    =                                 2
    PAYLOAD_CAMERA_VIDEO_DEFOG_HIGH   =                                 3

PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE =                                    "C_V_AE"
class payload_camera_video_auto_exposure(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_AUTO    =                             0
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_MANUAL  =                             3
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_SHUTTER =                             10
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_IRIS    =                             11
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_BRIGHT  =                             13

PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED =                                    "C_V_SP"
class payload_camera_video_shutter_speed(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_10   =                         13
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_20   =                         14
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_50   =                         17
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_100  =                         20
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_125  =                         21
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_500  =                         25
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_725  =                         26
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1000 =                         27
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1500 =                         28
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_2000 =                         30

# Aperture value can be set from 0 to 25, step 1
PAYLOAD_CAMERA_VIDEO_APERTURE_VALUE =                                   "C_V_IrP"

# Bright value can be set from 0 to 41, step 1a
PAYLOAD_CAMERA_VIDEO_BRIGHT_VALUE =                                     "C_V_BrP"

PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE =                                    "C_V_WB"
class payload_camera_video_white_balance(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_AUTO     =                       0
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_INDOOR   =                       1
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_OUTDOOR  =                       2
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_ONE_PUSH =                       3
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_ATW      =                       4
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_MANUAL   =                       5

PAYLOAD_CAMERA_VIDEO_ZOOM_MODE =                                        "C_V_ZM_MODE"
class payload_camera_video_zoom_mode(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_COMBINE          =                   0
    PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION =                   2

# Zoom super resolution value can be set from 1x to 30x
PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR =                     "C_V_ZM_SR_LV"
class payload_camera_video_zoom_super_resolution_factor(IntEnumBase):
    ZOOM_SUPER_RESOLUTION_1X  =                                         0
    ZOOM_SUPER_RESOLUTION_2X  =                                         1
    ZOOM_SUPER_RESOLUTION_4X  =                                         2
    ZOOM_SUPER_RESOLUTION_6X  =                                         3
    ZOOM_SUPER_RESOLUTION_8X  =                                         4
    ZOOM_SUPER_RESOLUTION_10X =                                         5
    ZOOM_SUPER_RESOLUTION_12X =                                         6
    ZOOM_SUPER_RESOLUTION_14X =                                         7
    ZOOM_SUPER_RESOLUTION_16X =                                         8
    ZOOM_SUPER_RESOLUTION_18X =                                         9
    ZOOM_SUPER_RESOLUTION_20X =                                         10
    ZOOM_SUPER_RESOLUTION_22X =                                         11
    ZOOM_SUPER_RESOLUTION_24X =                                         12
    ZOOM_SUPER_RESOLUTION_26X =                                         13
    ZOOM_SUPER_RESOLUTION_28X =                                         14
    ZOOM_SUPER_RESOLUTION_30X =                                         15

# Zoom super resolution value can be set from 1x to 240x
PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_FACTOR =                              "C_V_ZM_CB_LV"
class payload_camera_video_zoom_combine_factor(IntEnumBase):
    ZOOM_COMBINE_1X   =                                                 0
    ZOOM_COMBINE_10X  =                                                 1
    ZOOM_COMBINE_20X  =                                                 2
    ZOOM_COMBINE_40X  =                                                 3
    ZOOM_COMBINE_80X  =                                                 4
    ZOOM_COMBINE_120X =                                                 5
    ZOOM_COMBINE_240X =                                                 6

PAYLOAD_CAMERA_VIDEO_FOCUS_MODE =                                       "C_V_FM"
class payload_camera_video_focus_mode(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_MANUAL       =                      0
    PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_ZOOM_TRIGGER =                      1
    PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_NEAR    =                      2
    PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_FAR     =                      3

# Manual focus value can be set from 0 to 61440, step 10
PAYLOAD_CAMERA_VIDEO_FOCUS_VALUE =                                      "C_V_FV"

PAYLOAD_CAMERA_GIMBAL_MODE =                                            "GB_MODE"
class payload_camera_gimbal_mode(IntEnumBase):
    PAYLOAD_CAMERA_GIMBAL_MODE_OFF     =                                0
    PAYLOAD_CAMERA_GIMBAL_MODE_LOCK    =                                1
    PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW  =                                2
    PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING =                                3
    PAYLOAD_CAMERA_GIMBAL_MODE_RESET   =                                4

PAYLOAD_CAMERA_VIEW_SRC =                                               "C_SOURCE"
class payload_camera_view_src(IntEnumBase):
    PAYLOAD_CAMERA_VIEW_EOIR         =                                  0
    PAYLOAD_CAMERA_VIEW_EO           =                                  1
    PAYLOAD_CAMERA_VIEW_IR           =                                  2
    PAYLOAD_CAMERA_VIEW_IREO         =                                  3
    PAYLOAD_CAMERA_VIEW_SYNC         =                                  4
    PAYLOAD_CAMERA_VIEW_SIDE_BY_SIDE =                                  6

PAYLOAD_CAMERA_RECORD_SRC =                                             "C_V_REC"
class payload_camera_record_src(IntEnumBase):
    PAYLOAD_CAMERA_RECORD_BOTH =                                        0
    PAYLOAD_CAMERA_RECORD_EO   =                                        1
    PAYLOAD_CAMERA_RECORD_IR   =                                        2
    PAYLOAD_CAMERA_RECORD_OSD  =                                        5

PAYLOAD_CAMERA_IR_PALETTE =                                             "C_T_PALETTE"
class payload_camera_ir_palette(IntEnumBase):
    PAYLOAD_CAMERA_IR_PALETTE_1  =                                      0     #      F1: WhiteHot         |       G1: WhiteHot
    PAYLOAD_CAMERA_IR_PALETTE_2  =                                      1     #      F1: BlackHot         |       G1: Fulgurite
    PAYLOAD_CAMERA_IR_PALETTE_3  =                                      2     #      F1: Rainbow          |       G1: IronRed
    PAYLOAD_CAMERA_IR_PALETTE_4  =                                      3     #      F1: RainbowHC        |       G1: HotIron
    PAYLOAD_CAMERA_IR_PALETTE_5  =                                      4     #      F1: Ironbow          |       G1: Medical
    PAYLOAD_CAMERA_IR_PALETTE_6  =                                      5     #      F1: Lava             |       G1: Arctic
    PAYLOAD_CAMERA_IR_PALETTE_7  =                                      6     #      F1: Arctic           |       G1: Rainbow1
    PAYLOAD_CAMERA_IR_PALETTE_8  =                                      7     #      F1: Globow           |       G1: Rainbow2
    PAYLOAD_CAMERA_IR_PALETTE_9  =                                      8     #      F1: Gradedfire       |       G1: Tint
    PAYLOAD_CAMERA_IR_PALETTE_10 =                                      9     #      F1: Hottest          |       G1: BlackHot

PAYLOAD_CAMERA_IR_ZOOM_FACTOR =                                         "C_T_ZOOM"
class payload_camera_ir_zoom_factor(IntEnumBase):
    ZOOM_IR_1X =                                                        0
    ZOOM_IR_2X =                                                        1
    ZOOM_IR_3X =                                                        2
    ZOOM_IR_4X =                                                        3
    ZOOM_IR_5X =                                                        4
    ZOOM_IR_6X =                                                        5
    ZOOM_IR_7X =                                                        6
    ZOOM_IR_8X =                                                        7

class camera_zoom_value(FloatEnumBase):
    ZOOM_OUT  =                                                         -1
    ZOOM_STOP =                                                         0 
    ZOOM_IN   =                                                         1

class camera_focus_value(FloatEnumBase):
    FOCUS_OUT  =                                                        -1
    FOCUS_STOP =                                                        0
    FOCUS_IN   =                                                        1
    FOCUS_AUTO =                                                        2

PAYLOAD_CAMERA_VIDEO_ZOOM_FACTOR =                                      "C_V_ZOOM"
class payload_camera_video_zoom_factor(IntEnumBase):
    ZOOM_EO_1X  =                                                       0
    ZOOM_EO_2X  =                                                       1
    ZOOM_EO_3X  =                                                       2
    ZOOM_EO_4X  =                                                       3
    ZOOM_EO_5X  =                                                       4
    ZOOM_EO_6X  =                                                       5
    ZOOM_EO_7X  =                                                       6
    ZOOM_EO_8X  =                                                       7
    ZOOM_EO_9X  =                                                       8
    ZOOM_EO_10X =                                                       9
    ZOOM_EO_11X =                                                       10
    ZOOM_EO_12X =                                                       11

PAYLOAD_CAMERA_STORAGE =                                                "STORAGE"
class payload_camera_storage(IntEnumBase):
    PAYLOAD_CAMERA_STORAGE_INTERNAL =                                   0
    PAYLOAD_CAMERA_STORAGE_SDCARD   =                                   1

PAYLOAD_CAMERA_OBJECT_DETECTION =                                       "DETECTION_EN"
class payload_camera_object_detection(IntEnumBase):
    PAYLOAD_CAMERA_OBJECT_DETECTION_DISABLE =                           0                                        
    PAYLOAD_CAMERA_OBJECT_DETECTION_ENABLE  =                           1                                          

PAYLOAD_CAMERA_IR_ISOTHERMS =                                           "ISOTHERMS_EN"
class payload_camera_ir_isotherms(IntEnumBase):
    PAYLOAD_CAMERA_IR_ISOTHERMS_DISABLE =                               0                                        
    PAYLOAD_CAMERA_IR_ISOTHERMS_ENABLE  =                               1      

PAYLOAD_CAMERA_IR_ISOTHERMS_GAIN =                                      "ISOTHERMS_GAIN"
class payload_camera_ir_isotherms_gain(IntEnumBase):
    PAYLOAD_CAMERA_IR_ISOTHERMS_HIGH_GAIN =                             0                                        
    PAYLOAD_CAMERA_IR_ISOTHERMS_LOW_GAIN  =                             1

# ICR Mode: AutoICR vs Manual
PAYLOAD_CAMERA_VIDEO_ICR_MODE =                                         "C_V_ICR"
class payload_camera_video_icr_mode(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_ICR_MODE_AUTO   =                              2  # Auto ICR (excludes C_V_ICR_MAN)
    PAYLOAD_CAMERA_VIDEO_ICR_MODE_MANUAL =                              3  # Manual ICR (excludes C_V_ICR_THR)

# ICR Manual Control: On or Off
PAYLOAD_CAMERA_VIDEO_ICR_MANUAL =                                       "C_V_ICR_MAN"
class payload_camera_video_icr_manual(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_ICR_MANUAL_ON  =                               2  # ICR manual ON
    PAYLOAD_CAMERA_VIDEO_ICR_MANUAL_OFF =                               3  # ICR manual OFF

# ICR Threshold Value (0–255)
PAYLOAD_CAMERA_VIDEO_ICR_THRESHOLD      =                               "C_V_ICR_THR"
# Numeric range: 0 <= C_V_ICR_THR <= 255 (step = 1)