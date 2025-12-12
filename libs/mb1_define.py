from enum_base import IntEnumBase, FloatEnumBase

# MB1 Payload SDK Definitions
# This file contains definitions specific to MB1 payload model

# Zoom super resolution value can be set from 1x to 12x
PAYLOAD_CAMERA_VIDEO_ZOOM_FACTOR              =                         "C_V_ZOOM"
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

# OSD modes
PAYLOAD_CAMERA_VIDEO_OSD_MODE             =                             "OSD_MODE"
class payload_camera_osd_mode(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_OSD_MODE_DISABLE =                             0
    PAYLOAD_CAMERA_VIDEO_OSD_MODE_DEBUG   =                             1
    PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS  =                             2

# RC modes
PAYLOAD_CAMERA_RC_MODE                    =                             "RC_MODE"
class payload_camera_rc_mode(IntEnumBase):
    PAYLOAD_CAMERA_RC_MODE_GREMSY   =                                   0
    PAYLOAD_CAMERA_RC_MODE_STANDARD =                                   1

# Gimbal modes
PAYLOAD_CAMERA_GIMBAL_MODE             =                                "GB_MODE"
class payload_camera_gimbal_mode(IntEnumBase):
    PAYLOAD_CAMERA_GIMBAL_MODE_OFF     =                                0
    PAYLOAD_CAMERA_GIMBAL_MODE_LOCK    =                                1
    PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW  =                                2
    PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING =                                3
    PAYLOAD_CAMERA_GIMBAL_MODE_RESET   =                                4

# Camera sources
PAYLOAD_CAMERA_VIEW_SRC             =                                   "C_SOURCE"
class payload_camera_view_src(IntEnumBase):
    PAYLOAD_CAMERA_VIEW_EOIR         =                                  0
    PAYLOAD_CAMERA_VIEW_EO           =                                  1
    PAYLOAD_CAMERA_VIEW_IR           =                                  2
    PAYLOAD_CAMERA_VIEW_IREO         =                                  3
    PAYLOAD_CAMERA_VIEW_SYNC         =                                  4

# Camera record sources
PAYLOAD_CAMERA_RECORD_SRC            =                                  "C_V_REC"
class payload_camera_record_src(IntEnumBase):
    PAYLOAD_CAMERA_RECORD_BOTH =                                        0
    PAYLOAD_CAMERA_RECORD_EO   =                                        1
    PAYLOAD_CAMERA_RECORD_IR   =                                        2
    PAYLOAD_CAMERA_RECORD_OSD  =                                        5

# Storage selection
PAYLOAD_CAMERA_STORAGE                =                                 "STORAGE"
class payload_camera_storage(IntEnumBase):
    PAYLOAD_CAMERA_STORAGE_INTERNAL =                                   0
    PAYLOAD_CAMERA_STORAGE_SDCARD   =                                   1

# IR palettes
PAYLOAD_CAMERA_IR_PALETTE        =                                      "C_T_PALETTE"
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

# Object Detection
PAYLOAD_CAMERA_OBJECT_DETECTION              =                          "DETECTION_EN"
class payload_camera_object_detection(IntEnumBase):
    PAYLOAD_CAMERA_OBJECT_DETECTION_DISABLE  =                          0
    PAYLOAD_CAMERA_OBJECT_DETECTION_ENABLE   =                          1

# IR Isotherms
PAYLOAD_CAMERA_IR_ISOTHERMS              =                              "ISOTHERMS_EN"
class payload_camera_ir_isotherms(IntEnumBase):
    PAYLOAD_CAMERA_IR_ISOTHERMS_DISABLE  =                              0
    PAYLOAD_CAMERA_IR_ISOTHERMS_ENABLE   =                              1

# IR Isotherms Gain
PAYLOAD_CAMERA_IR_ISOTHERMS_GAIN              =                         "ISOTHERMS_GAIN"
class payload_camera_ir_isotherms_gain(IntEnumBase):
    PAYLOAD_CAMERA_IR_ISOTHERMS_HIGH_GAIN =                             0
    PAYLOAD_CAMERA_IR_ISOTHERMS_LOW_GAIN  =                             1

# Zoom value can be set from 1x to 8x for camera thermal
PAYLOAD_CAMERA_IR_ZOOM_FACTOR    =                                      "C_T_ZOOM"
class payload_camera_ir_zoom_factor(IntEnumBase):
    ZOOM_IR_1X =                                                        0
    ZOOM_IR_2X =                                                        1
    ZOOM_IR_3X =                                                        2
    ZOOM_IR_4X =                                                        3
    ZOOM_IR_5X =                                                        4
    ZOOM_IR_6X =                                                        5
    ZOOM_IR_7X =                                                        6
    ZOOM_IR_8X =                                                        7

class camera_zoom_value(IntEnumBase):
    ZOOM_OUT  =                                                         -1
    ZOOM_STOP =                                                         0
    ZOOM_IN   =                                                         1

class camera_focus_value(IntEnumBase):
    FOCUS_OUT  =                                                        -1
    FOCUS_STOP =                                                        0
    FOCUS_IN   =                                                        1
    FOCUS_AUTO =                                                        2
