from enum_base import IntEnumBase, FloatEnumBase

# Tracking modes
PAYLOAD_CAMERA_TRACKING_MODE              =                             "TRACK_MODE"
class payload_camera_tracking_mode(IntEnumBase):
    PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING  =                             0
    PAYLOAD_CAMERA_TRACKING_OBJ_DETECTION =                             1

# RC modes
PAYLOAD_CAMERA_RC_MODE                    =                             "RC_MODE"
class payload_camera_rc_mode(IntEnumBase):
    PAYLOAD_CAMERA_RC_MODE_GREMSY   =                                   0
    PAYLOAD_CAMERA_RC_MODE_STANDARD =                                   1

# Camera sources
PAYLOAD_CAMERA_VIEW_SRC             =                                   "C_SOURCE"
class payload_camera_view_src(IntEnumBase):
    PAYLOAD_CAMERA_VIEW_EOIR         =                                  0
    PAYLOAD_CAMERA_VIEW_EO           =                                  1
    PAYLOAD_CAMERA_VIEW_IR           =                                  2
    PAYLOAD_CAMERA_VIEW_IREO         =                                  3
    PAYLOAD_CAMERA_VIEW_SYNC         =                                  4
    PAYLOAD_CAMERA_VIEW_SIDE_BY_SIDE =                                  6

# Camera record sources    
PAYLOAD_CAMERA_RECORD_SRC            =                                  "C_V_REC"
class payload_camera_record_src(IntEnumBase):
    PAYLOAD_CAMERA_RECORD_BOTH =                                        0
    PAYLOAD_CAMERA_RECORD_EO   =                                        1
    PAYLOAD_CAMERA_RECORD_IR   =                                        2
    PAYLOAD_CAMERA_RECORD_OSD  =                                        5

# OSD modes
PAYLOAD_CAMERA_VIDEO_OSD_MODE             =                             "OSD_MODE"
class payload_camera_osd_mode(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_OSD_MODE_DISABLE =                             0
    PAYLOAD_CAMERA_VIDEO_OSD_MODE_DEBUG   =                             1
    PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS  =                             2

# Image flip
PAYLOAD_CAMERA_VIDEO_FLIP                 =                             "C_V_FLIP"
class payload_camera_video_flip(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_FLIP_ON  =                                     2
    PAYLOAD_CAMERA_VIDEO_FLIP_OFF =                                     3

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

# IR Zoom value can be set from 1x to 8x for camera thermal
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

# EO zoom modes
PAYLOAD_CAMERA_VIDEO_ZOOM_MODE                      =                   "C_V_ZM_MODE"
class payload_camera_video_zoom_mode(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_COMBINE          =                   0
    PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION =                   2

# Combine Zoom levels can be set from 1x to 240x
PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_FACTOR            =                   "C_V_ZM_CB_LV"
class payload_camera_video_zoom_combine_factor(IntEnumBase):
    ZOOM_COMBINE_1X     =                                               0
    ZOOM_COMBINE_2X     =                                               1
    ZOOM_COMBINE_4X     =                                               2
    ZOOM_COMBINE_6X     =                                               3
    ZOOM_COMBINE_8X     =                                               4
    ZOOM_COMBINE_10X    =                                               5
    ZOOM_COMBINE_12X    =                                               6
    ZOOM_COMBINE_14X    =                                               7
    ZOOM_COMBINE_16X    =                                               8
    ZOOM_COMBINE_18X    =                                               9
    ZOOM_COMBINE_20X    =                                               10
    ZOOM_COMBINE_40X    =                                               11
    ZOOM_COMBINE_60X    =                                               12
    ZOOM_COMBINE_80X    =                                               13
    ZOOM_COMBINE_100X   =                                               14
    ZOOM_COMBINE_120X   =                                               15
    ZOOM_COMBINE_140X   =                                               16
    ZOOM_COMBINE_160X   =                                               17
    ZOOM_COMBINE_180X   =                                               18
    ZOOM_COMBINE_200X   =                                               19
    ZOOM_COMBINE_220X   =                                               20
    ZOOM_COMBINE_240X   =                                               21

# Super Resolution Zoom levels can be set from 1x to 30x
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
    ZOOM_SUPER_RESOLUTION_30X =                                         11

# Image freeze
PAYLOAD_CAMERA_EO_FREEZE                =                               "C_V_FREEZE"
class payload_camera_eo_freeze(IntEnumBase):
    PAYLOAD_CAMERA_EO_FREEZE_ON         =                               2  
    PAYLOAD_CAMERA_EO_FREEZE_OFF        =                               3  

# Defog modes
PAYLOAD_CAMERA_VIDEO_DEFOG         =                                    "C_V_DEFOG"
class payload_camera_video_defog(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_DEFOG_ON  =                                    2
    PAYLOAD_CAMERA_VIDEO_DEFOG_OFF =                                    3

# Defog levels
PAYLOAD_CAMERA_VIDEO_DEFOG_LEVEL      =                                 "C_V_DEFOG_LV"
class payload_camera_video_defog_level(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_DEFOG_LOWEST =                                 0
    PAYLOAD_CAMERA_VIDEO_DEFOG_LOW    =                                 1
    PAYLOAD_CAMERA_VIDEO_DEFOG_MID    =                                 2
    PAYLOAD_CAMERA_VIDEO_DEFOG_HIGH   =                                 3

# EO High Sensitivity
PAYLOAD_CAMERA_EO_HS                    =                               "C_V_HS"
class payload_camera_eo_hs(IntEnumBase):
    PAYLOAD_CAMERA_EO_HS_ON             =                               2  
    PAYLOAD_CAMERA_EO_HS_OFF            =                               3

# Exposure modes
PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE        =                             "C_V_AE"
class payload_camera_video_auto_exposure(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_AUTO    =                             0
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_MANUAL  =                             3
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_SHUTTER =                             10
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_IRIS    =                             11
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_BRIGHT  =                             13
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_GAIN    =                             14

# Shutter speeds
PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED              =                       "C_V_SP"
class payload_camera_video_shutter_speed(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1      =                       6
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_2_3      =                       7
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_2      =                       8
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_3      =                       9
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_4      =                       10
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_6      =                       11
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_8      =                       12
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_10     =                       13
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_15     =                       14
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_20     =                       15
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_30     =                       16
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_50     =                       17
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_60     =                       18
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_90     =                       19
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_100    =                       20
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_125    =                       21
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_180    =                       22
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_250    =                       23
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_350    =                       24
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_500    =                       25
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_725    =                       26
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1000   =                       27
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1500   =                       28
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_2000   =                       29
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_3000   =                       30
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_4000   =                       31
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_6000   =                       32
    PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_10000  =                       33

# EO shutter min limit
PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT                 =                   "C_V_MinSP"
class payload_camera_eo_shutter_min_limit(IntEnumBase):
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_10        =                   13
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_15        =                   14
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_20        =                   15
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_30        =                   16
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_50        =                   17
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_60        =                   18
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_90        =                   19
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_100       =                   20
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_125       =                   21
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_180       =                   22
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_250       =                   23
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_350       =                   24
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_500       =                   25
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_725       =                   26
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_1000      =                   27
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_1500      =                   28
    PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_2000      =                   29 

# Aperture values
PAYLOAD_CAMERA_VIDEO_APERTURE_VALUE         =                           "C_V_IrP"
class payload_camera_video_aperture_value(IntEnumBase):
    PAYLOAD_CAMERA_EO_APERTURE_F2_0         =                           25
    PAYLOAD_CAMERA_EO_APERTURE_F2_2         =                           24
    PAYLOAD_CAMERA_EO_APERTURE_F2_4         =                           23
    PAYLOAD_CAMERA_EO_APERTURE_F2_6         =                           22
    PAYLOAD_CAMERA_EO_APERTURE_F2_8         =                           21
    PAYLOAD_CAMERA_EO_APERTURE_F3_1         =                           20
    PAYLOAD_CAMERA_EO_APERTURE_F3_4         =                           19
    PAYLOAD_CAMERA_EO_APERTURE_F4_0         =                           17
    PAYLOAD_CAMERA_EO_APERTURE_F5_2         =                           14
    PAYLOAD_CAMERA_EO_APERTURE_F6_8         =                           11
    PAYLOAD_CAMERA_EO_APERTURE_F7_3         =                           10
    PAYLOAD_CAMERA_EO_APERTURE_F8_7         =                           8
    PAYLOAD_CAMERA_EO_APERTURE_F9_6         =                           7
    PAYLOAD_CAMERA_EO_APERTURE_F10_0        =                           6
    PAYLOAD_CAMERA_EO_APERTURE_F11_0        =                           5

# EO white-balance modes
PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE              =                       "C_V_WB"
class payload_camera_video_white_balance(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_AUTO     =                       0
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_INDOOR   =                       1
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_OUTDOOR  =                       2
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_ONE_PUSH =                       3
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_ATW      =                       4
    PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_MANUAL   =                       5

# EO R gains, values can be set from 0 to 255, step 1, in case of PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE set to PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_MANUAL
PAYLOAD_CAMERA_EO_R_GAIN                         =                      "C_V_RGAIN"

# EO B gains, values can be set from 0 to 255, step 1, in case of PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE set to PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_MANUAL
PAYLOAD_CAMERA_EO_B_GAIN                         =                      "C_V_BGAIN" 

# EO focus modes
PAYLOAD_CAMERA_VIDEO_FOCUS_MODE                  =                      "C_V_FM"
class payload_camera_video_focus_mode(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_MANUAL       =                      0
    PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_ZOOM_TRIGGER =                      1
    PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_NEAR    =                      2
    PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_FAR     =                      3


# EO Manual focus value can be set from 0 to 61440, step 10
PAYLOAD_CAMERA_VIDEO_FOCUS_VALUE            =                           "C_V_FV"

# EO ICR modes
PAYLOAD_CAMERA_EO_ICR_MODE                  =                           "C_V_ICR"
class payload_camera_eo_icr_mode(IntEnumBase):
    PAYLOAD_CAMERA_EO_ICR_MODE_AUTO         =                           2
    PAYLOAD_CAMERA_EO_ICR_MODE_MANUAL       =                           3 

# EO ICR AUTO threshold, values can be set from 0 to 255, step 1
PAYLOAD_CAMERA_EO_ICR_MODE_AUTO_THRESHOLD   =                           "C_V_ICR_THR"

# EO ICR MANUAL modes
PAYLOAD_CAMERA_EO_ICR_MANUAL                =                           "C_V_ICR_MAN"
class payload_camera_eo_icr_manual(IntEnumBase):
    PAYLOAD_CAMERA_EO_ICR_MANUAL_ON         =                           2
    PAYLOAD_CAMERA_EO_ICR_MANUAL_OFF        =                           3 

# Gimbal modes
PAYLOAD_CAMERA_GIMBAL_MODE             =                                "GB_MODE"
class payload_camera_gimbal_mode(IntEnumBase):
    PAYLOAD_CAMERA_GIMBAL_MODE_OFF     =                                0
    PAYLOAD_CAMERA_GIMBAL_MODE_LOCK    =                                1
    PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW  =                                2
    PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING =                                3
    PAYLOAD_CAMERA_GIMBAL_MODE_RESET   =                                4

# LRF modes
PAYLOAD_LRF_MODE                =                                       "LRF_MODE"
class payload_lrf_mode(IntEnumBase):
    PAYLOAD_LRF_MODE_OFF        =                                       3
    PAYLOAD_LRF_MODE_1HZ        =                                       0
    PAYLOAD_LRF_MODE_4HZ        =                                       1
    PAYLOAD_LRF_MODE_10HZ       =                                       2 
    
class camera_zoom_value(IntEnumBase):
    ZOOM_OUT  =                                                         -1
    ZOOM_STOP =                                                         0 
    ZOOM_IN   =                                                         1

class camera_focus_value(IntEnumBase):
    FOCUS_OUT  =                                                        -1
    FOCUS_STOP =                                                        0
    FOCUS_IN   =                                                        1
    FOCUS_AUTO =                                                        2