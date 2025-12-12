from enum_base import IntEnumBase, FloatEnumBase

# ZIO Payload SDK Definitions
# This file contains definitions specific to ZIO payload model

# Object Detection/Tracking modes
PAYLOAD_CAMERA_OBJECT_DETECTION              =                          "TRACK_MODE"
class payload_camera_object_detection(IntEnumBase):
    PAYLOAD_CAMERA_OBJECT_DETECTION_DISABLE  =                          0
    PAYLOAD_CAMERA_OBJECT_DETECTION_ENABLE   =                          1

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

# Image flip
PAYLOAD_CAMERA_VIDEO_FLIP                 =                             "C_V_FLIP"
class payload_camera_video_flip(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_FLIP_ON  =                                     2
    PAYLOAD_CAMERA_VIDEO_FLIP_OFF =                                     3

# Image freeze
PAYLOAD_CAMERA_EO_FREEZE                =                               "C_V_FREEZE"
class payload_camera_eo_freeze(IntEnumBase):
    PAYLOAD_CAMERA_EO_FREEZE_ON         =                               2
    PAYLOAD_CAMERA_EO_FREEZE_OFF        =                               3

# Defog modes
PAYLOAD_CAMERA_VIDEO_DEFOG         =                                    "C_V_DEFOG"
class payload_camera_video_defog(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_DEFOG_OFF =                                    0
    PAYLOAD_CAMERA_VIDEO_DEFOG_ON  =                                    1

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
    PAYLOAD_CAMERA_EO_HS_OFF            =                               0
    PAYLOAD_CAMERA_EO_HS_ON             =                               1

# Exposure modes
PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE        =                             "C_V_AE"
class payload_camera_video_auto_exposure(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_AUTO    =                             0
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_MANUAL  =                             3
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_SHUTTER =                             10
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_IRIS    =                             11
    PAYLOAD_CAMERA_VIDEO_EXPOSURE_BRIGHT  =                             13

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

# EO Gain in High Sensitivity mode, PAYLOAD_CAMERA_EO_HS = PAYLOAD_CAMERA_EO_HS_ON
PAYLOAD_CAMERA_EO_GAIN_HS                   =                           "C_V_GAIN_HS"
class payload_camera_eo_gain_hs(IntEnumBase):
    PAYLOAD_CAMERA_EO_GAIN_HS_48DB          =                           17
    PAYLOAD_CAMERA_EO_GAIN_HS_45DB          =                           16
    PAYLOAD_CAMERA_EO_GAIN_HS_42DB          =                           15
    PAYLOAD_CAMERA_EO_GAIN_HS_39DB          =                           14
    PAYLOAD_CAMERA_EO_GAIN_HS_36DB          =                           13
    PAYLOAD_CAMERA_EO_GAIN_HS_33DB          =                           12
    PAYLOAD_CAMERA_EO_GAIN_HS_30DB          =                           11
    PAYLOAD_CAMERA_EO_GAIN_HS_27DB          =                           10
    PAYLOAD_CAMERA_EO_GAIN_HS_24DB          =                           9
    PAYLOAD_CAMERA_EO_GAIN_HS_21DB          =                           8
    PAYLOAD_CAMERA_EO_GAIN_HS_18DB          =                           7
    PAYLOAD_CAMERA_EO_GAIN_HS_15DB          =                           6
    PAYLOAD_CAMERA_EO_GAIN_HS_12DB          =                           5
    PAYLOAD_CAMERA_EO_GAIN_HS_9DB           =                           4
    PAYLOAD_CAMERA_EO_GAIN_HS_6DB           =                           3
    PAYLOAD_CAMERA_EO_GAIN_HS_3DB           =                           2
    PAYLOAD_CAMERA_EO_GAIN_HS_0DB           =                           1

# EO Gain in Low Sensitivity mode, PAYLOAD_CAMERA_EO_HS = PAYLOAD_CAMERA_EO_HS_OFF
PAYLOAD_CAMERA_EO_GAIN_LS                   =                           "C_V_GAIN_LS"
class payload_camera_eo_gain_ls(IntEnumBase):
    PAYLOAD_CAMERA_EO_GAIN_LS_36DB          =                           13
    PAYLOAD_CAMERA_EO_GAIN_LS_33DB          =                           12
    PAYLOAD_CAMERA_EO_GAIN_LS_30DB          =                           11
    PAYLOAD_CAMERA_EO_GAIN_LS_27DB          =                           10
    PAYLOAD_CAMERA_EO_GAIN_LS_24DB          =                           9
    PAYLOAD_CAMERA_EO_GAIN_LS_21DB          =                           8
    PAYLOAD_CAMERA_EO_GAIN_LS_18DB          =                           7
    PAYLOAD_CAMERA_EO_GAIN_LS_15DB          =                           6
    PAYLOAD_CAMERA_EO_GAIN_LS_12DB          =                           5
    PAYLOAD_CAMERA_EO_GAIN_LS_9DB           =                           4
    PAYLOAD_CAMERA_EO_GAIN_LS_6DB           =                           3
    PAYLOAD_CAMERA_EO_GAIN_LS_3DB           =                           2
    PAYLOAD_CAMERA_EO_GAIN_LS_0DB           =                           1

# EO Bright in High Sensitivity mode, PAYLOAD_CAMERA_EO_HS = PAYLOAD_CAMERA_EO_HS_ON
# values can be set from 0 to 41, step 1
PAYLOAD_CAMERA_VIDEO_BRIGHT_HS_VALUE              =                     "C_V_BrP_HS"

# EO Bright in Low Sensitivity mode, PAYLOAD_CAMERA_EO_HS = PAYLOAD_CAMERA_EO_HS_OFF
# values can be set from 0 to 37, step 1
PAYLOAD_CAMERA_VIDEO_BRIGHT_LS_VALUE              =                     "C_V_BrP_LS"

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

# EO zoom modes
PAYLOAD_CAMERA_VIDEO_ZOOM_MODE                      =                   "C_V_ZM_MODE"
class payload_camera_video_zoom_mode(IntEnumBase):
    PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_COMBINE          =                   0
    PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION =                   2

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
    ZOOM_SUPER_RESOLUTION_22X =                                         11
    ZOOM_SUPER_RESOLUTION_24X =                                         12
    ZOOM_SUPER_RESOLUTION_26X =                                         13
    ZOOM_SUPER_RESOLUTION_28X =                                         14
    ZOOM_SUPER_RESOLUTION_30X =                                         15

# Combine Zoom levels can be set from 1x to 240x
PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_FACTOR            =                   "C_V_ZM_CB_LV"
class payload_camera_video_zoom_combine_factor(IntEnumBase):
    ZOOM_COMBINE_1X     =                                               0
    ZOOM_COMBINE_10X    =                                               1
    ZOOM_COMBINE_20X    =                                               2
    ZOOM_COMBINE_40X    =                                               3
    ZOOM_COMBINE_80X    =                                               4
    ZOOM_COMBINE_120X   =                                               5
    ZOOM_COMBINE_240X   =                                               6

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

class camera_zoom_value(IntEnumBase):
    ZOOM_OUT  =                                                         -1
    ZOOM_STOP =                                                         0
    ZOOM_IN   =                                                         1

class camera_focus_value(IntEnumBase):
    FOCUS_OUT  =                                                        -1
    FOCUS_STOP =                                                        0
    FOCUS_IN   =                                                        1
    FOCUS_AUTO =                                                        2
