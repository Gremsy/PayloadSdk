from enum_base import IntEnumBase, FloatEnumBase

# MB1 Payload SDK Definitions
# This file contains definitions specific to MB1 payload model
# Complete port from mb1_sdk.h
#
# MB1 Payload Variants:
# - Ghadron: Uses FLIR IR sensor, supports advanced Isotherm regions (RG0-RG5)
# - Infisignt: Uses Guide IR sensor, supports simple Isotherm threshold
# - Lynx: Uses Guide IR sensor, supports simple Isotherm threshold, EO zoom 1x-10x

# ============================================================================
# GENERAL PAYLOAD SETTINGS
# ============================================================================

# Camera Mode (Photo/Video) - Internal control parameter
PAYLOAD_CAMERA_CAM_MODE                   =                             "CAM_MODE"
class payload_camera_cam_mode(IntEnumBase):
    PAYLOAD_CAMERA_CAM_MODE_PHOTO         =                             0
    PAYLOAD_CAMERA_CAM_MODE_VIDEO         =                             1

# Setting Target - Select which device to configure
PAYLOAD_CAMERA_SETTING_TARGET             =                             "SETTING_TARGET"
class payload_camera_setting_target(IntEnumBase):
    PAYLOAD_CAMERA_SETTING_TARGET_EO      =                             0
    PAYLOAD_CAMERA_SETTING_TARGET_IR      =                             1
    PAYLOAD_CAMERA_SETTING_TARGET_GIMBAL  =                             2

# EO Zoom and Focus Speed
PAYLOAD_CAMERA_EO_ZOOM_SPEED              =                             "C_V_ZOOM_SPEED"
PAYLOAD_CAMERA_EO_FOCUS_SPEED             =                             "C_V_FOCUS_SPEED"

# EO Zoom factor
# Ghadron/Infisignt: 1x-12x, Lynx: 1x-10x
PAYLOAD_CAMERA_VIDEO_ZOOM_FACTOR          =                             "C_V_ZOOM"
class payload_camera_video_zoom_factor(IntEnumBase):
    ZOOM_EO_1X  =                                                        0
    ZOOM_EO_2X  =                                                        1
    ZOOM_EO_3X  =                                                        2
    ZOOM_EO_4X  =                                                        3
    ZOOM_EO_5X  =                                                        4
    ZOOM_EO_6X  =                                                        5
    ZOOM_EO_7X  =                                                        6
    ZOOM_EO_8X  =                                                        7
    ZOOM_EO_9X  =                                                        8
    ZOOM_EO_10X =                                                        9   # Lynx max
    ZOOM_EO_11X =                                                        10  # Ghadron/Infisignt only
    ZOOM_EO_12X =                                                        11  # Ghadron/Infisignt only

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

# Alternative storage type definition (used in some payloads)
PAYLOAD_CAMERA_STORAGE_TYPE           =                                 "STORAGE_TYPE"
class payload_camera_storage_type(IntEnumBase):
    PAYLOAD_CAMERA_STORAGE_TYPE_INTERNAL =                              0
    PAYLOAD_CAMERA_STORAGE_TYPE_SDCARD   =                              1

# IR palettes
PAYLOAD_CAMERA_IR_PALETTE        =                                      "C_T_PALETTE"
class payload_camera_ir_palette(IntEnumBase):
    # Ghadron (FLIR sensor) palette values:
    PAYLOAD_CAMERA_IR_PALETTE_1  =                                      0     # Ghadron: WhiteHot    | Infisignt/Lynx: WhiteHot
    PAYLOAD_CAMERA_IR_PALETTE_2  =                                      1     # Ghadron: BlackHot    | Infisignt/Lynx: Fulgurite
    PAYLOAD_CAMERA_IR_PALETTE_3  =                                      2     # Ghadron: Rainbow     | Infisignt/Lynx: IronRed
    PAYLOAD_CAMERA_IR_PALETTE_4  =                                      3     # Ghadron: RainbowHC   | Infisignt/Lynx: HotIron
    PAYLOAD_CAMERA_IR_PALETTE_5  =                                      4     # Ghadron: Ironbow     | Infisignt/Lynx: Medical
    PAYLOAD_CAMERA_IR_PALETTE_6  =                                      5     # Ghadron: Lava        | Infisignt/Lynx: Arctic
    PAYLOAD_CAMERA_IR_PALETTE_7  =                                      6     # Ghadron: Arctic      | Infisignt/Lynx: Rainbow1
    PAYLOAD_CAMERA_IR_PALETTE_8  =                                      7     # Ghadron: Globow      | Infisignt/Lynx: Rainbow2
    PAYLOAD_CAMERA_IR_PALETTE_9  =                                      8     # Ghadron: Gradedfire  | Infisignt/Lynx: Tint
    PAYLOAD_CAMERA_IR_PALETTE_10 =                                      9     # Ghadron: Hottest     | Infisignt/Lynx: BlackHot

# Ghadron-specific palette aliases (FLIR sensor)
class payload_camera_ir_palette_ghadron(IntEnumBase):
    PAYLOAD_CAMERA_IR_PALETTE_WHITEHOT   =                              0
    PAYLOAD_CAMERA_IR_PALETTE_BLACKHOT   =                              1
    PAYLOAD_CAMERA_IR_PALETTE_RAINBOW    =                              2
    PAYLOAD_CAMERA_IR_PALETTE_RAINBOWHC  =                              3
    PAYLOAD_CAMERA_IR_PALETTE_IRONBOW    =                              4
    PAYLOAD_CAMERA_IR_PALETTE_LAVA       =                              5
    PAYLOAD_CAMERA_IR_PALETTE_ARCTIC     =                              6
    PAYLOAD_CAMERA_IR_PALETTE_GLOBOW     =                              7
    PAYLOAD_CAMERA_IR_PALETTE_GRADEDFIRE =                              8
    PAYLOAD_CAMERA_IR_PALETTE_HOTTEST    =                              9

# Infisignt/Lynx-specific palette aliases (Guide sensor)
class payload_camera_ir_palette_guide(IntEnumBase):
    PAYLOAD_CAMERA_IR_PALETTE_WHITEHOT   =                              0
    PAYLOAD_CAMERA_IR_PALETTE_FULGURITE  =                              1
    PAYLOAD_CAMERA_IR_PALETTE_IRONRED    =                              2
    PAYLOAD_CAMERA_IR_PALETTE_HOTIRON    =                              3
    PAYLOAD_CAMERA_IR_PALETTE_MEDICAL    =                              4
    PAYLOAD_CAMERA_IR_PALETTE_ARCTIC     =                              5
    PAYLOAD_CAMERA_IR_PALETTE_RAINBOW1   =                              6
    PAYLOAD_CAMERA_IR_PALETTE_RAINBOW2   =                              7
    PAYLOAD_CAMERA_IR_PALETTE_TINT       =                              8
    PAYLOAD_CAMERA_IR_PALETTE_BLACKHOT   =                              9

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

# ============================================================================
# IR CAMERA ADVANCED SETTINGS
# ============================================================================

# IR Gain Mode (Temperature Range)
PAYLOAD_CAMERA_IR_GAIN                      =                           "C_T_G"
class payload_camera_ir_gain(IntEnumBase):
    PAYLOAD_CAMERA_IR_GAIN_LOW              =                           0    # -50°C~150°C or -20°C~150°C
    PAYLOAD_CAMERA_IR_GAIN_HIGH             =                           1    # -50°C~550°C or -20°C~550°C

# IR Contrast Mode
PAYLOAD_CAMERA_IR_CONTRAST_MODE             =                           "C_T_CONST_M"
class payload_camera_ir_contrast_mode(IntEnumBase):
    PAYLOAD_CAMERA_IR_CONTRAST_MODE_DEFAULT =                           0
    PAYLOAD_CAMERA_IR_CONTRAST_MODE_CUSTOM  =                           1

# IR AGC (Automatic Gain Control) Mode
PAYLOAD_CAMERA_IR_AGC_MODE                  =                           "C_T_AGC_M"
class payload_camera_ir_agc_mode(IntEnumBase):
    PAYLOAD_CAMERA_IR_AGC_MODE_NORMAL       =                           0
    PAYLOAD_CAMERA_IR_AGC_MODE_HOLD         =                           1
    PAYLOAD_CAMERA_IR_AGC_MODE_THRESHOLD    =                           2
    PAYLOAD_CAMERA_IR_AGC_MODE_BRIGHT       =                           3
    PAYLOAD_CAMERA_IR_AGC_MODE_LINEAR       =                           4
    PAYLOAD_CAMERA_IR_AGC_MODE_MANUAL       =                           5

# IR AGC Linear Percent (0-100, step 10)
PAYLOAD_CAMERA_IR_AGC_LINEAR_PERCENT        =                           "C_T_AGC_LNP"

# IR SpotMeter Mode
PAYLOAD_CAMERA_IR_SPOTMETER_MODE            =                           "C_T_SPOT_M"
class payload_camera_ir_spotmeter_mode(IntEnumBase):
    PAYLOAD_CAMERA_IR_SPOTMETER_MODE_DISABLE =                          0
    PAYLOAD_CAMERA_IR_SPOTMETER_MODE_ENABLE  =                          1

# IR SpotMeter Units
PAYLOAD_CAMERA_IR_SPOTMETER_UNITS           =                           "C_T_SPOT_U"
class payload_camera_ir_spotmeter_units(IntEnumBase):
    PAYLOAD_CAMERA_IR_SPOTMETER_UNITS_CELSIUS    =                      0
    PAYLOAD_CAMERA_IR_SPOTMETER_UNITS_FAHRENHEIT =                      1
    PAYLOAD_CAMERA_IR_SPOTMETER_UNITS_KELVIN     =                      2

# IR SpotMeter Size (16-128, step 4)
PAYLOAD_CAMERA_IR_SPOTMETER_SIZE            =                           "C_T_SPOT_S"

# IR Isotherm Mode (Advanced)
PAYLOAD_CAMERA_IR_ISOTHERM_MODE             =                           "C_T_ISO_M"
class payload_camera_ir_isotherm_mode(IntEnumBase):
    PAYLOAD_CAMERA_IR_ISOTHERM_MODE_DISABLE =                           0
    PAYLOAD_CAMERA_IR_ISOTHERM_MODE_ENABLE  =                           1

# IR Isotherm Units
PAYLOAD_CAMERA_IR_ISOTHERM_UNITS            =                           "C_T_ISO_U"
class payload_camera_ir_isotherm_units(IntEnumBase):
    PAYLOAD_CAMERA_IR_ISOTHERM_UNITS_KELVIN     =                       0
    PAYLOAD_CAMERA_IR_ISOTHERM_UNITS_CELSIUS    =                       1
    PAYLOAD_CAMERA_IR_ISOTHERM_UNITS_FAHRENHEIT =                       2
    PAYLOAD_CAMERA_IR_ISOTHERM_UNITS_PERCENT    =                       3
    PAYLOAD_CAMERA_IR_ISOTHERM_UNITS_COUNTS     =                       4

# IR Isotherm Threshold (0-150, step 5) - Used in Infisignt/Lynx
PAYLOAD_CAMERA_IR_ISOTHERM_THRESHOLD        =                           "C_T_ISO_THR"

# IR Isotherm Region 0 Settings
PAYLOAD_CAMERA_IR_ISOTHERM_RG0_TEMP         =                           "C_T_RG0_TMP"      # -1000~1000, step 10
PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MODE   =                           "C_T_RG0_CLR_M"
class payload_camera_ir_isotherm_rg_color_mode(IntEnumBase):
    PAYLOAD_CAMERA_IR_ISOTHERM_RG_COLOR_MODE_DISABLE    =                0
    PAYLOAD_CAMERA_IR_ISOTHERM_RG_COLOR_MODE_STANDARD   =                1
    PAYLOAD_CAMERA_IR_ISOTHERM_RG_COLOR_MODE_LINEAR_RGB =                2
    PAYLOAD_CAMERA_IR_ISOTHERM_RG_COLOR_MODE_LINEAR_HSV =                3
    PAYLOAD_CAMERA_IR_ISOTHERM_RG_COLOR_MODE_NON_LINEAR =                4
    PAYLOAD_CAMERA_IR_ISOTHERM_RG_COLOR_MODE_SINGLE     =                5

PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MIN    =                           "C_T_RG0_CLR_MN"
PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MAX    =                           "C_T_RG0_CLR_MX"

# IR Isotherm Region 1 Settings
PAYLOAD_CAMERA_IR_ISOTHERM_RG1_TEMP         =                           "C_T_RG1_TMP"
PAYLOAD_CAMERA_IR_ISOTHERM_RG1_COLOR_MODE   =                           "C_T_RG1_CLR_M"
PAYLOAD_CAMERA_IR_ISOTHERM_RG1_COLOR_MIN    =                           "C_T_RG1_CLR_MN"
PAYLOAD_CAMERA_IR_ISOTHERM_RG1_COLOR_MAX    =                           "C_T_RG1_CLR_MX"

# IR Isotherm Region 2 Settings
PAYLOAD_CAMERA_IR_ISOTHERM_RG2_TEMP         =                           "C_T_RG2_TMP"
PAYLOAD_CAMERA_IR_ISOTHERM_RG2_COLOR_MODE   =                           "C_T_RG2_CLR_M"
PAYLOAD_CAMERA_IR_ISOTHERM_RG2_COLOR_MIN    =                           "C_T_RG2_CLR_MN"
PAYLOAD_CAMERA_IR_ISOTHERM_RG2_COLOR_MAX    =                           "C_T_RG2_CLR_MX"

# IR Isotherm Region 3 Settings
PAYLOAD_CAMERA_IR_ISOTHERM_RG3_TEMP         =                           "C_T_RG3_TMP"
PAYLOAD_CAMERA_IR_ISOTHERM_RG3_COLOR_MODE   =                           "C_T_RG3_CLR_M"
PAYLOAD_CAMERA_IR_ISOTHERM_RG3_COLOR_MIN    =                           "C_T_RG3_CLR_MN"
PAYLOAD_CAMERA_IR_ISOTHERM_RG3_COLOR_MAX    =                           "C_T_RG3_CLR_MX"

# IR Isotherm Region 4 Settings
PAYLOAD_CAMERA_IR_ISOTHERM_RG4_TEMP         =                           "C_T_RG4_TMP"
PAYLOAD_CAMERA_IR_ISOTHERM_RG4_COLOR_MODE   =                           "C_T_RG4_CLR_M"
PAYLOAD_CAMERA_IR_ISOTHERM_RG4_COLOR_MIN    =                           "C_T_RG4_CLR_MN"
PAYLOAD_CAMERA_IR_ISOTHERM_RG4_COLOR_MAX    =                           "C_T_RG4_CLR_MX"

# IR Isotherm Region 5 Settings (no temperature setting for RG5)
PAYLOAD_CAMERA_IR_ISOTHERM_RG5_COLOR_MODE   =                           "C_T_RG5_CLR_M"
PAYLOAD_CAMERA_IR_ISOTHERM_RG5_COLOR_MIN    =                           "C_T_RG5_CLR_MN"
PAYLOAD_CAMERA_IR_ISOTHERM_RG5_COLOR_MAX    =                           "C_T_RG5_CLR_MX"

# Isotherm Color Values (for all regions)
class isotherm_color(IntEnumBase):
    ISOTHERM_COLOR_ICE_BLUE     =                                       0
    ISOTHERM_COLOR_SKY_BLUE     =                                       1
    ISOTHERM_COLOR_LIGHT_GREEN  =                                       2
    ISOTHERM_COLOR_YELLOW_GREEN =                                       3
    ISOTHERM_COLOR_YELLOW       =                                       4
    ISOTHERM_COLOR_ORANGE       =                                       5
    ISOTHERM_COLOR_RED          =                                       6

# ============================================================================
# EO CAMERA SETTINGS
# ============================================================================

# EO Image Flip
PAYLOAD_CAMERA_EO_FLIP                      =                           "C_V_FLIP"
PAYLOAD_CAMERA_VIDEO_FLIP                   =                           PAYLOAD_CAMERA_EO_FLIP
class payload_camera_eo_flip(IntEnumBase):
    PAYLOAD_CAMERA_EO_FLIP_ON               =                           2
    PAYLOAD_CAMERA_EO_FLIP_OFF              =                           3

# EO Camera Scene Mode (Optimization)
PAYLOAD_CAMERA_EO_SCENE_MODE                =                           "C_G_SCENE"
class payload_camera_eo_scene_mode(IntEnumBase):
    PAYLOAD_CAMERA_EO_SCENE_DISABLED        =                           0
    PAYLOAD_CAMERA_EO_SCENE_FACE_PRIORITY   =                           1
    PAYLOAD_CAMERA_EO_SCENE_ACTION          =                           2
    PAYLOAD_CAMERA_EO_SCENE_PORTRAIT        =                           3
    PAYLOAD_CAMERA_EO_SCENE_LANDSCAPE       =                           4
    PAYLOAD_CAMERA_EO_SCENE_NIGHT           =                           5
    PAYLOAD_CAMERA_EO_SCENE_NIGHT_PORTRAIT  =                           6
    PAYLOAD_CAMERA_EO_SCENE_THEATRE         =                           7
    PAYLOAD_CAMERA_EO_SCENE_BEACH           =                           8
    PAYLOAD_CAMERA_EO_SCENE_SNOW            =                           9
    PAYLOAD_CAMERA_EO_SCENE_SUNSET          =                           10
    PAYLOAD_CAMERA_EO_SCENE_STEADY_PHOTO    =                           11
    PAYLOAD_CAMERA_EO_SCENE_FIREWORKS       =                           12
    PAYLOAD_CAMERA_EO_SCENE_SPORTS          =                           13
    PAYLOAD_CAMERA_EO_SCENE_PARTY           =                           14
    PAYLOAD_CAMERA_EO_SCENE_CANDLELIGHT     =                           15
    PAYLOAD_CAMERA_EO_SCENE_HDR             =                           16

# EO Auto Exposure (alias for compatibility)
PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE          =                           "C_G_AE_MODE"

# EO Shutter Speed (alias for compatibility)
PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED          =                           "C_G_SHUTTER"

# EO Aperture/Iris (alias for compatibility)
PAYLOAD_CAMERA_VIDEO_APERTURE_VALUE         =                           "C_G_IRIS"

# EO Gain (alias for compatibility)
PAYLOAD_CAMERA_EO_GAIN_LS                   =                           "C_G_GAIN"

# EO AE Compensation (-12 to 12, step 1)
PAYLOAD_CAMERA_EO_AE_COMPENSATION           =                           "C_G_AE_COMP"

# EO White Balance
PAYLOAD_CAMERA_EO_WHITE_BALANCE             =                           "C_G_WB"
PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE          =                           PAYLOAD_CAMERA_EO_WHITE_BALANCE
class payload_camera_eo_white_balance(IntEnumBase):
    PAYLOAD_CAMERA_EO_WB_OFF                =                           0
    PAYLOAD_CAMERA_EO_WB_MANUAL_CC_TEMP     =                           1
    PAYLOAD_CAMERA_EO_WB_MANUAL_RGB_GAINS   =                           2
    PAYLOAD_CAMERA_EO_WB_AUTO               =                           3
    PAYLOAD_CAMERA_EO_WB_SHADE              =                           4
    PAYLOAD_CAMERA_EO_WB_INCANDESCENT       =                           5
    PAYLOAD_CAMERA_EO_WB_FLUORESCENT        =                           6
    PAYLOAD_CAMERA_EO_WB_WARM_FLUORESCENT   =                           7
    PAYLOAD_CAMERA_EO_WB_DAYLIGHT           =                           8
    PAYLOAD_CAMERA_EO_WB_CLOUDY_DAYLIGHT    =                           9
    PAYLOAD_CAMERA_EO_WB_TWILIGHT           =                           10

# EO ISO Settings
PAYLOAD_CAMERA_EO_ISO                       =                           "C_G_ISO"
class payload_camera_eo_iso(IntEnumBase):
    PAYLOAD_CAMERA_EO_ISO_AUTO              =                           0
    PAYLOAD_CAMERA_EO_ISO_DEBLUR            =                           1
    PAYLOAD_CAMERA_EO_ISO_100               =                           2
    PAYLOAD_CAMERA_EO_ISO_200               =                           3
    PAYLOAD_CAMERA_EO_ISO_400               =                           4
    PAYLOAD_CAMERA_EO_ISO_800               =                           5
    PAYLOAD_CAMERA_EO_ISO_1600              =                           6
    PAYLOAD_CAMERA_EO_ISO_3200              =                           7

# EO Sharpness (0-6, step 1)
PAYLOAD_CAMERA_EO_SHARPNESS                 =                           "C_G_SHARPNESS"

# EO Camera Profile - selects a preset; Custom keeps the manual settings below
PAYLOAD_CAMERA_EO_PROFILE                   =                           "C_G_PROFILE"
class payload_camera_eo_profile(IntEnumBase):
    PAYLOAD_CAMERA_EO_PROFILE_CUSTOM        =                           0
    PAYLOAD_CAMERA_EO_PROFILE_DAYLIGHT      =                           1
    PAYLOAD_CAMERA_EO_PROFILE_NIGHT_MODE    =                           2

# EO Control Mode
PAYLOAD_CAMERA_EO_CONTROL_MODE              =                           "C_G_CTRL_M"
class payload_camera_eo_control_mode(IntEnumBase):
    PAYLOAD_CAMERA_EO_CONTROL_MODE_OFF            =                     0
    PAYLOAD_CAMERA_EO_CONTROL_MODE_AUTO           =                     1
    PAYLOAD_CAMERA_EO_CONTROL_MODE_USE_SCENE      =                     2
    PAYLOAD_CAMERA_EO_CONTROL_MODE_OFF_KEEP_STATE =                     3

# EO Exposure Mode - manual exposure uses C_G_EXPO_TIME and C_G_ISO_M
PAYLOAD_CAMERA_EO_EXPOSURE_MODE             =                           "C_G_EXPO_M"
class payload_camera_eo_exposure_mode(IntEnumBase):
    PAYLOAD_CAMERA_EO_EXPOSURE_MODE_MANUAL  =                           0
    PAYLOAD_CAMERA_EO_EXPOSURE_MODE_AUTO    =                           1

# EO Exposure Lock
PAYLOAD_CAMERA_EO_EXPOSURE_LOCK             =                           "C_G_EXPO_L"
class payload_camera_eo_exposure_lock(IntEnumBase):
    PAYLOAD_CAMERA_EO_EXPOSURE_LOCK_OFF     =                           0
    PAYLOAD_CAMERA_EO_EXPOSURE_LOCK_ON      =                           1

# EO Exposure Metering
PAYLOAD_CAMERA_EO_EXPOSURE_METERING         =                           "C_G_EXPO_ME"
class payload_camera_eo_exposure_metering(IntEnumBase):
    PAYLOAD_CAMERA_EO_EXPOSURE_METERING_AVERAGE =                       0
    PAYLOAD_CAMERA_EO_EXPOSURE_METERING_CENTER  =                       1
    PAYLOAD_CAMERA_EO_EXPOSURE_METERING_SPOT    =                       2

# EO Exposure Time in microseconds (200-100000, step 100)
PAYLOAD_CAMERA_EO_EXPOSURE_TIME             =                           "C_G_EXPO_TIME"

# EO Manual ISO (100-3200, step 10)
PAYLOAD_CAMERA_EO_ISO_MANUAL                =                           "C_G_ISO_M"

# EO Super HDR
PAYLOAD_CAMERA_EO_SUPER_HDR                 =                           "C_G_SHDR"
class payload_camera_eo_super_hdr(IntEnumBase):
    PAYLOAD_CAMERA_EO_SUPER_HDR_OFF         =                           0
    PAYLOAD_CAMERA_EO_SUPER_HDR_ON          =                           1

# EO ADRC (Active Disturbance Rejection Control)
PAYLOAD_CAMERA_EO_ADRC                      =                           "C_G_ADRC"
class payload_camera_eo_adrc(IntEnumBase):
    PAYLOAD_CAMERA_EO_ADRC_OFF              =                           0
    PAYLOAD_CAMERA_EO_ADRC_ON               =                           1

# EO Noise Reduction
PAYLOAD_CAMERA_EO_NOISE_REDUCTION           =                           "C_G_NR"
class payload_camera_eo_noise_reduction(IntEnumBase):
    PAYLOAD_CAMERA_EO_NOISE_REDUCTION_OFF   =                           0
    PAYLOAD_CAMERA_EO_NOISE_REDUCTION_FAST  =                           1
    PAYLOAD_CAMERA_EO_NOISE_REDUCTION_HQ    =                           2

# EO Contrast (1-10, step 1)
PAYLOAD_CAMERA_EO_CONTRAST                  =                           "C_G_CONTRA"

# EO Saturation (0-10, step 1)
PAYLOAD_CAMERA_EO_SATURATION                =                           "C_G_SATU"

# EO Night Mode
PAYLOAD_CAMERA_EO_NIGHT_MODE                =                           "C_G_N_MODE_OPT"
class payload_camera_eo_night_mode(IntEnumBase):
    PAYLOAD_CAMERA_EO_NIGHT_MODE_AUTO       =                           0
    PAYLOAD_CAMERA_EO_NIGHT_MODE_OFF        =                           1
    PAYLOAD_CAMERA_EO_NIGHT_MODE_ON         =                           2

# EO Night Mode frame rate
PAYLOAD_CAMERA_EO_NIGHT_MODE_FPS            =                           "C_G_N_MODE_FPS"
class payload_camera_eo_night_mode_fps(IntEnumBase):
    PAYLOAD_CAMERA_EO_NIGHT_MODE_FPS_30     =                           0
    PAYLOAD_CAMERA_EO_NIGHT_MODE_FPS_20     =                           1
    PAYLOAD_CAMERA_EO_NIGHT_MODE_FPS_10     =                           2

# ============================================================================
# ADVANCED IMAGE PROCESSING
# ============================================================================

# Electronic Image Stabilizer
PAYLOAD_CAMERA_ADV_EIS                      =                           "ADV_EIS"
class payload_camera_adv_eis(IntEnumBase):
    PAYLOAD_CAMERA_ADV_EIS_OFF              =                           0
    PAYLOAD_CAMERA_ADV_EIS_ON               =                           1

# IR Image Enhancement
PAYLOAD_CAMERA_ADV_IR_IMAGE_ENHANCE         =                           "ADV_IMG_ENHANCE"
class payload_camera_adv_ir_image_enhance(IntEnumBase):
    PAYLOAD_CAMERA_ADV_IR_IMAGE_ENHANCE_OFF =                           0
    PAYLOAD_CAMERA_ADV_IR_IMAGE_ENHANCE_ON  =                           1

# ============================================================================
# GIMBAL SETTINGS
# ============================================================================

# Tracking modes
PAYLOAD_CAMERA_TRACKING_MODE                =                           "TRACK_MODE"
class payload_camera_tracking_mode(IntEnumBase):
    PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING    =                           0
    PAYLOAD_CAMERA_TRACKING_OBJ_DETECTION   =                           1

# LRF Mode (Laser Range Finder) - MB1 may not have LRF, but add for compatibility
PAYLOAD_LRF_MODE                            =                           "LRF_MODE"

# Gimbal Forward Flag
PAYLOAD_CAMERA_GIMBAL_FW_FLAG               =                           "GB_FW_FLAG"
class payload_camera_gimbal_fw_flag(IntEnumBase):
    PAYLOAD_CAMERA_GIMBAL_FW_FLAG_OVERWRITE =                           0
    PAYLOAD_CAMERA_GIMBAL_FW_FLAG_FORWARD   =                           1

# ============================================================================
# ZOOM AND FOCUS CONTROLS
# ============================================================================

class camera_zoom_value(IntEnumBase):
    ZOOM_OUT  =                                                         -1
    ZOOM_STOP =                                                         0
    ZOOM_IN   =                                                         1

class camera_focus_value(IntEnumBase):
    FOCUS_OUT  =                                                        -1
    FOCUS_STOP =                                                        0
    FOCUS_IN   =                                                        1
    FOCUS_AUTO =                                                        2

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

