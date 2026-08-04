#ifndef MB1_SDK_H
#define MB1_SDK_H

// ============================================================================
// GENERAL PAYLOAD SETTINGS
// ============================================================================

// Setting Target - Select which device to configure
#define PAYLOAD_CAMERA_SETTING_TARGET                "SETTING_TARGET"
#define PAYLOAD_CAMERA_SETTING_TARGET_EO                 0
#define PAYLOAD_CAMERA_SETTING_TARGET_IR                 1
#define PAYLOAD_CAMERA_SETTING_TARGET_GIMBAL             2

// EO Zoom and Focus Speed
#define PAYLOAD_CAMERA_EO_ZOOM_SPEED              "C_V_ZOOM_SPEED"
#define PAYLOAD_CAMERA_EO_FOCUS_SPEED             "C_V_FOCUS_SPEED"

// Zoom super resolution value can be set from 1x to 12x
#define PAYLOAD_CAMERA_VIDEO_ZOOM_FACTOR              "C_V_ZOOM"
enum _zoom_eo_factor{
    ZOOM_EO_1X = 0,
    ZOOM_EO_2X,
    ZOOM_EO_3X,
    ZOOM_EO_4X,
    ZOOM_EO_5X,
    ZOOM_EO_6X,
    ZOOM_EO_7X,
    ZOOM_EO_8X,
    ZOOM_EO_9X,
    ZOOM_EO_10X,
    ZOOM_EO_11X,
    ZOOM_EO_12X
};

#define PAYLOAD_CAMERA_VIDEO_OSD_MODE    "OSD_MODE"
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_DISABLE       0
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_DEBUG         1
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS        2

#define PAYLOAD_CAMERA_RC_MODE 			"RC_MODE"
#define PAYLOAD_CAMERA_RC_MODE_GREMSY 		    0
#define PAYLOAD_CAMERA_RC_MODE_STANDARD 		1

#define PAYLOAD_CAMERA_GIMBAL_MODE               "GB_MODE"
#define PAYLOAD_CAMERA_GIMBAL_MODE_OFF               0
#define PAYLOAD_CAMERA_GIMBAL_MODE_LOCK              1
#define PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW            2
#define PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING           3
#define PAYLOAD_CAMERA_GIMBAL_MODE_RESET             4

#define PAYLOAD_CAMERA_VIEW_SRC             "C_SOURCE"
#define PAYLOAD_CAMERA_VIEW_EOIR                0
#define PAYLOAD_CAMERA_VIEW_EO                  1
#define PAYLOAD_CAMERA_VIEW_IR                  2
#define PAYLOAD_CAMERA_VIEW_IREO                3

#define PAYLOAD_CAMERA_RECORD_SRC             "C_V_REC"
#define PAYLOAD_CAMERA_RECORD_BOTH              0
#define PAYLOAD_CAMERA_RECORD_EO                1
#define PAYLOAD_CAMERA_RECORD_IR                2
#define PAYLOAD_CAMERA_RECORD_OSD               5

#define PAYLOAD_CAMERA_STORAGE                "STORAGE"
#define PAYLOAD_CAMERA_STORAGE_INTERNAL         0
#define PAYLOAD_CAMERA_STORAGE_SDCARD           1

// Alternative storage type definition (used in some payloads)
#define PAYLOAD_CAMERA_STORAGE_TYPE           "STORAGE_TYPE"
#define PAYLOAD_CAMERA_STORAGE_TYPE_INTERNAL      0
#define PAYLOAD_CAMERA_STORAGE_TYPE_SDCARD        1

#define PAYLOAD_CAMERA_IR_PALETTE             "C_T_PALETTE" 
#define PAYLOAD_CAMERA_IR_PALETTE_1           0           //      F1: WhiteHot         |       G1: WhiteHot
#define PAYLOAD_CAMERA_IR_PALETTE_2           1           //      F1: BlackHot         |       G1: Fulgurite
#define PAYLOAD_CAMERA_IR_PALETTE_3           2           //      F1: Rainbow          |       G1: IronRed
#define PAYLOAD_CAMERA_IR_PALETTE_4           3           //      F1: RainbowHC        |       G1: HotIron
#define PAYLOAD_CAMERA_IR_PALETTE_5           4           //      F1: Ironbow          |       G1: Medical
#define PAYLOAD_CAMERA_IR_PALETTE_6           5           //      F1: Lava             |       G1: Arctic
#define PAYLOAD_CAMERA_IR_PALETTE_7           6           //      F1: Arctic           |       G1: Rainbow1
#define PAYLOAD_CAMERA_IR_PALETTE_8           7           //      F1: Globow           |       G1: Rainbow2
#define PAYLOAD_CAMERA_IR_PALETTE_9           8           //      F1: Gradedfire       |       G1: Tint
#define PAYLOAD_CAMERA_IR_PALETTE_10          9           //      F1: Hottest          |       G1: BlackHot

#define PAYLOAD_CAMERA_OBJECT_DETECTION         	"DETECTION_EN"
#define PAYLOAD_CAMERA_OBJECT_DETECTION_DISABLE 	    0
#define PAYLOAD_CAMERA_OBJECT_DETECTION_ENABLE 		    1

#define PAYLOAD_CAMERA_IR_ISOTHERMS         	"ISOTHERMS_EN"
#define PAYLOAD_CAMERA_IR_ISOTHERMS_DISABLE 	    0
#define PAYLOAD_CAMERA_IR_ISOTHERMS_ENABLE 		    1

#define PAYLOAD_CAMERA_IR_ISOTHERMS_GAIN         	"ISOTHERMS_GAIN"
#define PAYLOAD_CAMERA_IR_ISOTHERMS_HIGH_GAIN 		    0
#define PAYLOAD_CAMERA_IR_ISOTHERMS_LOW_GAIN 	        1

// ============================================================================
// IR CAMERA ADVANCED SETTINGS
// ============================================================================

// IR Gain Mode (Temperature Range)
#define PAYLOAD_CAMERA_IR_GAIN                      "C_T_G"
#define PAYLOAD_CAMERA_IR_GAIN_LOW                      0    // -50°C~150°C or -20°C~150°C
#define PAYLOAD_CAMERA_IR_GAIN_HIGH                     1    // -50°C~550°C or -20°C~550°C

// IR Contrast Mode
#define PAYLOAD_CAMERA_IR_CONTRAST_MODE             "C_T_CONST_M"
#define PAYLOAD_CAMERA_IR_CONTRAST_MODE_DEFAULT         0
#define PAYLOAD_CAMERA_IR_CONTRAST_MODE_CUSTOM          1

// IR AGC (Automatic Gain Control) Mode
#define PAYLOAD_CAMERA_IR_AGC_MODE                  "C_T_AGC_M"
#define PAYLOAD_CAMERA_IR_AGC_MODE_NORMAL               0
#define PAYLOAD_CAMERA_IR_AGC_MODE_HOLD                 1
#define PAYLOAD_CAMERA_IR_AGC_MODE_THRESHOLD            2
#define PAYLOAD_CAMERA_IR_AGC_MODE_BRIGHT               3
#define PAYLOAD_CAMERA_IR_AGC_MODE_LINEAR               4
#define PAYLOAD_CAMERA_IR_AGC_MODE_MANUAL               5

// IR AGC Linear Percent (0-100, step 10)
#define PAYLOAD_CAMERA_IR_AGC_LINEAR_PERCENT        "C_T_AGC_LNP"

// IR SpotMeter Mode
#define PAYLOAD_CAMERA_IR_SPOTMETER_MODE            "C_T_SPOT_M"
#define PAYLOAD_CAMERA_IR_SPOTMETER_MODE_DISABLE        0
#define PAYLOAD_CAMERA_IR_SPOTMETER_MODE_ENABLE         1

// IR SpotMeter Units
#define PAYLOAD_CAMERA_IR_SPOTMETER_UNITS           "C_T_SPOT_U"
#define PAYLOAD_CAMERA_IR_SPOTMETER_UNITS_CELSIUS       0
#define PAYLOAD_CAMERA_IR_SPOTMETER_UNITS_FAHRENHEIT    1
#define PAYLOAD_CAMERA_IR_SPOTMETER_UNITS_KELVIN        2

// IR SpotMeter Size (16-128, step 4)
#define PAYLOAD_CAMERA_IR_SPOTMETER_SIZE            "C_T_SPOT_S"

// IR Isotherm Mode (Advanced)
#define PAYLOAD_CAMERA_IR_ISOTHERM_MODE             "C_T_ISO_M"
#define PAYLOAD_CAMERA_IR_ISOTHERM_MODE_DISABLE         0
#define PAYLOAD_CAMERA_IR_ISOTHERM_MODE_ENABLE          1

// IR Isotherm Units
#define PAYLOAD_CAMERA_IR_ISOTHERM_UNITS            "C_T_ISO_U"
#define PAYLOAD_CAMERA_IR_ISOTHERM_UNITS_KELVIN         0
#define PAYLOAD_CAMERA_IR_ISOTHERM_UNITS_CELSIUS        1
#define PAYLOAD_CAMERA_IR_ISOTHERM_UNITS_FAHRENHEIT     2
#define PAYLOAD_CAMERA_IR_ISOTHERM_UNITS_PERCENT        3
#define PAYLOAD_CAMERA_IR_ISOTHERM_UNITS_COUNTS         4

// IR Isotherm Threshold (0-150, step 5) - Used in Infisignt/Lynx
#define PAYLOAD_CAMERA_IR_ISOTHERM_THRESHOLD        "C_T_ISO_THR"

// IR Isotherm Region 0 Settings
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG0_TEMP         "C_T_RG0_TMP"      // -1000~1000, step 10
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MODE   "C_T_RG0_CLR_M"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MODE_DISABLE       0
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MODE_STANDARD      1
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MODE_LINEAR_RGB    2
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MODE_LINEAR_HSV    3
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MODE_NON_LINEAR    4
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MODE_SINGLE        5
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MIN    "C_T_RG0_CLR_MN"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG0_COLOR_MAX    "C_T_RG0_CLR_MX"

// IR Isotherm Region 1 Settings
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG1_TEMP         "C_T_RG1_TMP"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG1_COLOR_MODE   "C_T_RG1_CLR_M"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG1_COLOR_MIN    "C_T_RG1_CLR_MN"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG1_COLOR_MAX    "C_T_RG1_CLR_MX"

// IR Isotherm Region 2 Settings
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG2_TEMP         "C_T_RG2_TMP"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG2_COLOR_MODE   "C_T_RG2_CLR_M"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG2_COLOR_MIN    "C_T_RG2_CLR_MN"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG2_COLOR_MAX    "C_T_RG2_CLR_MX"

// IR Isotherm Region 3 Settings
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG3_TEMP         "C_T_RG3_TMP"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG3_COLOR_MODE   "C_T_RG3_CLR_M"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG3_COLOR_MIN    "C_T_RG3_CLR_MN"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG3_COLOR_MAX    "C_T_RG3_CLR_MX"

// IR Isotherm Region 4 Settings
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG4_TEMP         "C_T_RG4_TMP"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG4_COLOR_MODE   "C_T_RG4_CLR_M"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG4_COLOR_MIN    "C_T_RG4_CLR_MN"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG4_COLOR_MAX    "C_T_RG4_CLR_MX"

// IR Isotherm Region 5 Settings (no temperature setting for RG5)
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG5_COLOR_MODE   "C_T_RG5_CLR_M"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG5_COLOR_MIN    "C_T_RG5_CLR_MN"
#define PAYLOAD_CAMERA_IR_ISOTHERM_RG5_COLOR_MAX    "C_T_RG5_CLR_MX"

// Isotherm Color Values (for all regions)
enum _isotherm_color {
    ISOTHERM_COLOR_ICE_BLUE = 0,
    ISOTHERM_COLOR_SKY_BLUE,
    ISOTHERM_COLOR_LIGHT_GREEN,
    ISOTHERM_COLOR_YELLOW_GREEN,
    ISOTHERM_COLOR_YELLOW,
    ISOTHERM_COLOR_ORANGE,
    ISOTHERM_COLOR_RED
};

// ============================================================================
// EO CAMERA SETTINGS
// ============================================================================

// EO Image Flip
#define PAYLOAD_CAMERA_EO_FLIP                      "C_V_FLIP"
#define PAYLOAD_CAMERA_EO_FLIP_OFF                      3
#define PAYLOAD_CAMERA_EO_FLIP_ON                       2
#define PAYLOAD_CAMERA_VIDEO_FLIP                   PAYLOAD_CAMERA_EO_FLIP

// EO Camera Scene Mode (Optimization)
#define PAYLOAD_CAMERA_EO_SCENE_MODE                "C_G_SCENE"
#define PAYLOAD_CAMERA_EO_SCENE_DISABLED                0
#define PAYLOAD_CAMERA_EO_SCENE_FACE_PRIORITY           1
#define PAYLOAD_CAMERA_EO_SCENE_ACTION                  2
#define PAYLOAD_CAMERA_EO_SCENE_PORTRAIT                3
#define PAYLOAD_CAMERA_EO_SCENE_LANDSCAPE               4
#define PAYLOAD_CAMERA_EO_SCENE_NIGHT                   5
#define PAYLOAD_CAMERA_EO_SCENE_NIGHT_PORTRAIT          6
#define PAYLOAD_CAMERA_EO_SCENE_THEATRE                 7
#define PAYLOAD_CAMERA_EO_SCENE_BEACH                   8
#define PAYLOAD_CAMERA_EO_SCENE_SNOW                    9
#define PAYLOAD_CAMERA_EO_SCENE_SUNSET                  10
#define PAYLOAD_CAMERA_EO_SCENE_STEADY_PHOTO            11
#define PAYLOAD_CAMERA_EO_SCENE_FIREWORKS               12
#define PAYLOAD_CAMERA_EO_SCENE_SPORTS                  13
#define PAYLOAD_CAMERA_EO_SCENE_PARTY                   14
#define PAYLOAD_CAMERA_EO_SCENE_CANDLELIGHT             15
#define PAYLOAD_CAMERA_EO_SCENE_HDR                     16

// EO Auto Exposure (alias for compatibility)
#define PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE          "C_G_AE_MODE"

// EO Shutter Speed (alias for compatibility)
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED          "C_G_SHUTTER"

// EO Aperture/Iris (alias for compatibility)
#define PAYLOAD_CAMERA_VIDEO_APERTURE_VALUE         "C_G_IRIS"

// EO Gain (alias for compatibility)
#define PAYLOAD_CAMERA_EO_GAIN_LS                   "C_G_GAIN"

// EO AE Compensation (-12 to 12, step 1)
#define PAYLOAD_CAMERA_EO_AE_COMPENSATION           "C_G_AE_COMP"

// EO White Balance
#define PAYLOAD_CAMERA_EO_WHITE_BALANCE             "C_G_WB"
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE          PAYLOAD_CAMERA_EO_WHITE_BALANCE
#define PAYLOAD_CAMERA_EO_WB_OFF                        0
#define PAYLOAD_CAMERA_EO_WB_MANUAL_CC_TEMP             1
#define PAYLOAD_CAMERA_EO_WB_MANUAL_RGB_GAINS           2
#define PAYLOAD_CAMERA_EO_WB_AUTO                       3
#define PAYLOAD_CAMERA_EO_WB_SHADE                      4
#define PAYLOAD_CAMERA_EO_WB_INCANDESCENT               5
#define PAYLOAD_CAMERA_EO_WB_FLUORESCENT                6
#define PAYLOAD_CAMERA_EO_WB_WARM_FLUORESCENT           7
#define PAYLOAD_CAMERA_EO_WB_DAYLIGHT                   8
#define PAYLOAD_CAMERA_EO_WB_CLOUDY_DAYLIGHT            9
#define PAYLOAD_CAMERA_EO_WB_TWILIGHT                   10

// EO ISO Settings
#define PAYLOAD_CAMERA_EO_ISO                       "C_G_ISO"
#define PAYLOAD_CAMERA_EO_ISO_AUTO                      0
#define PAYLOAD_CAMERA_EO_ISO_DEBLUR                    1
#define PAYLOAD_CAMERA_EO_ISO_100                       2
#define PAYLOAD_CAMERA_EO_ISO_200                       3
#define PAYLOAD_CAMERA_EO_ISO_400                       4
#define PAYLOAD_CAMERA_EO_ISO_800                       5
#define PAYLOAD_CAMERA_EO_ISO_1600                      6
#define PAYLOAD_CAMERA_EO_ISO_3200                      7

// EO Sharpness (0-6, step 1)
#define PAYLOAD_CAMERA_EO_SHARPNESS                 "C_G_SHARPNESS"

// EO Camera Profile - selects a preset; Custom keeps the manual settings below
#define PAYLOAD_CAMERA_EO_PROFILE                   "C_G_PROFILE"
#define PAYLOAD_CAMERA_EO_PROFILE_CUSTOM                0
#define PAYLOAD_CAMERA_EO_PROFILE_DAYLIGHT              1
#define PAYLOAD_CAMERA_EO_PROFILE_NIGHT_MODE            2

// EO Control Mode
#define PAYLOAD_CAMERA_EO_CONTROL_MODE              "C_G_CTRL_M"
#define PAYLOAD_CAMERA_EO_CONTROL_MODE_OFF              0
#define PAYLOAD_CAMERA_EO_CONTROL_MODE_AUTO             1
#define PAYLOAD_CAMERA_EO_CONTROL_MODE_USE_SCENE        2
#define PAYLOAD_CAMERA_EO_CONTROL_MODE_OFF_KEEP_STATE   3

// EO Exposure Mode - manual exposure uses C_G_EXPO_TIME and C_G_ISO_M
#define PAYLOAD_CAMERA_EO_EXPOSURE_MODE             "C_G_EXPO_M"
#define PAYLOAD_CAMERA_EO_EXPOSURE_MODE_MANUAL          0
#define PAYLOAD_CAMERA_EO_EXPOSURE_MODE_AUTO            1

// EO Exposure Lock
#define PAYLOAD_CAMERA_EO_EXPOSURE_LOCK             "C_G_EXPO_L"
#define PAYLOAD_CAMERA_EO_EXPOSURE_LOCK_OFF             0
#define PAYLOAD_CAMERA_EO_EXPOSURE_LOCK_ON              1

// EO Exposure Metering
#define PAYLOAD_CAMERA_EO_EXPOSURE_METERING         "C_G_EXPO_ME"
#define PAYLOAD_CAMERA_EO_EXPOSURE_METERING_AVERAGE     0
#define PAYLOAD_CAMERA_EO_EXPOSURE_METERING_CENTER      1
#define PAYLOAD_CAMERA_EO_EXPOSURE_METERING_SPOT        2

// EO Exposure Time in microseconds (200-100000, step 100)
#define PAYLOAD_CAMERA_EO_EXPOSURE_TIME             "C_G_EXPO_TIME"

// EO Manual ISO (100-3200, step 10)
#define PAYLOAD_CAMERA_EO_ISO_MANUAL                "C_G_ISO_M"

// EO Super HDR
#define PAYLOAD_CAMERA_EO_SUPER_HDR                 "C_G_SHDR"
#define PAYLOAD_CAMERA_EO_SUPER_HDR_OFF                 0
#define PAYLOAD_CAMERA_EO_SUPER_HDR_ON                  1

// EO ADRC (Active Disturbance Rejection Control)
#define PAYLOAD_CAMERA_EO_ADRC                      "C_G_ADRC"
#define PAYLOAD_CAMERA_EO_ADRC_OFF                      0
#define PAYLOAD_CAMERA_EO_ADRC_ON                       1

// EO Noise Reduction
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION           "C_G_NR"
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_OFF           0
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_FAST          1
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_HQ            2

// EO Contrast (1-10, step 1)
#define PAYLOAD_CAMERA_EO_CONTRAST                  "C_G_CONTRA"

// EO Saturation (0-10, step 1)
#define PAYLOAD_CAMERA_EO_SATURATION                "C_G_SATU"

// EO Night Mode
#define PAYLOAD_CAMERA_EO_NIGHT_MODE                "C_G_N_MODE_OPT"
#define PAYLOAD_CAMERA_EO_NIGHT_MODE_AUTO               0
#define PAYLOAD_CAMERA_EO_NIGHT_MODE_OFF                1
#define PAYLOAD_CAMERA_EO_NIGHT_MODE_ON                 2

// EO Night Mode frame rate
#define PAYLOAD_CAMERA_EO_NIGHT_MODE_FPS            "C_G_N_MODE_FPS"
#define PAYLOAD_CAMERA_EO_NIGHT_MODE_FPS_30             0
#define PAYLOAD_CAMERA_EO_NIGHT_MODE_FPS_20             1
#define PAYLOAD_CAMERA_EO_NIGHT_MODE_FPS_10             2

// ============================================================================
// ADVANCED IMAGE PROCESSING
// ============================================================================

// Electronic Image Stabilizer
#define PAYLOAD_CAMERA_ADV_EIS                      "ADV_EIS"
#define PAYLOAD_CAMERA_ADV_EIS_OFF                      0
#define PAYLOAD_CAMERA_ADV_EIS_ON                       1

// IR Image Enhancement
#define PAYLOAD_CAMERA_ADV_IR_IMAGE_ENHANCE         "ADV_IMG_ENHANCE"
#define PAYLOAD_CAMERA_ADV_IR_IMAGE_ENHANCE_OFF         0
#define PAYLOAD_CAMERA_ADV_IR_IMAGE_ENHANCE_ON          1

// ============================================================================
// GIMBAL SETTINGS
// ============================================================================

// Tracking modes
#define PAYLOAD_CAMERA_TRACKING_MODE 	"TRACK_MODE"
#define PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING 	    0
#define PAYLOAD_CAMERA_TRACKING_OBJ_DETECTION 		1

// LRF Mode (Laser Range Finder) - MB1 may not have LRF, but add for compatibility
#define PAYLOAD_LRF_MODE                    "LRF_MODE"

// Gimbal Forward Flag
#define PAYLOAD_CAMERA_GIMBAL_FW_FLAG               "GB_FW_FLAG"
#define PAYLOAD_CAMERA_GIMBAL_FW_FLAG_OVERWRITE         0
#define PAYLOAD_CAMERA_GIMBAL_FW_FLAG_FORWARD           1

// Zoom value can be set from 1x to 8x for camera thermal
#define PAYLOAD_CAMERA_IR_ZOOM_FACTOR              "C_T_ZOOM"
enum _zoom_ir_factor{
    ZOOM_IR_1X = 0,
    ZOOM_IR_2X,
    ZOOM_IR_3X,
    ZOOM_IR_4X,
    ZOOM_IR_5X,
    ZOOM_IR_6X,
    ZOOM_IR_7X,
    ZOOM_IR_8X
};

#endif