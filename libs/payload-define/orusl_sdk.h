#ifndef ORUSL_SDK_H
#define ORUSL_SDK_H

// Tracking modes
#define PAYLOAD_CAMERA_TRACKING_MODE 	"TRACK_MODE"
#define PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING 	    0
#define PAYLOAD_CAMERA_TRACKING_OBJ_DETECTION 		1

// RC modes
#define PAYLOAD_CAMERA_RC_MODE 			"RC_MODE"
#define PAYLOAD_CAMERA_RC_MODE_GREMSY 		0
#define PAYLOAD_CAMERA_RC_MODE_STANDARD 		1

// Camera sources
#define PAYLOAD_CAMERA_VIEW_SRC             "C_SOURCE"
#define PAYLOAD_CAMERA_VIEW_EOIR                0
#define PAYLOAD_CAMERA_VIEW_EO                  1
#define PAYLOAD_CAMERA_VIEW_IR                  2
#define PAYLOAD_CAMERA_VIEW_IREO                3
#define PAYLOAD_CAMERA_VIEW_SYNC                4
#define PAYLOAD_CAMERA_VIEW_SIDE_BY_SIDE        6

// Camera record sources
#define PAYLOAD_CAMERA_RECORD_SRC             "C_V_REC"
#define PAYLOAD_CAMERA_RECORD_BOTH              0
#define PAYLOAD_CAMERA_RECORD_EO                1
#define PAYLOAD_CAMERA_RECORD_IR                2
#define PAYLOAD_CAMERA_RECORD_OSD               5

// OSD modes
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE    "OSD_MODE"
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_DISABLE       0
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_DEBUG         1
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS        2

// Image flip
#define PAYLOAD_CAMERA_VIDEO_FLIP        "C_V_FLIP"
#define PAYLOAD_CAMERA_VIDEO_FLIP_OFF       3
#define PAYLOAD_CAMERA_VIDEO_FLIP_ON        2

// ---------------------------------------------------------

// IR palettes
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

// IR Zoom value can be set from 1x to 8x for camera thermal
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

// ---------------------------------------------------------

// EO zoom modes
#define PAYLOAD_CAMERA_VIDEO_ZOOM_MODE              "C_V_ZM_MODE"
#define PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_COMBINE              0
#define PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION     2

// Combine Zoom levels can be set from 1x to 240x
#define PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_FACTOR              "C_V_ZM_CB_LV"
enum _zoom_combine_factor{
    ZOOM_COMBINE_1X = 0,
    ZOOM_COMBINE_2X,
    ZOOM_COMBINE_4X,
    ZOOM_COMBINE_6X,
    ZOOM_COMBINE_8X,
    ZOOM_COMBINE_10X,
    ZOOM_COMBINE_12X,
    ZOOM_COMBINE_14X,
    ZOOM_COMBINE_16X,
    ZOOM_COMBINE_18X,
    ZOOM_COMBINE_20X,
    ZOOM_COMBINE_22X,
    ZOOM_COMBINE_25X,
    ZOOM_COMBINE_50X,
    ZOOM_COMBINE_75X,
    ZOOM_COMBINE_100X,
    ZOOM_COMBINE_125X,
    ZOOM_COMBINE_150X,
    ZOOM_COMBINE_175X,
    ZOOM_COMBINE_200X,
    ZOOM_COMBINE_225X,
    ZOOM_COMBINE_250X,
    ZOOM_COMBINE_275X,
    ZOOM_COMBINE_300X
};

// Super Resolution Zoom levels can be set from 1x to 30x
#define PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR              "C_V_ZM_SR_LV"
enum _zoom_super_resolution_factor{
    ZOOM_SUPER_RESOLUTION_1X = 0,
    ZOOM_SUPER_RESOLUTION_2X,
    ZOOM_SUPER_RESOLUTION_4X,
    ZOOM_SUPER_RESOLUTION_6X,
    ZOOM_SUPER_RESOLUTION_8X,
    ZOOM_SUPER_RESOLUTION_10X,
    ZOOM_SUPER_RESOLUTION_12X,
    ZOOM_SUPER_RESOLUTION_14X,
    ZOOM_SUPER_RESOLUTION_16X,
    ZOOM_SUPER_RESOLUTION_18X,
    ZOOM_SUPER_RESOLUTION_20X,   
    ZOOM_SUPER_RESOLUTION_22X,
    ZOOM_SUPER_RESOLUTION_25X
};

// EO Zoom Speed
// values can be set from 0 to 7, step 1
#define PAYLOAD_CAMERA_EO_ZOOM_SPEED    "C_V_Z_SPD"

// Image freeze
#define PAYLOAD_CAMERA_EO_FREEZE        "C_V_FREEZE"
#define PAYLOAD_CAMERA_EO_FREEZE_OFF       3
#define PAYLOAD_CAMERA_EO_FREEZE_ON        2

// Defog modes
#define PAYLOAD_CAMERA_VIDEO_DEFOG        "C_V_DEFOG"
#define PAYLOAD_CAMERA_VIDEO_DEFOG_OFF       3
#define PAYLOAD_CAMERA_VIDEO_DEFOG_ON        2

// Defog levels
#define PAYLOAD_CAMERA_VIDEO_DEFOG_LEVEL  "C_V_DEFOG_LV"
#define PAYLOAD_CAMERA_VIDEO_DEFOG_LOWEST       0
#define PAYLOAD_CAMERA_VIDEO_DEFOG_LOW          1
#define PAYLOAD_CAMERA_VIDEO_DEFOG_MID          2
#define PAYLOAD_CAMERA_VIDEO_DEFOG_HIGH         3

// EO High Sensitivity
#define PAYLOAD_CAMERA_EO_HS        "C_V_HS"
#define PAYLOAD_CAMERA_EO_HS_OFF       3
#define PAYLOAD_CAMERA_EO_HS_ON        2

// Exposure modes
#define PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE              "C_V_AE"
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_AUTO                  0
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_MANUAL                3
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_SHUTTER               10
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_IRIS                  11

// Shutter speeds
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED              "C_V_SP"
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1             0
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_2             1
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_4             2
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_8             3
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_15            4
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_30            5
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_60            6
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_90            7
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_100           8
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_125           9
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_180           10
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_250           11
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_350           12
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_500           13
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_725           14
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1000          15
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1500          16
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_2000          17
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_3000          18
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_4000          19
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_6000          20
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_10000         21

// EO shutter min limit
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT             "C_V_MinSP"

// Aperture values
#define PAYLOAD_CAMERA_VIDEO_APERTURE_VALUE              "C_V_IrP"
#define PAYLOAD_CAMERA_EO_APERTURE_F1_6                 16
#define PAYLOAD_CAMERA_EO_APERTURE_F2_0                 15
#define PAYLOAD_CAMERA_EO_APERTURE_F2_4                 14
#define PAYLOAD_CAMERA_EO_APERTURE_F2_8                 13
#define PAYLOAD_CAMERA_EO_APERTURE_F3_4                 12
#define PAYLOAD_CAMERA_EO_APERTURE_F4_0                 11
#define PAYLOAD_CAMERA_EO_APERTURE_F5_6                 10
#define PAYLOAD_CAMERA_EO_APERTURE_F6_8                 9
#define PAYLOAD_CAMERA_EO_APERTURE_F8_0                 8
#define PAYLOAD_CAMERA_EO_APERTURE_F9_6                 7
#define PAYLOAD_CAMERA_EO_APERTURE_F11                  6
#define PAYLOAD_CAMERA_EO_APERTURE_F14                  5  

// EO Gain in High Sensitivity mode, PAYLOAD_CAMERA_EO_HS = PAYLOAD_CAMERA_EO_HS_ON
#define PAYLOAD_CAMERA_EO_GAIN_HS           ""
#define PAYLOAD_CAMERA_EO_GAIN_HS_48DB      17
#define PAYLOAD_CAMERA_EO_GAIN_HS_45DB      16
#define PAYLOAD_CAMERA_EO_GAIN_HS_42DB      15
#define PAYLOAD_CAMERA_EO_GAIN_HS_39DB      14
#define PAYLOAD_CAMERA_EO_GAIN_HS_36DB      13
#define PAYLOAD_CAMERA_EO_GAIN_HS_33DB      12
#define PAYLOAD_CAMERA_EO_GAIN_HS_30DB      11
#define PAYLOAD_CAMERA_EO_GAIN_HS_27DB      10
#define PAYLOAD_CAMERA_EO_GAIN_HS_24DB      9
#define PAYLOAD_CAMERA_EO_GAIN_HS_21DB      8
#define PAYLOAD_CAMERA_EO_GAIN_HS_18DB      7
#define PAYLOAD_CAMERA_EO_GAIN_HS_15DB      6
#define PAYLOAD_CAMERA_EO_GAIN_HS_12DB      5
#define PAYLOAD_CAMERA_EO_GAIN_HS_9DB      4
#define PAYLOAD_CAMERA_EO_GAIN_HS_6DB      3
#define PAYLOAD_CAMERA_EO_GAIN_HS_3DB      2
#define PAYLOAD_CAMERA_EO_GAIN_HS_0DB      1

// EO Gain in High Sensitivity mode, PAYLOAD_CAMERA_EO_HS = PAYLOAD_CAMERA_EO_HS_OFF
#define PAYLOAD_CAMERA_EO_GAIN_LS           ""
#define PAYLOAD_CAMERA_EO_GAIN_LS_36DB      13
#define PAYLOAD_CAMERA_EO_GAIN_LS_33DB      12
#define PAYLOAD_CAMERA_EO_GAIN_LS_30DB      11
#define PAYLOAD_CAMERA_EO_GAIN_LS_27DB      10
#define PAYLOAD_CAMERA_EO_GAIN_LS_24DB      9
#define PAYLOAD_CAMERA_EO_GAIN_LS_21DB      8
#define PAYLOAD_CAMERA_EO_GAIN_LS_18DB      7
#define PAYLOAD_CAMERA_EO_GAIN_LS_15DB      6
#define PAYLOAD_CAMERA_EO_GAIN_LS_12DB      5
#define PAYLOAD_CAMERA_EO_GAIN_LS_9DB      4
#define PAYLOAD_CAMERA_EO_GAIN_LS_6DB      3
#define PAYLOAD_CAMERA_EO_GAIN_LS_3DB      2
#define PAYLOAD_CAMERA_EO_GAIN_LS_0DB      1

// EO Bright in High Sensitivity mode, PAYLOAD_CAMERA_EO_HS = PAYLOAD_CAMERA_EO_HS_ON
// values can be set from 0 to 41, step 1
#define PAYLOAD_CAMERA_VIDEO_BRIGHT_VALUE              "C_V_BrP_HS"

// EO Bright in High Sensitivity mode, PAYLOAD_CAMERA_EO_HS = PAYLOAD_CAMERA_EO_HS_OFF
// values can be set from 0 to 37, step 1
#define PAYLOAD_CAMERA_VIDEO_BRIGHT_VALUE              "C_V_BrP_LS"

// EO white-balance modes
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE              "C_V_WB"
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_AUTO             0
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_INDOOR           1
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_OUTDOOR          2
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_ONE_PUSH         3
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_ATW              4
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_MANUAL           5

// EO R gains, values can be set from 0 to 255, step 1, in case of PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE set to PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_MANUAL
#define PAYLOAD_CAMERA_EO_R_GAIN            "C_V_RGAIN"

// EO B gains, values can be set from 0 to 255, step 1, in case of PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE set to PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_MANUAL
#define PAYLOAD_CAMERA_EO_B_GAIN            "C_V_BGAIN"

// EO focus modes
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE              "C_V_FM"
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_MANUAL                       0
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_FOCUS                   1
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_FOCUS_ONEPUSH           2

// EO Manual focus value can be set from 0 to 61440, step 1
#define PAYLOAD_CAMERA_VIDEO_FOCUS_VALUE              "C_V_FV"

// EO Focus Speed
// values can be set from 0 to 7, step 1
#define PAYLOAD_CAMERA_EO_FOCUS_SPEED    "C_V_F_SPD"

// EO ICR modes
#define PAYLOAD_CAMERA_EO_ICR_MODE      "C_V_ICR"
#define PAYLOAD_CAMERA_EO_ICR_MODE_AUTO       2
#define PAYLOAD_CAMERA_EO_ICR_MODE_MANUAL     3

// EO ICR AUTO threshold, values can be set from 0 to 255, step 1
#define PAYLOAD_CAMERA_EO_ICR_MODE_AUTO_THRESHOLD      "C_V_ICR_THR"

// EO ICR MANUAL modes
#define PAYLOAD_CAMERA_EO_ICR_MANUAL      "C_V_ICR_MAN"
#define PAYLOAD_CAMERA_EO_ICR_MANUAL_ON     2
#define PAYLOAD_CAMERA_EO_ICR_MANUAL_OFF     3

// EO EIS modes
#define PAYLOAD_CAMERA_EO_EIS_MODE          "C_V_EIS"
#define PAYLOAD_CAMERA_EO_EIS_MODE_HOLD     0
#define PAYLOAD_CAMERA_EO_EIS_MODE_ON       2
#define PAYLOAD_CAMERA_EO_EIS_MODE_OFF      3

// EO EIS levels
#define PAYLOAD_CAMERA_EO_EIS_LEVEL         "C_V_EIS_LV"
#define PAYLOAD_CAMERA_EO_EIS_LEVEL_SUPER       2
#define PAYLOAD_CAMERA_EO_EIS_LEVEL_SUPER_PLUS  3

// EO Spot Light Avoidance
#define PAYLOAD_CAMERA_EO_SPOT_LIGHT_AVOIDANCE          "C_V_SPAVOID"
#define PAYLOAD_CAMERA_EO_SPOT_LIGHT_AVOIDANCE_ON           1
#define PAYLOAD_CAMERA_EO_SPOT_LIGHT_AVOIDANCE_OFF          0

// EO Flicker Reduction
#define PAYLOAD_CAMERA_EO_FLICKER_REDUCTION         "C_V_FLREDUCT"
#define PAYLOAD_CAMERA_EO_FLICKER_REDUCTION_ON          2
#define PAYLOAD_CAMERA_EO_FLICKER_REDUCTION_OFF         3

// EO Noise Reduction
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION           "C_V_NSREDUCT"
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_OFF           0
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_LV1           1
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_LV2           2
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_LV3           3
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_LV4           4
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_LV5           5
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_MANUAL           127

// EO 2D noise reduction, in case of PAYLOAD_CAMERA_EO_NOISE_REDUCTION = PAYLOAD_CAMERA_EO_NOISE_REDUCTION_MANUAL
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_2D            "C_V_NSREDUCT2D"
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_2D_OFF        0
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_2D_LV1        1
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_2D_LV2        2
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_2D_LV3        3
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_2D_LV4        4
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_2D_LV5        5

// EO 3D noise reduction, in case of PAYLOAD_CAMERA_EO_NOISE_REDUCTION = PAYLOAD_CAMERA_EO_NOISE_REDUCTION_MANUAL
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_3D            "C_V_NSREDUCT3D"
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_3D_OFF        0
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_3D_LV1        1
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_3D_LV2        2
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_3D_LV3        3
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_3D_LV4        4
#define PAYLOAD_CAMERA_EO_NOISE_REDUCTION_3D_LV5        5

// EO stable zoom
#define PAYLOAD_CAMERA_EO_STABLE_ZOOM           "C_V_STZOOM"
#define PAYLOAD_CAMERA_EO_STABLE_ZOOM_OFF       0
#define PAYLOAD_CAMERA_EO_STABLE_ZOOM_ON        1

// Payload defog fan
#define PAYLOAD_FAN_DEFOG               "C_F_DEFOG"
#define PAYLOAD_FAN_DEFOG_AUTO           0
#define PAYLOAD_FAN_DEFOG_OFF            1
#define PAYLOAD_FAN_DEFOG_ON             2

// Gimbal modes
#define PAYLOAD_CAMERA_GIMBAL_MODE              "GB_MODE"
#define PAYLOAD_CAMERA_GIMBAL_MODE_OFF              0
#define PAYLOAD_CAMERA_GIMBAL_MODE_LOCK             1
#define PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW           2
#define PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING          3
#define PAYLOAD_CAMERA_GIMBAL_MODE_RESET            4

// LRF modes
#define PAYLOAD_LRF_MODE        "LRF_MODE"
#define PAYLOAD_LRF_MODE_OFF    3
#define PAYLOAD_LRF_MODE_1HZ    0
#define PAYLOAD_LRF_MODE_4HZ    1
#define PAYLOAD_LRF_MODE_10HZ    2


#endif