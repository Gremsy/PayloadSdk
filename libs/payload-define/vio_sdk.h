#ifndef VIO_SDK_H
#define VIO_SDK_H

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
    ZOOM_COMBINE_40X,
    ZOOM_COMBINE_60X,
    ZOOM_COMBINE_80X,
    ZOOM_COMBINE_100X,
    ZOOM_COMBINE_120X,
    ZOOM_COMBINE_140X,
    ZOOM_COMBINE_160X,
    ZOOM_COMBINE_180X,
    ZOOM_COMBINE_200X,
    ZOOM_COMBINE_220X,
    ZOOM_COMBINE_240X
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
    ZOOM_SUPER_RESOLUTION_30X
};

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
#define PAYLOAD_CAMERA_EO_HS_OFF       0
#define PAYLOAD_CAMERA_EO_HS_ON        1

// Exposure modes
#define PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE              "C_V_AE"
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_AUTO                  0
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_MANUAL                3
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_SHUTTER               10
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_IRIS                  11
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_BRIGHT                13

// Shutter speeds
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED              "C_V_SP"
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1             6
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_2_3             7
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_2             8
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_3             9
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_4             10
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_6             11
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_8             12
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_10             13
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_15             14
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_20             15
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_30             16
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_50             17
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_60             18
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_90             19
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_100             20
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_125             21
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_180             22
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_250             23
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_350             24
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_500             25
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_725             26
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1000             27
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1500             28
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_2000             29
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_3000             30
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_4000             31
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_6000             32
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_10000            33

// EO shutter min limit
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT             "C_V_MinSP"
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_10        13
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_15        14
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_20        15
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_30        16
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_50        17
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_60        18
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_90        19
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_100        20
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_125        21
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_180        22
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_250        23
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_350        24
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_500        25
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_725        26
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_1000        27
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_1500        28
#define PAYLOAD_CAMERA_EO_SHUTTER_MIN_LIMIT_1_2000        29

// Aperture values
#define PAYLOAD_CAMERA_VIDEO_APERTURE_VALUE              "C_V_IrP"
#define PAYLOAD_CAMERA_EO_APERTURE_F2_0                 25
#define PAYLOAD_CAMERA_EO_APERTURE_F2_2                 24
#define PAYLOAD_CAMERA_EO_APERTURE_F2_4                 23
#define PAYLOAD_CAMERA_EO_APERTURE_F2_6                 22
#define PAYLOAD_CAMERA_EO_APERTURE_F2_8                 21
#define PAYLOAD_CAMERA_EO_APERTURE_F3_1                 20
#define PAYLOAD_CAMERA_EO_APERTURE_F3_4                 19
#define PAYLOAD_CAMERA_EO_APERTURE_F4_0                 17
#define PAYLOAD_CAMERA_EO_APERTURE_F5_2                 14
#define PAYLOAD_CAMERA_EO_APERTURE_F6_8                 11
#define PAYLOAD_CAMERA_EO_APERTURE_F7_3                 10
#define PAYLOAD_CAMERA_EO_APERTURE_F8_7                 8
#define PAYLOAD_CAMERA_EO_APERTURE_F9_6                 7
#define PAYLOAD_CAMERA_EO_APERTURE_F10_0                 6
#define PAYLOAD_CAMERA_EO_APERTURE_F11_0                 5  

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
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_MANUAL              0
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_ZOOM_TRIGGER        1
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_NEAR           2
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_FAR            3

// EO Manual focus value can be set from 0 to 61440, step 10
#define PAYLOAD_CAMERA_VIDEO_FOCUS_VALUE              "C_V_FV"

// EO ICR modes
#define PAYLOAD_CAMERA_EO_ICR_MODE      "C_V_ICR"
#define PAYLOAD_CAMERA_EO_ICR_MODE_AUTO     2
#define PAYLOAD_CAMERA_EO_ICR_MODE_MANUAL     3

// EO ICR AUTO threshold, values can be set from 0 to 255, step 1
#define PAYLOAD_CAMERA_EO_ICR_MODE_AUTO_THRESHOLD      "C_V_ICR_THR"

// EO ICR MANUAL modes
#define PAYLOAD_CAMERA_EO_ICR_MANUAL      "C_V_ICR_MAN"
#define PAYLOAD_CAMERA_EO_ICR_MANUAL_ON      2
#define PAYLOAD_CAMERA_EO_ICR_MANUAL_OFF     3

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

enum Camera_Zoom_Value
{
   ZOOM_OUT  = -1,
   ZOOM_STOP = 0, 
   ZOOM_IN   = 1,
};

enum Camera_Focus_Value
{
   FOCUS_OUT  = -1,
   FOCUS_STOP = 0, 
   FOCUS_IN   = 1,
   FOCUS_AUTO,
};



#endif