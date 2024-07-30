#ifndef VIO_SDK_H
#define VIO_SDK_H

#define PAYLOAD_CAMERA_TRACKING_MODE 	"TRACK_MODE"
#define PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING 	    0
#define PAYLOAD_CAMERA_TRACKING_OBJ_DETECTION 		1

#define PAYLOAD_CAMERA_VIDEO_OSD_MODE    "OSD_MODE"
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_DISABLE       0
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_DEBUG         1
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS        2

#define PAYLOAD_CAMERA_RC_MODE 			"RC_MODE"
#define PAYLOAD_CAMERA_RC_MODE_GREMSY 		0
#define PAYLOAD_CAMERA_RC_MODE_STANDARD 		1

#define PAYLOAD_CAMERA_VIDEO_FLIP        "C_V_FLIP"
#define PAYLOAD_CAMERA_VIDEO_FLIP_OFF       3
#define PAYLOAD_CAMERA_VIDEO_FLIP_ON        2


#define PAYLOAD_CAMERA_VIDEO_DEFOG        "C_V_DEFOG"
#define PAYLOAD_CAMERA_VIDEO_DEFOG_OFF       0
#define PAYLOAD_CAMERA_VIDEO_DEFOG_ON        1

#define PAYLOAD_CAMERA_VIDEO_DEFOG_LEVEL  "C_V_DEFOG_LV"
#define PAYLOAD_CAMERA_VIDEO_DEFOG_LOWEST       0
#define PAYLOAD_CAMERA_VIDEO_DEFOG_LOW          1
#define PAYLOAD_CAMERA_VIDEO_DEFOG_MID          2
#define PAYLOAD_CAMERA_VIDEO_DEFOG_HIGH         3

#define PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE              "C_V_AE"
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_AUTO                  0
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_MANUAL                3
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_SHUTTER               10
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_IRIS                  11
#define PAYLOAD_CAMERA_VIDEO_EXPOSURE_BRIGHT                13

#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED              "C_V_SP"
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_10             13
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_20             14
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_50             17
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_100             20
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_125             21
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_500             25
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_725             26
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1000             27
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1500             28
#define PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_2000             30

// Aperture value can be set from 0 to 25, step 1
#define PAYLOAD_CAMERA_VIDEO_APERTURE_VALUE              "C_V_IrP"  

// Bright value can be set from 0 to 41, step 1
#define PAYLOAD_CAMERA_VIDEO_BRIGHT_VALUE              "C_V_BrP"

#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE              "C_V_WB"
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_AUTO             0
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_INDOOR           1
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_OUTDOOR          2
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_ONE_PUSH         3
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_ATW              4
#define PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_MANUAL           5

#define PAYLOAD_CAMERA_VIDEO_ZOOM_MODE              "C_V_ZM_MODE"
#define PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_COMBINE              0
#define PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION     2

// Zoom super resolution value can be set from 1x to 30x
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
    ZOOM_SUPER_RESOLUTION_24X,
    ZOOM_SUPER_RESOLUTION_26X,
    ZOOM_SUPER_RESOLUTION_28X,
    ZOOM_SUPER_RESOLUTION_30X
};

// Zoom super resolution value can be set from 1x to 240x
#define PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_FACTOR              "C_V_ZM_CB_LV"
enum _zoom_combine_factor{
    ZOOM_COMBINE_1X = 0,
    ZOOM_COMBINE_10X,
    ZOOM_COMBINE_20X,
    ZOOM_COMBINE_40X,
    ZOOM_COMBINE_80X,
    ZOOM_COMBINE_120X,
    ZOOM_COMBINE_240X
};

#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE              "C_V_FM"
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_MANUAL              0
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_ZOOM_TRIGGER        1
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_NEAR           2
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_FAR            3

// Manual focus value can be set from 0 to 61440, step 10
#define PAYLOAD_CAMERA_VIDEO_FOCUS_VALUE              "C_V_FV"

#define PAYLOAD_CAMERA_GIMBAL_MODE              "GB_MODE"
#define PAYLOAD_CAMERA_GIMBAL_MODE_OFF              0
#define PAYLOAD_CAMERA_GIMBAL_MODE_LOCK             1
#define PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW           2
#define PAYLOAD_CAMERA_GIMBAL_MODE_RESET            4

#define PAYLOAD_CAMERA_VIEW_SRC             "C_SOURCE"
#define PAYLOAD_CAMERA_VIEW_EOIR                0
#define PAYLOAD_CAMERA_VIEW_EO                  1
#define PAYLOAD_CAMERA_VIEW_IR                  2
#define PAYLOAD_CAMERA_VIEW_IREO                3
#define PAYLOAD_CAMERA_VIEW_SYNC                4

#define PAYLOAD_CAMERA_RECORD_SRC             "C_V_REC"
#define PAYLOAD_CAMERA_RECORD_BOTH              0
#define PAYLOAD_CAMERA_RECORD_EO                1
#define PAYLOAD_CAMERA_RECORD_IR                2
#define PAYLOAD_CAMERA_RECORD_OSD               5

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