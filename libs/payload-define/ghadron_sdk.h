#ifndef GHADRON_SDK_H
#define GHADRON_SDK_H

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
#define PAYLOAD_CAMERA_VIEW_SYNC                4

#define PAYLOAD_CAMERA_RECORD_SRC             "C_V_REC"
#define PAYLOAD_CAMERA_RECORD_BOTH              0
#define PAYLOAD_CAMERA_RECORD_EO                1
#define PAYLOAD_CAMERA_RECORD_IR                2
#define PAYLOAD_CAMERA_RECORD_OSD               5

#define PAYLOAD_CAMERA_STORAGE                "STORAGE"
#define PAYLOAD_CAMERA_STORAGE_INTERNAL         0
#define PAYLOAD_CAMERA_STORAGE_SDCARD           1

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
    ZOOM_IR_2X = 2,
    ZOOM_IR_4X = 4,
    ZOOM_IR_8X = 8,
    ZOOM_IR_12X = 12,
    ZOOM_IR_16X = 16,
    ZOOM_IR_20X = 20,
    ZOOM_IR_24X = 24,
    ZOOM_IR_28X = 28,
    ZOOM_IR_32X = 32,
    ZOOM_IR_36X = 36,
    ZOOM_IR_40X = 40,
    ZOOM_IR_44X = 44,
    ZOOM_IR_48X = 48
};

#endif