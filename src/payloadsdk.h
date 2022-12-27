#ifndef PAYLOADSDK_H_
#define PAYLOADSDK_H_

// mavlink communication
#include <common/mavlink.h>
#include <signal.h>
#include "autopilot_interface.h"
#include "serial_port.h"

#define CAM_PARAM_ID_LEN 16
#define CAM_PARAM_VALUE_LEN 128
typedef struct {
    union {
        float param_float;
        int32_t param_int32;
        uint32_t param_uint32;
        int16_t param_int16;
        uint16_t param_uint16;
        int8_t param_int8;
        uint8_t param_uint8;
        char bytes[CAM_PARAM_VALUE_LEN];
    };
    uint8_t type;
} cam_param_union_t;

enum param_type {
    PARAM_TYPE_UINT8 = 1,
    PARAM_TYPE_INT8,
    PARAM_TYPE_UINT16,
    PARAM_TYPE_INT16,
    PARAM_TYPE_UINT32,
    PARAM_TYPE_INT32,
    PARAM_TYPE_UINT64,
    PARAM_TYPE_INT64,
    PARAM_TYPE_REAL32,
    PARAM_TYPE_REAL64
};

#define PAYLOAD_CAMERA_TRACKING_MODE 	"TRACK_EN"
#define PAYLOAD_CAMERA_TRACKING_DISABLE 	0
#define PAYLOAD_CAMERA_TRACKING_ENABLE 		1

#define PAYLOAD_CAMERA_VIDEO_OSD_MODE    "OSD_MODE"
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_DISABLE       0
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_DEBUG         1
#define PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS        2

#define PAYLOAD_CAMERA_RC_MODE 			"RC_MODE"
#define PAYLOAD_CAMERA_RC_MODE_SINGLE 		0
#define PAYLOAD_CAMERA_RC_MODE_DUAL 		1

#define PAYLOAD_CAMERA_VIDEO_FLIP        "C_V_FLIP"
#define PAYLOAD_CAMERA_VIDEO_FLIP_OFF       0
#define PAYLOAD_CAMERA_VIDEO_FLIP_ON        1

#define PAYLOAD_CAMERA_VIDEO_OUTPUT      "C_V_OUT"
#define PAYLOAD_CAMERA_VIDEO_OUTPUT_HDMI       0
#define PAYLOAD_CAMERA_VIDEO_OUTPUT_UDP        1
#define PAYLOAD_CAMERA_VIDEO_OUTPUT_BOTH       2

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

// Zoom super resolution value can be set from 0 to 21846 (1x to 30x), step 1
#define PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_VALUE              "C_V_ZM_SR"

// Zoom super resolution value can be set from 0 to 31242 (1x to 240x), step 1
#define PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_VALUE              "C_V_ZM_CB"

#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE              "C_V_FM"
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_MANUAL              0
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_ZOOM_TRIGGER        1
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_NEAR           2
#define PAYLOAD_CAMERA_VIDEO_FOCUS_MODE_AUTO_FAR            3

// Manual focus value can be set from 0 to 61440, step 10
#define PAYLOAD_CAMERA_VIDEO_FOCUS_VALUE              "C_V_FV"

#define PAYLOAD_CAMERA_GIMBAL_MODE              "GB_MODE"
#define PAYLOAD_CAMERA_GIMBAL_MODE_OFF              0
#define PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW           1
#define PAYLOAD_CAMERA_GIMBAL_MODE_LOCK             2


#endif