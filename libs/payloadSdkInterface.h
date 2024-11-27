#ifndef PAYLOADSDK_INTERFACE_H_
#define PAYLOADSDK_INTERFACE_H_

#include <iostream>
#include <chrono> // for get time
#include "payloadsdk.h"
#include <functional>

enum payload_status_event_t{
    PAYLOAD_CAM_CAPTURE_STATUS = 0,
    PAYLOAD_CAM_STORAGE_INFO,
    PAYLOAD_CAM_SETTINGS,
    PAYLOAD_CAM_PARAM_VALUE,

    PAYLOAD_GB_ATTITUDE,
    PAYLOAD_GB_ACK,

    PAYLOAD_CAM_INFO,
    PAYLOAD_CAM_STREAMINFO,

    PAYLOAD_PARAMS,
};

enum {
    PARAM_EO_ZOOM_LEVEL = 0,
    PARAM_IR_ZOOM_LEVEL,
    PARAM_LRF_RANGE,

    PARAM_TRACK_POS_X,
    PARAM_TRACK_POS_Y,
    PARAM_TRACK_POS_W,
    PARAM_TRACK_POS_H,
    PARAM_TRACK_STATUS,

    PARAM_LRF_OFSET_X,
    PARAM_LRF_OFSET_Y,

    PARAM_TARGET_COOR_LON,
    PARAM_TARGET_COOR_LAT,
    PARAM_TARGET_COOR_ALT,

    PARAM_PAYLOAD_APP_VER_X,
    PARAM_PAYLOAD_APP_VER_Y,
    PARAM_PAYLOAD_APP_VER_Z,

    PARAM_COUNT
};

struct {
    const uint8_t index;
    const char *id;
    float value;
    uint16_t msg_rate;

} payloadParams[PARAM_COUNT] = {

    {PARAM_EO_ZOOM_LEVEL,   "EO_ZOOM", 0,0},
    {PARAM_IR_ZOOM_LEVEL,   "IR_ZOOM", 0,0},
    {PARAM_LRF_RANGE,       "LRF_RANGE", 0,0},

    {PARAM_TRACK_POS_X,     "TRK_POS_X", 0,0},
    {PARAM_TRACK_POS_Y,     "TRK_POS_Y", 0,0},
    {PARAM_TRACK_POS_W,     "TRK_POS_W", 0,0},
    {PARAM_TRACK_POS_H,     "TRK_POS_H", 0,0},
    {PARAM_TRACK_STATUS,    "TRK_STATUS", 0,0},

    {PARAM_LRF_OFSET_X,     "LRF_OFFSET_X", 0,0},
    {PARAM_LRF_OFSET_Y,     "LRF_OFFSET_Y", 0,0},

    {PARAM_TARGET_COOR_LON, "TARGET_LON", 0,0},
    {PARAM_TARGET_COOR_LAT, "TARGET_LAT", 0,0},
    {PARAM_TARGET_COOR_ALT, "TARGET_ALT", 0,0},

    {PARAM_PAYLOAD_APP_VER_X, "APP_VER_X", 0,0},
    {PARAM_PAYLOAD_APP_VER_Y, "APP_VER_Y", 0,0},
    {PARAM_PAYLOAD_APP_VER_Z, "APP_VER_Z", 0,0},
};

static std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();;
static long long _getElapsedTimeInMs(){
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    return elapsed.count();
}

/* PayloadSDK log.
   class: must be a pointer to class
 */
// #define SDK_DEBUG
#ifdef SDK_DEBUG
#define SDK_LOG(fmt, ...)               \
    printf("[%lld] SDK %s(): " fmt "\n",   \
            _getElapsedTimeInMs(),          \
            __func__,                       \
            ##__VA_ARGS__);
#else
#define SDK_LOG(fmt, ...)
    ;
#endif

class PayloadSdkInterface
{
public:
    typedef std::function<void(int event, double* param)> payload_status_callback_t;
    typedef std::function<void(int event, char* param_char, double* param_double)> payload_param_callback_t;
    typedef std::function<void(int event, char* param_char, double* param_double)> payload_streamInfo_callback_t;

    PayloadSdkInterface();
    PayloadSdkInterface(T_ConnInfo data);
    ~PayloadSdkInterface();

    void regPayloadStatusChanged(payload_status_callback_t func);
      payload_status_callback_t __notifyPayloadStatusChanged = NULL;

      void regPayloadParamChanged(payload_param_callback_t func);
      payload_param_callback_t __notifyPayloadParamChanged = NULL;

      void regPayloadStreamChanged(payload_streamInfo_callback_t func);
      payload_streamInfo_callback_t __notifyPayloadStreamChanged = NULL;

    /**
     * Init connection to payload
     **/
    bool sdkInitConnection();
    /**
     * Interface terminator
     **/
    void sdkQuit();

    bool all_threads_init();

    void checkPayloadConnection();

    /**
     * Check new message 
     **/
    uint8_t getNewMewssage(mavlink_message_t& new_msg);

    /**
     * set payload's camera parameter
     **/
    void setPayloadCameraParam(char param_id[], uint32_t param_value, uint8_t param_type);

    /**
     * get payload's settings
     **/
    void getPayloadCameraSettingList();

    /**
     * get payload's storage volume
     **/
    void getPayloadStorage();

    /**
     * get payload's capture status
     **/
    void getPayloadCaptureStatus();

    /**
     * get payload's camera mode
     **/
    void getPayloadCameraMode();

    /**
     * get payload's camera information
     **/
    void getPayloadCameraInformation();

    /**
     * get payload's camera streaming information
     **/
    void getPayloadCameraStreamingInformation();

    /**
     * set payload's camera mode
     **/
    void setPayloadCameraMode(CAMERA_MODE mode);

    /**
     * set payload's camera capture image
     **/
    void setPayloadCameraCaptureImage(int = 0);

    /**
     * set payload's camera stop image
     **/
    void setPayloadCameraStopImage();

    /**
     * set payload's camera start record video
     **/
    void setPayloadCameraRecordVideoStart();

    /**
     * set payload's camera stop record video
     **/
    void setPayloadCameraRecordVideoStop();

    void requestParamValue(uint8_t pIndex);
    void setParamRate(uint8_t pIndex, uint16_t time_ms);
    void requestMessageStreamInterval();

private:
    pthread_t thrd_recv;
    pthread_t thrd_request_params;

    uint8_t payload_ctrl_type = CONTROL_METHOD;
    Generic_Port *port;
    Generic_Port *port_quit = nullptr;

    uint8_t SYS_ID = 1;
    uint8_t COMP_ID = MAV_COMP_ID_ONBOARD_COMPUTER;

    bool time_to_exit = false;

    uint8_t SYS_ID_USER2 = 1;

    bool is_send_stream_request = false;

public:
    /*!<@brief: used to rotate gimbal for each axis depend on angular rate or angle mode
     * @para1,2,3 : value for each axis
     * @para4 : Angular rate or angle mode
     * */
    void setGimbalSpeed(float spd_pitch, float spd_roll, float spd_yaw, input_mode_t mode);
    /**
     * set camera zoom ZOOM_TYPE_CONTINUOUS
     * (ZOOM_OUT, ZOOM_STOP, ZOOM_IN)
     * */
    void setCameraZoom(float zoomType,float zoomValue);
    /**
     * set camera focus
     * (FOCUS_OUT, FOCUS_STOP, FOCUS_IN)
     * */
    void setCameraFocus(float focusType, float focusValue=0);

    /**
     * send bounding box position for object tracking feature
     **/
    void setPayloadObjectTrackingParams(float cmd, float pos_x=960, float pos_y=540);

    /**
     * Send the GPS information to the payload
     **/
    void sendPayloadGPSPosition(mavlink_global_position_int_t gps);

    /**
     * Send the Sytem Time to the payload
     **/
    void sendPayloadSystemTime(mavlink_system_time_t sys_time);

    // handle receive message
    void payload_recv_handle();

    // handle request message
    void payload_request_handle();

    void _handle_msg_param_ext_value(mavlink_message_t* msg);
    void _handle_msg_command_ack(mavlink_message_t* msg);
    void _handle_msg_storage_information(mavlink_message_t* msg);
    void _handle_msg_camera_capture_status(mavlink_message_t* msg);
    void _handle_msg_camera_settings(mavlink_message_t* msg);

    void _handle_msg_mount_orientation(mavlink_message_t* msg);
    void _handle_msg_param_value(mavlink_message_t* msg);

    void _handle_msg_camera_stream_information(mavlink_message_t* msg);
    void _handle_msg_camera_information(mavlink_message_t* msg);

};
#endif