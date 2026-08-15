#ifndef PAYLOADSDK_INTERFACE_H_
#define PAYLOADSDK_INTERFACE_H_

#include <iostream>
#include <chrono> // for get time
#include <map>
#include "payloadsdk.h"
#include "detection_packet.h"
#include <functional>

enum payload_status_event_t{
    PAYLOAD_CAM_CAPTURE_STATUS = 0,
    PAYLOAD_CAM_STORAGE_INFO,
    PAYLOAD_COMP_INFO,
    PAYLOAD_CAM_SETTINGS,
    PAYLOAD_CAM_PARAMS,

    PAYLOAD_GB_ATTITUDE,
    PAYLOAD_GB_PARAMS,
    PAYLOAD_ACK,

    PAYLOAD_CAM_INFO,
    PAYLOAD_CAM_STREAMINFO,

    PAYLOAD_PARAMS,
    PAYLOAD_PARAM_EXT_ACK,

    PAYLOAD_PARAM_CAM_FOV_STATUS,
    PAYLOAD_PARAM_DISTANCE_SENSOR,

    PAYLOAD_RECORD_STATUS
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
    PARAM_PAYLOAD_GPS_LON,
    PARAM_PAYLOAD_GPS_LAT,
    PARAM_PAYLOAD_GPS_ALT,
    PARAM_PAYLOAD_APP_VER_X,
    PARAM_PAYLOAD_APP_VER_Y,
    PARAM_PAYLOAD_APP_VER_Z,
    PARAM_CAM_VIEW_MODE,
    PARAM_CAM_REC_SOURCE,
    PARAM_CAM_IR_TYPE,
    PARAM_CAM_IR_PALETTE_ID,
    PARAM_CAM_IR_FFC_MODE,
    PARAM_GIMBAL_MODE,

    PARAM_IR_TEMP_MAX,
    PARAM_IR_TEMP_MIN,
    PARAM_IR_TEMP_MEAN,

    PARAM_COUNT
};

struct {
        const uint8_t index;
        const char *id;
        float value;
        uint16_t msg_rate; // in ms
        uint16_t tick_ms;

} payloadParams[PARAM_COUNT] = {

    {PARAM_EO_ZOOM_LEVEL,   "EO_ZOOM", 0, 0, 0},
    {PARAM_IR_ZOOM_LEVEL,   "IR_ZOOM", 0, 0, 0},
    {PARAM_LRF_RANGE,       "LRF_RANGE", 0, 0, 0},
    {PARAM_TRACK_POS_X,     "TRK_POS_X", 0, 0, 0},
    {PARAM_TRACK_POS_Y,     "TRK_POS_Y", 0, 0, 0},
    {PARAM_TRACK_POS_W,     "TRK_POS_W", 0, 0, 0},
    {PARAM_TRACK_POS_H,     "TRK_POS_H", 0, 0, 0},
    {PARAM_TRACK_STATUS,    "TRK_STATUS", 0, 0, 0},
    {PARAM_LRF_OFSET_X,     "LRF_OFFSET_X", 0, 0, 0},
    {PARAM_LRF_OFSET_Y,     "LRF_OFFSET_Y", 0, 0, 0},
    {PARAM_TARGET_COOR_LON, "TARGET_LON", 0, 0, 0},
    {PARAM_TARGET_COOR_LAT, "TARGET_LAT", 0, 0, 0},
    {PARAM_TARGET_COOR_ALT, "TARGET_ALT", 0, 0, 0},
    {PARAM_PAYLOAD_GPS_LON, "PAY_LON", 0, 0, 0},
    {PARAM_PAYLOAD_GPS_LAT, "PAY_LAT", 0, 0, 0},
    {PARAM_PAYLOAD_GPS_ALT, "PAY_ALT", 0, 0, 0},
    {PARAM_PAYLOAD_APP_VER_X, "APP_VER_X", 0, 0, 0},
    {PARAM_PAYLOAD_APP_VER_Y, "APP_VER_Y", 0, 0, 0},
    {PARAM_PAYLOAD_APP_VER_Z, "APP_VER_Z", 0, 0, 0},
    {PARAM_CAM_VIEW_MODE,       "VIEW_MODE", 0, 0, 0},
    {PARAM_CAM_REC_SOURCE,      "REC_SRC", 0, 0, 0},
    {PARAM_CAM_IR_TYPE,         "IR_TYPE", 0, 0, 0},
    {PARAM_CAM_IR_PALETTE_ID,   "PALETTE_ID", 0, 0, 0},
    {PARAM_CAM_IR_FFC_MODE,     "FFC_MODE", 0, 0, 0},
    {PARAM_GIMBAL_MODE,         "GB_MODE", 0, 0, 0},

    {PARAM_IR_TEMP_MAX,         "IR_TEMP_MAX", 0, 0, 0},
    {PARAM_IR_TEMP_MIN,         "IR_TEMP_MIN", 0, 0, 0},
    {PARAM_IR_TEMP_MEAN,         "IR_TEMP_MEAN", 0, 0, 0},
};

typedef enum{
    PAYLOADSDK_ZOOM_IN    = 1,
    PAYLOADSDK_ZOOM_OUT   = -1,
    PAYLOADSDK_ZOOM_STOP  = 0,
    PAYLOADSDK_ZOOM_WIDE  = 2,
    PAYLOADSDK_ZOOM_TELE  = 3,
    PAYLOADSDK_ZOOM_POS   = 4
}payloadsdk_zoom_mode_t;

static std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();;
static long long _getElapsedTimeInMs(){
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    return elapsed.count();
}

/* PayloadSDK log.
   class: must be a pointer to class
 */
#define SDK_DEBUG
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
    typedef std::function<void(int event, std::vector<std::string> info)> payload_info_callback_t;
    typedef std::function<void(int event, double* param)> payload_status_callback_t;
    typedef std::function<void(int event, char* param_char, double* param_double)> payload_param_callback_t;
    typedef std::function<void(int event, char* param_char, double* param_double)> payload_streamInfo_callback_t;
    typedef std::function<void(int event, char* param_char, double* param_double)> payload_recordInfo_callback_t;
    typedef std::function<void(const det_packet_t& pkt)> payload_detection_callback_t;
    typedef std::function<void(mavlink_message_t msg)> payload_heartbeat_callback_t;

    PayloadSdkInterface();
    PayloadSdkInterface(T_ConnInfo data);
    ~PayloadSdkInterface();

    void regPayloadInfoChanged(payload_info_callback_t func);
    payload_info_callback_t __notifyPayloadInfoChanged = NULL;

    void regPayloadStatusChanged(payload_status_callback_t func);
    payload_status_callback_t __notifyPayloadStatusChanged = NULL;

    void regPayloadParamChanged(payload_param_callback_t func);
    payload_param_callback_t __notifyPayloadParamChanged = NULL;

    void regPayloadStreamChanged(payload_streamInfo_callback_t func);
    payload_streamInfo_callback_t __notifyPayloadStreamChanged = NULL;

    void regPayloadRecordInfoChanged(payload_recordInfo_callback_t func);
    payload_recordInfo_callback_t __notifyPayloadRecordChanged = NULL;

    void regPayloadHeartbeatChanged(payload_heartbeat_callback_t func);
    payload_heartbeat_callback_t __notifyPayloadHeartbeatChanged = NULL;

    void regPayloadDetectionChanged(payload_detection_callback_t func);
    payload_detection_callback_t __notifyPayloadDetectionChanged = NULL;


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
    uint8_t getNewMessage(mavlink_message_t& new_msg);
    uint8_t getNewMewssage(mavlink_message_t& new_msg);

    /**
     * set payload's camera parameter
     **/
    void setPayloadCameraParam(char param_id[], uint32_t param_value, uint8_t param_type);

    /**
     * get all payload's settings
     **/
    void getPayloadCameraSettingList();

    /**
     * get payload's setting by id
     **/
    void getPayloadCameraSettingByID(char* ID);

    /**
     * get payload's setting by index
     **/
    void getPayloadCameraSettingByIndex(uint8_t idx);

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
    void getPayloadCameraStreamingInformation(uint32_t stream_id = 0);

    /**
     * get payload's component information
     * for the serial number
     **/
    void getPayloadComponentBasicInformation();

    /**
     * set payload's gimbal param
     **/
    void setPayloadGimbalParamByID(char* param_id, float param_value);

    /**
     * Send command to trigger gimbal gyro calibration
     **/
    void sendPayloadGimbalCalibGyro();

    /**
     * Send command to trigger gimbal accel calibration
     **/
    void sendPayloadGimbalCalibAccel();

    /**
     * Send command to trigger gimbal motor calibration
     **/
    void sendPayloadGimbalCalibMotor();

    /**
     * Send command to trigger gimbal search home
     **/
    void sendPayloadGimbalSearchHome();

    /**
     * Send command to trigger gimbal auto tune
     **/
    void sendPayloadGimbalAutoTune(bool status);

    /**
     * get all gimbal's settings
     **/
    void getPayloadGimbalSettingList();

    /**
     * get specific gimbal param by id string
     **/
    void getPayloadGimbalSettingByID(char* ID);

    /**
     * get specific gimbal param by index
     **/
    void getPayloadGimbalSettingByIndex(uint8_t idx);

    /**
     * set payload's camera mode
     **/
    void setPayloadCameraMode(CAMERA_MODE mode);

    /**
     * set payload's camera capture image
     **/
    void setPayloadCameraCaptureImage(float = 0);

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

    /**
     * Set IR FFC mode
     **/
    void setPayloadCameraFFCMode(uint8_t mode);

    /**
     * Get IR FFC mode
     **/
    void getPayloadCameraFFCMode(uint8_t& mode);

    /**
     * Set IR FFC trigger
     **/
    void setPayloadCameraFFCTrigg();

    /**

     * Set EO WB One Push trigger
     **/
    void setPayloadCameraWBOnePushTrigg();

    // set stream bitrate
    // bitrate is in bit per second
    void setPayloadStreamBitrate(uint32_t cam_id, uint32_t bitrate);

    // set stream resolution
    void setPayloadStreamResolution(uint32_t cam_id, uint32_t resolution_lv);

    // set stream encoder profile
    void setPayloadStreamProfile(uint32_t cam_id, uint32_t enc_profile);

    // set standby mode
    void setPayloadStandbyMode(bool mode);

    // send command to restart the apps
    void setPayloadRestartApps(uint8_t app_id);

    // send command to change external settings for EO (only on ORUSL)
    void setCameraExtSettings_SpotAE_Display(uint8_t mode);
    void setCameraExtSettings_SpotAE_Mode(uint8_t mode);
    void setCameraExtSettings_SpotAE_Position(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

    // get the current stream bitrate
    uint32_t getPayloadStreamBitrate(); 

    /// get payload camera fov status
    void getPayloadCameraFOVStatus(camera_type_t cam_type);

    void requestParamValue(uint8_t pIndex);
    void setParamRate(int pIndex, uint16_t time_ms);
    void requestMessageStreamInterval();    

private:
    Autopilot_Interface* payload_interface = nullptr;
    
    pthread_t thrd_recv;
    pthread_t thrd_request_params;

    uint8_t payload_ctrl_type = CONTROL_METHOD;
    Generic_Port *port;
    Generic_Port *port_quit = nullptr;

    bool time_to_exit = false;

    uint8_t SYS_ID_USER2 = 1;

    bool is_send_stream_request = false;

    uint32_t current_gimbal_mode;
    uint16_t current_attitude_flags;

    std::map<uint16_t, StatusTextBuffer> statustext_buffers;
public:
    /*!<@brief: used to rotate gimbal for each axis depend on angular rate or angle mode
     * @para1,2,3 : value for each axis
     * @para4 : Angular rate or angle mode
     * */
    void setGimbalSpeed(float spd_pitch, float spd_roll, float spd_yaw, input_mode_t mode);

    /**
     * Set gimbal angle
     * Use this function for control the gimbal over the limitation of the quatenion
     * If the yaw angle out of the range (-180:180)
     * If the pitch angle out of the range (-90:90)
     * If do angle control, the unit is degrees
     * * If do speed control, the unit is degrees/s
     **/
    void setGimbalMove_MAVLinkV1(float pitch, float roll, float yaw, input_mode_t mode);

    /**
     * set camera zoom ZOOM_TYPE_CONTINUOUS
     * (ZOOM_OUT, ZOOM_STOP, ZOOM_IN)
     * */
    void setCameraZoom(float zoomType,float zoomValue);

    /**
     * Custom command for zooming to the specific target
     * @target the zoom level need to be reached, from 1x to 25x (Dz Off) or 300x (Combine zoom)
     **/
    void setCameraZoomTarget(float target_level);

    /**
     * set camera focus
     * (FOCUS_OUT, FOCUS_STOP, FOCUS_IN)
     * */
    void setCameraFocus(float focusType, float focusValue=0);

    // send object tracking mode
    // 3 modes: 
    // - Stop tracking:   0
    // - Active tracking: 1 (tracking actived but in idle mode, waiting for a trigger command with position)
    // - EagleEyes:       2 (gimbal move only, no tracking trigger) 
    void setPayloadObjectTrackingMode(float mode);

    // send object tracking trigger
    void setPayloadObjectTrackingPosition(float pos_x=960, float pos_y=540, float width=128, float height=128);

    /**
     * Send the GPS information to the payload
     **/
    void sendPayloadGPSPosition(mavlink_global_position_int_t gps);

    /**
     * Send the GPS RAW information to the payload
     **/
    void sendPayloadGPSRawInt(mavlink_gps_raw_int_t gps_raw);

    /**
     * Send the Sytem Time to the payload
     **/
    void sendPayloadSystemTime(mavlink_system_time_t sys_time);

    /**
     * Send request for stream rate
     **/
    void sendPayloadRequestStreamRate(int index, uint16_t time_ms);

    // handle receive message
    void payload_recv_handle();

    // handle request message
    void payload_request_handle();

    void _handle_msg_param_ext_value(mavlink_message_t* msg);
    void _handle_msg_command_ext_ack(mavlink_message_t* msg);
    void _handle_msg_command_ack(mavlink_message_t* msg);
    void _handle_msg_storage_information(mavlink_message_t* msg);
    void _handle_msg_camera_capture_status(mavlink_message_t* msg);
    void _handle_msg_camera_settings(mavlink_message_t* msg);

    void _handle_msg_mount_orientation(mavlink_message_t* msg);
    void _handle_msg_param_value(mavlink_message_t* msg);
    void _handle_msg_debug(mavlink_message_t* msg);

    void _handle_msg_camera_stream_information(mavlink_message_t* msg);
    void _handle_msg_camera_information(mavlink_message_t* msg);

    void _handle_msg_device_attitude(mavlink_message_t* msg);
    void _handle_request_camera_fov_status(mavlink_message_t* msg);
    void _handle_request_component_info(mavlink_message_t* msg);

    void _handle_statustext(mavlink_message_t* msg);
    void _handle_distance_sensor(mavlink_message_t* msg);
    void _handle_msg_v2_extension(mavlink_message_t* msg);
};
#endif
