#ifndef PAYLOADSDK_INTERFACE_H_
#define PAYLOADSDK_INTERFACE_H_

#include <iostream>
#include "payloadsdk.h"
#include "gimbal_protocol_v2.h"

typedef void (*payload_status_callback_t)(int event, double* param);
typedef void (*payload_streamInfo_callback_t)(int event, char* param_char, double* param_double);

enum payload_status_event_t{
	PAYLOAD_CAM_CAPTURE_STATUS = 0,
	PAYLOAD_CAM_STORAGE_INFO,
	PAYLOAD_CAM_SETTINGS,
	PAYLOAD_CAM_PARAM_VALUE,

	PAYLOAD_GB_ATTITUDE,

	PAYLOAD_CAM_INFO,
	PAYLOAD_CAM_STREAMINFO,

	PAYLOAD_PARAMS,
};

enum payload_param_t{
	PARAM_EO_ZOOM_LEVEL = 0,
	PARAM_IR_ZOOM_LEVEL,

	PARAM_LRF_RANGE,
	PARAM_COUNT
};

class PayloadSdkInterface
{

public:

	PayloadSdkInterface();
	PayloadSdkInterface(T_ConnInfo data);
	~PayloadSdkInterface();

	void regPayloadStatusChanged(payload_status_callback_t func);
  	payload_status_callback_t __notifyPayloadStatusChanged = NULL;

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
	 * Move gimbal
	 * @param pitch_spd: speed to move pitch axis, 0 to 90
	 * @param yaw_spd: speed to move yaw aixs, 0 to 90
	 **/
	void moveGimbal(float pitch_spd, float yaw_spd);

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
	void setPayloadCameraCaptureImage();

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

private:
	Autopilot_Interface* payload_interface;

	pthread_t thrd_recv;
	pthread_t thrd_request_params;

	uint8_t payload_ctrl_type = CONTROL_METHOD;
	Generic_Port *port;
	Generic_Port *port_quit = nullptr;
	char *payload_uart_port = (char*)"/dev/ttyUSB0";
	int payload_uart_baud = 115200;
	const char *udp_ip_target = (char*)"192.168.12.250";	// This is an ip address of the payload
    int udp_port = 14566;									// Do not change

	uint8_t SYS_ID = 1;
	uint8_t COMP_ID = MAV_COMP_ID_ONBOARD_COMPUTER;

	bool time_to_exit = false;

	uint16_t paramRate[PARAM_COUNT];


public:
	/* for proceed gSDK to communicate with gimbal */
	Gimbal_Protocol_V2* myGimbal = nullptr;
	Serial_Port* myGimbalPort = nullptr;
	mavlink_system_t _system_id;
	mavlink_system_t _gimbal_id;

	void initGimbal(Serial_Port* port);
	/*!<@brief: used to rotate gimbal for each axis depend on angular rate or angle mode
	 * @para1,2,3 : value for each axis
	 * @para4 : Angular rate or angle mode
	 * */
	void setGimbalSpeed(float spd_pitch, float spd_roll, float spd_yaw, Gimbal_Protocol::input_mode_t mode);
	/**
	 * set gimbal mode
	 * (LOCK , FOLLOW , OFF)
	 * */
	void setGimbalMode(Gimbal_Protocol::control_mode_t mode);

	/**
	 * set gimbal reset mode
	 * */
	void setGimbalResetMode(Gimbal_Protocol::gimbal_reset_mode_t reset_mode);
	/**
	 * turn gimbal power on 
	 * */
	void setGimbalPowerOn();
	/**
	 * turn gimbal power off
	 * */
	void setGimbalPowerOff();
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

	// handle receive message
	void payload_recv_handle();

	// handle request message
	void payload_request_handle();

private:
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