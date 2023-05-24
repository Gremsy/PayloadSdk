#ifndef PAYLOADSDK_INTERFACE_H_
#define PAYLOADSDK_INTERFACE_H_

#include <iostream>
#include "payloadsdk.h"
#include "gimbal_protocol_v2.h"

#define SDK_VERSION "1.0.0_build.06Jan2023"

class PayloadSdkInterface
{

public:

	PayloadSdkInterface();
	~PayloadSdkInterface();

	/**
	 * Init connection to payload
	 **/
	bool sdkInitConnection();

	/**
	 * Interface terminator
	 **/
	void sdkQuit();

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


private:
	Autopilot_Interface* payload_interface;
	Generic_Port *port;
	Generic_Port *port_quit = nullptr;
	char *payload_uart_port = (char*)"/dev/ttyUSB0";
	int payload_uart_baud = 115200;
	const char *udp_ip_target = (char*)"192.168.12.200";
    int udp_port = 14565;

	uint8_t SYS_ID = 1;
	uint8_t COMP_ID = MAV_COMP_ID_ONBOARD_COMPUTER;


public:
	/* for proceed gSDK to communicate with gimbal */
	Gimbal_Protocol_V2* myGimbal = nullptr;
	Serial_Port* myGimbalPort = nullptr;
	mavlink_system_t _system_id;
	mavlink_system_t _gimbal_id;

	void initGimbal(Serial_Port* port);
	void setGimbalSpeed(float spd_pitch, float spd_roll, float spd_yaw, Gimbal_Protocol::input_mode_t mode);

};
#endif