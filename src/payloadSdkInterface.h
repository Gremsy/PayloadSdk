#ifndef PAYLOADSDK_INTERFACE_H_
#define PAYLOADSDK_INTERFACE_H_

#include <iostream>
#include "payloadsdk.h"

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

	uint8_t SYS_ID = 1;
	uint8_t COMP_ID = MAV_COMP_ID_ONBOARD_COMPUTER;
};
#endif