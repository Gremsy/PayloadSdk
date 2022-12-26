#ifndef PAYLOADSDK_INTERFACE_H_
#define PAYLOADSDK_INTERFACE_H_

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

private:
	Autopilot_Interface* payload_interface;
	Generic_Port *port_quit = nullptr;
	char *payload_uart_port = (char*)"/dev/ttyUSB0";
	int payload_uart_baud = 115200;
};
#endif