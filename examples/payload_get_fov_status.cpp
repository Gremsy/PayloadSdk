/**
 * This sample will show you how to get the FOV_STATUS from the payload
 **/

#include "stdio.h"
#include"payloadSdkInterface.h"

#include <iostream>
#include <chrono>

PayloadSdkInterface* my_payload = nullptr;

#if (CONTROL_METHOD == CONTROL_UART)
T_ConnInfo s_conn = {
    CONTROL_UART,
    payload_uart_port,
    payload_uart_baud
};
#else
T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};
#endif

void quit_handler(int sig);
void onPayloadStatusChanged(int event, double* param);

int main(int argc, char *argv[]){
	printf("Starting SendSystemTime example...\n");
	signal(SIGINT,quit_handler);

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// check connection
	my_payload->checkPayloadConnection();

	int msg_cnt = 0;
	int boot_time_ms = 0;

	while(1){
		
		my_payload->getPayloadCameraFOVStatus(CAMERA_EO);
		usleep(500000); // 500ms, 2Hz

		my_payload->getPayloadCameraFOVStatus(CAMERA_IR);
		usleep(500000); // 500ms, 2Hz
	}

	return 0;
}

void quit_handler( int sig ){
    printf("\n");
    printf("TERMINATING AT USER REQUEST \n");
    printf("\n");

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    // end program here
    exit(0);
}

void onPayloadStatusChanged(int event, double* param){
	// return;
	switch(event){
	case PAYLOAD_PARAM_CAM_FOV_STATUS:{
		// param[0]: camera id
		// param[1]: hfov
		// param[2]: vfov

		printf("Camera ID: %.2f - HFOV: %.2f - VFOV: %.2f \n", param[0], param[1], param[2]);
		break;
	}
	default: break;
	}
}