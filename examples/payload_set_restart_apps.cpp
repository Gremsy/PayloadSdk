/**
 * This sample will send the commands to restart apps
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
	printf("Starting Restart apps example...\n");
	signal(SIGINT,quit_handler);

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// check connection
	my_payload->checkPayloadConnection();

	int rq_app_id = 0;
	// The app of the app need to be restarted
	// 0: payload app
	// 1: streaming app
	// 2: tracking app
	// 3: web ui app
	// 4: gimbal app

	while(1){
		// send command to restart the payload app
		rq_app_id = 4;
		my_payload->setPayloadRestartApps(rq_app_id);
		usleep(5000000); // do nothing

		printf("Exit after sending the restart command. \n");
		exit(0);
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
	switch(event){
	case PAYLOAD_ACK:{
		if(param[0] == MAV_CMD_USER_4){
			// param[0]: command
			// param[1]: result
			// param[2]: progress
			printf("Got PAYLOAD_ACK for command %.2f with status %.2f, progress: %.2f\n", param[0], param[1], param[2]);
		}
		break;
	}
	default: break;
	}
}