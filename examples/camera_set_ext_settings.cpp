/**
 * This sample will send the commands for Spot AE feature
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

	while(1){
		// send command to change Spot AE Display to OFF
		my_payload->setCameraExtSettings_SpotAE_Display(0);
		usleep(5000000); // do nothing

		// send command to change Spot AE Display to ON
		my_payload->setCameraExtSettings_SpotAE_Display(1);
		usleep(1000000); // do nothing

		// send command to change the power of the Spot AE Mode
		my_payload->setCameraExtSettings_SpotAE_Mode(2);
		usleep(1000000); // do nothing

		// send command to change the position of the Spot AE Mode
		my_payload->setCameraExtSettings_SpotAE_Position(6,6, 2,2);
		usleep(1000000); // do nothing

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