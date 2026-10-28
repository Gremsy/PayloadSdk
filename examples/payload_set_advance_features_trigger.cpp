/**
 * This sample will send the commands to trigger the advance features
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
	printf("Starting trigger Advance features example...\n");
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

	int rq_feature_id = 0;
	// The id of the feature need to be triggered
	// - 0: EO EIS
	// - 1: EO Image enhance (Not ready now)
	// - 2: IR EIS (Not ready now)
	// - 3: IR Image enhance
	// - 4: Payload yaw drift compensation

	while(1){
		// send command to restart the payload app
		rq_feature_id = 4;
		printf("Set feature: %d to %s \n", rq_feature_id, "On");
		my_payload->setPayloadTriggerAdvanceFeatures(rq_feature_id, true);
		usleep(5000000);
		printf("Set feature: %d to %s \n", rq_feature_id, "Off");
		my_payload->setPayloadTriggerAdvanceFeatures(rq_feature_id, false);
		usleep(1000000);

		printf("Exit after sending the trigger command. \n");
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