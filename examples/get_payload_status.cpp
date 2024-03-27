#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <string>

#include"payloadSdkInterface.h"

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

PayloadSdkInterface* my_payload = nullptr;
bool time_to_exit = false;

void quit_handler(int sig);
void onPayloadStatusChanged(int event, double* param);


int main(int argc, char *argv[]){
	printf("Starting Set gimbal mode example...\n");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// check connection
	my_payload->checkPayloadConnection();

	// set the interval for param update
	my_payload->setParamRate(PARAM_LRF_RANGE, 1000);

	while(!time_to_exit){

		usleep(10000);
	}

    
	return 0;
}

void quit_handler( int sig ){
    printf("\n");
    printf("TERMINATING AT USER REQUEST \n");
    printf("\n");

    time_to_exit = true;

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
	case PAYLOAD_GB_ATTITUDE:{
		// param[0]: pitch
		// param[1]: roll
		// param[2]: yaw

		printf("Pich: %.2f - Roll: %.2f - Yaw: %.2f\n", param[0], param[1], param[2]);
		break;
	}
	case PAYLOAD_PARAMS:{
		// param[0]: param index
		// param[1]: value
		if(param[0] == PARAM_EO_ZOOM_LEVEL){
			printf("Payload EO_ZOOM_LEVEL: %.2f \n", param[1]);
		}
		else if(param[0] == PARAM_IR_ZOOM_LEVEL){
			printf("Payload IR_ZOOM_LEVEL: %.2f \n", param[1]);
		}
		else if(param[0] == PARAM_LRF_RANGE){
			printf("Payload LRF_RANGE: %.2f \n", param[1]);
		}
		break;
	}
	default: break;
	}
}