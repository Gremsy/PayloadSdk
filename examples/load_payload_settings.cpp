#include "stdio.h"
#include <pthread.h>
#include <cstdlib>

#include"payloadSdkInterface.h"

T_ConnInfo s_conn = {
	CONTROL_UART,
	"/dev/ttyUSB0",
	115200
};

PayloadSdkInterface* my_payload = nullptr;

void onPayloadStatusChanged(int event, double* param);

void quit_handler(int sig);

void _handle_msg_param_ext_value(mavlink_message_t* msg);

int main(int argc, char *argv[]){
	printf("Starting LoadPayloadSettings example...\n");
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
	
	// request to read all settings of payload
	my_payload->getPayloadCameraSettingList();

	while(1){
		// main loop
		usleep(1000);
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
	printf("%s %d \n", __func__, event);

	switch(event){
	case PAYLOAD_CAM_PARAM_VALUE:{
		// param[0]: param_index
		// param[1]: value

		printf(" --> Param_id: %.2f, value: %.2f\n", param[0], param[1]);
		break;
	}
	default: break;
	}
}