#include "stdio.h"
#include"payloadSdkInterface.h"

PayloadSdkInterface* my_payload = nullptr;
void onPayloadStatusChanged(int event, double* param);

bool time_to_exit = false;

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

int main(int argc, char *argv[]){
	printf("Starting ConnectPayload example...\n");
	signal(SIGINT,quit_handler);

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// check connection
	my_payload->checkPayloadConnection();

	// register to get status 
	my_payload->setParamRate(PARAM_EO_ZOOM_LEVEL, 1000);
	my_payload->setParamRate(PARAM_IR_ZOOM_LEVEL, 1000);

	// set view source
    printf("Set view source to EO/IR! \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IREO, PARAM_TYPE_UINT32);

	// change EO zoom mode to Super resolution
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION, PARAM_TYPE_UINT32);

	// check payload messages
	while(1){
		// zoom EO to 1x
		printf("zoom EO to 1x \n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_1X, PARAM_TYPE_UINT32);
		// sleep for 3 secs
		usleep(3000000);

		// zoom EO to 4x
		printf("zoom EO to 4x \n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_4X, PARAM_TYPE_UINT32);
		// sleep for 3 secs
		usleep(3000000);

		// zoom IR to 1x
		printf("zoom IR to 1x \n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_1X, PARAM_TYPE_UINT32);
		// sleep for 3 secs
		usleep(3000000);

		// zoom IR to 4x
		printf("zoom IR to 4x \n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_4X, PARAM_TYPE_UINT32);
		// sleep for 3 secs
		usleep(3000000);

		// do nothing
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
    
    switch(event){
    case PAYLOAD_ACK:{
        printf(" --> Got ack, from command: %.f - result: %.2f\n", param[0], param[1]);
        break;
    }
	case PAYLOAD_PARAM_EXT_ACK:{
		printf(" --> Got ext_ack, result %.2f\n", param[0]);
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
		break;
	}
    default: break;
    }
}