#include "stdio.h"
#include"payloadSdkInterface.h"

// Global pointer to the payload SDK wrapper so callbacks can access it
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
	printf("[Init] Starting payload zoom control example...\n");
	signal(SIGINT,quit_handler);

	// Create the payload SDK object with the chosen connection settings
	my_payload = new PayloadSdkInterface(s_conn);

	// Initialise the connection to the payload
	my_payload->sdkInitConnection();
	printf("[Init] Waiting for payload handshake...\n");

	// Subscribe to status updates so we can read acknowledgements and zoom levels
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// Verify the payload link comes up before issuing commands
	my_payload->checkPayloadConnection();

	// Request periodic zoom level updates (in milliseconds)
	my_payload->setParamRate(PARAM_EO_ZOOM_LEVEL, 1000);
	my_payload->setParamRate(PARAM_IR_ZOOM_LEVEL, 1000);

	// Continuously demonstrate EO/IR zoom transitions until the user exits
	while(1){
		printf("\n[EO] Switching zoom mode to Super Resolution...\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION, PARAM_TYPE_UINT32);
		usleep(4000000); // allow the payload to process the mode change

		printf("[EO] Forcing view source to EO over IR composite...\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EOIR, PARAM_TYPE_UINT32);

		printf("[EO] Setting Super Resolution zoom to 1x.\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_1X, PARAM_TYPE_UINT32);
		usleep(4000000);

		printf("[EO] Stepping Super Resolution zoom up to 4x.\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_4X, PARAM_TYPE_UINT32);
		usleep(4000000);

		printf("[EO] Driving Super Resolution zoom to 30x.\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_30X, PARAM_TYPE_UINT32);
		usleep(4000000);

		printf("[EO] Switching to Combine zoom mode for extended range...\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_COMBINE, PARAM_TYPE_UINT32);
		usleep(4000000);
		
		printf("[EO] Combine zoom: resetting to 1x.\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_FACTOR, ZOOM_COMBINE_1X, PARAM_TYPE_UINT32);
		usleep(4000000);

		printf("[EO] Combine zoom: jumping to 40x.\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_FACTOR, ZOOM_COMBINE_40X, PARAM_TYPE_UINT32);
		usleep(4000000);

		printf("[EO] Combine zoom: pushing to 240x.\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_COMBINE_FACTOR, ZOOM_COMBINE_240X, PARAM_TYPE_UINT32);
		usleep(4000000);

		printf("[IR] Switching view source to IR over EO composite...\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IREO, PARAM_TYPE_UINT32);

		printf("[IR] Setting zoom to 1x.\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_1X, PARAM_TYPE_UINT32);
		usleep(4000000);

		printf("[IR] Stepping zoom to 4x.\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_4X, PARAM_TYPE_UINT32);
		usleep(4000000);

		printf("[IR] Increasing zoom to 8x for maximum magnification.\n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_8X, PARAM_TYPE_UINT32);
		usleep(4000000);

		usleep(1000); // short delay before repeating the demonstration
	}

	return 0;
}

void quit_handler( int sig ){
    printf("\n[Exit] Received interrupt, shutting down cleanly...\n\n");

    // Close the interface so the payload releases the connection
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    // Terminate the sample application
    exit(0);
}

void onPayloadStatusChanged(int event, double* param){
    
    switch(event){
    case PAYLOAD_ACK:{
        printf("[Status] Command %.0f acknowledged with result %.2f\n", param[0], param[1]);
        break;
    }
	case PAYLOAD_PARAM_EXT_ACK:{
		printf("[Status] Extended ACK received: %.2f\n", param[0]);
		break;
	}
	case PAYLOAD_PARAMS:{
		// param[0]: parameter index returned by the payload
		// param[1]: latest value for that parameter
		if(param[0] == PARAM_EO_ZOOM_LEVEL){
			printf("[Status] EO zoom level reported at %.2f\n", param[1]);
		}
		else if(param[0] == PARAM_IR_ZOOM_LEVEL){
			printf("[Status] IR zoom level reported at %.2f\n", param[1]);
		}
		break;
	}
    default: break;
    }
}