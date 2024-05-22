#include "stdio.h"
#include <pthread.h>
#include <cstdlib>

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

void onPayloadParamChanged(int event, char* param_char, double* param_double);
void quit_handler(int sig);

int main(int argc, char *argv[]){
	printf("Starting SetPayloadSettings example...\n");
	signal(SIGINT,quit_handler);
	
	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadParamChanged(onPayloadParamChanged);

	// check connection
	my_payload->checkPayloadConnection();
	
	// change setting of RC_MODE to STANDARD
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, PAYLOAD_CAMERA_RC_MODE_STANDARD, PARAM_TYPE_UINT32);

	// change setting of OSD_MODE to STATUS to view Zoom factor
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_OSD_MODE, PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS, PARAM_TYPE_UINT32);
	
	printf("------------------------> Init values \n");
	// request to read all settings of payload, then check the RC_MODE setting
	my_payload->getPayloadCameraSettingList();
	usleep(3000000);

	printf("\nChange some params\n");

	// change Zio zoom mode to SuperResolution
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION, PARAM_TYPE_UINT32);
	usleep(3000000);

	// request to read all settings of payload to verify
	printf("------------------------> Changed values \n");
	my_payload->getPayloadCameraSettingList();
	usleep(3000000);

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

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

void onPayloadParamChanged(int event, char* param_char, double* param){
	switch(event){
	case PAYLOAD_CAM_PARAM_VALUE:{
		// param[0]: param_index
		// param[1]: value
		printf(" --> Param_id: %s, value: %.2f\n", param_char, param[1]);
		break;
	}
	default: break;
	}
}