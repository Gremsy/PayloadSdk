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
bool time_to_exit = false;

void quit_handler(int sig);

int main(int argc, char *argv[]){
	printf("Starting CaptureImage example...\n");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	my_payload->checkPayloadConnection();

    usleep(1000000);

    // set record source
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IR, PARAM_TYPE_UINT32);
	
	usleep(500000);

	while(!time_to_exit){
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_1X , PARAM_TYPE_UINT32);
        printf("Start Zoom Index 1X! \n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_2X , PARAM_TYPE_UINT32);
        printf("Start Zoom Index 2X! \n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_3X , PARAM_TYPE_UINT32);
        printf("Start Zoom Index 3X! \n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_4X , PARAM_TYPE_UINT32);
        printf("Start Zoom Index 4X! \n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_5X , PARAM_TYPE_UINT32);
        printf("Start Zoom Index 5X! \n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_6X , PARAM_TYPE_UINT32);
        printf("Start Zoom Index 6X! \n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_7X , PARAM_TYPE_UINT32);
        printf("Start Zoom Index 7X! \n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_8X , PARAM_TYPE_UINT32);
        printf("Start Zoom Index 8X! \n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_1X , PARAM_TYPE_UINT32);
        printf("Start Zoom Index 1X! \n");
        usleep(200000); // sleep 2s

        // close payload interface
        try {
            my_payload->sdkQuit();
        }
        catch (int error){}
        
        exit(0);
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
