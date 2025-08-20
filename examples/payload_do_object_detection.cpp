#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <string>
using namespace std;

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

void quit_handler(int sig);

int main(int argc, char *argv[]){
	printf("Starting Set gimbal mode example...\n");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	my_payload->checkPayloadConnection();

	#ifndef ZIO
	// set view source
    printf("Set view source to EO! \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
	usleep(500000);
	#endif

	printf("Enable object detection, delay in 5 secs \n");
    #if defined VIO 
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE, PAYLOAD_CAMERA_TRACKING_OBJ_DETECTION, PARAM_TYPE_UINT32);
    #elif defined MB1 || defined ZIO
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_OBJECT_DETECTION, PAYLOAD_CAMERA_OBJECT_DETECTION_ENABLE, PARAM_TYPE_UINT32);
    #endif
	usleep(5000000);

	printf("Disable object detection. Exit! \n");
    #if defined VIO 
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE, PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING, PARAM_TYPE_UINT32);
    #elif defined MB1 || defined ZIO
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_OBJECT_DETECTION, PAYLOAD_CAMERA_OBJECT_DETECTION_DISABLE, PARAM_TYPE_UINT32);
    #endif
	usleep(500000);

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