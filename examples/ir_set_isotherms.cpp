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

void quit_handler(int sig);

int main(int argc, char *argv[]){
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	my_payload->checkPayloadConnection();

    usleep(1000000);
	// set view source
    printf("Set view source to IR! \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IR, PARAM_TYPE_UINT32);
    usleep(1000000);

	printf("Enable IR Isotherms with high GAIN, sleep 5s ... \n");
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERMS, PAYLOAD_CAMERA_IR_ISOTHERMS_ENABLE , PARAM_TYPE_UINT32);
    usleep(100000);
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERMS_GAIN, PAYLOAD_CAMERA_IR_ISOTHERMS_HIGH_GAIN , PARAM_TYPE_UINT32);
    usleep(5000000); // sleep 5s

	printf("Switch low GAIN, sleep 5s ... \n");
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERMS_GAIN, PAYLOAD_CAMERA_IR_ISOTHERMS_LOW_GAIN , PARAM_TYPE_UINT32);
    usleep(5000000); // sleep 5s

	printf("Disable IR Isotherms.\n");
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ISOTHERMS, PAYLOAD_CAMERA_IR_ISOTHERMS_DISABLE , PARAM_TYPE_UINT32);
    usleep(100000);

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
