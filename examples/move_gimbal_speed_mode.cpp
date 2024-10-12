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
	usleep(100000);

	printf("Set gimbal RC mode \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, PAYLOAD_CAMERA_RC_MODE_STANDARD, PARAM_TYPE_UINT32);
	usleep(100000);

	printf("Move gimbal yaw to the right 20 deg/s, delay in 5secs \n");
	my_payload->setGimbalSpeed(0, 0 , 20, INPUT_SPEED);
	usleep(5000000);

	printf("Move gimbal yaw to the left 20 deg/s, delay in 5secs \n");
	my_payload->setGimbalSpeed(-0, -0 , -20, INPUT_SPEED);
	usleep(5000000);

	printf("Keep gimbal stop, delay in 5secs \n");
	my_payload->setGimbalSpeed(0, 0 , 0, INPUT_SPEED);
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

void onPayloadStatusChanged(int event, double* param){
	
	switch(event){
	case PAYLOAD_GB_ATTITUDE:{
		// param[0]: pitch
		// param[1]: roll
		// param[2]: yaw

		printf("Pich: %.2f - Roll: %.2f - Yaw: %.2f\n", param[0], param[1], param[2]);
		break;
	}
	default: break;
	}
}