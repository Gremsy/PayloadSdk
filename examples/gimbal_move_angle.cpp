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

pthread_t thrd_recv;
pthread_t thrd_gstreamer;

bool gstreamer_start();
void gstreamer_terminate();
void *start_loop_thread(void *threadid);


bool all_threads_init();
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
	usleep(100000);

	printf("Set gimbal RC mode \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, PAYLOAD_CAMERA_RC_MODE_STANDARD, PARAM_TYPE_UINT32);
	usleep(100000);

	printf("Move gimbal yaw to 90 deg, delay in 5secs \n");
	my_payload->setGimbalSpeed(0, 0 , 90, INPUT_ANGLE);
	usleep(5000000);

	printf("Move gimbal yaw to -90 deg, delay in 5secs \n");
	my_payload->setGimbalSpeed(-0, -0 , -90, INPUT_ANGLE);
	usleep(5000000);

	printf("Move gimbal yaw to 0 deg, delay in 5secs \n");
	my_payload->setGimbalSpeed(0, 0 , 0, INPUT_ANGLE);
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