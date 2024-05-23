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
bool time_to_exit = false;

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
	
	// change payload zoom mode to SuperResolution
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION, PARAM_TYPE_UINT32);

	
	while(!time_to_exit){
		printf("Move gimbal yaw to 90 deg, zoom in to 20x, delay in 5secs \n");
		my_payload->setGimbalSpeed(0, 0 , 90, Gimbal_Protocol::INPUT_ANGLE);
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_20X, PARAM_TYPE_UINT32);
		usleep(5000000);

		printf("Move gimbal yaw to -90 deg, zoom out to 1x, delay in 5secs \n");
		my_payload->setGimbalSpeed(-0, -0 , -90, Gimbal_Protocol::INPUT_ANGLE);
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_1X, PARAM_TYPE_UINT32);
		usleep(5000000);

		printf("Move gimbal yaw to 0 deg, zoom in to 10x, delay in 5secs \n");
		my_payload->setGimbalSpeed(0, 0 , 0, Gimbal_Protocol::INPUT_ANGLE);
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_10X, PARAM_TYPE_UINT32);
		usleep(500000);

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