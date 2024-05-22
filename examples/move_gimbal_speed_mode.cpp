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
	
	// change payload zoom mode to SuperResolution
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION, PARAM_TYPE_UINT32);

	/*
	 * change gimbal control combine with zoom option
	 */
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_COMBINE_ZOOM, PAYLOAD_CAMERA_GIMBAL_COMBINE_ZOOM_DISABLE, PARAM_TYPE_UINT32);
	
	
	while(!time_to_exit){
		printf("Move gimbal yaw to the right 20 deg/s, zoom in to 20x, delay in 5secs \n");
		my_payload->setGimbalSpeed(0, 0 , 120, Gimbal_Protocol::INPUT_SPEED);
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_20X, PARAM_TYPE_UINT32);
		usleep(5000000);
		
		printf("Move gimbal yaw to the left 20 deg/s, zoom in to 1x, delay in 5secs \n");
		my_payload->setGimbalSpeed(-0, -0 , -120, Gimbal_Protocol::INPUT_SPEED);
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_1X, PARAM_TYPE_UINT32);
		usleep(5000000);
		
		printf("Keep gimbal stop, zoom in to 10x, delay in 5secs \n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_10X, PARAM_TYPE_UINT32);
		my_payload->setGimbalSpeed(0, 0 , 0, Gimbal_Protocol::INPUT_SPEED);
		usleep(5000000);

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