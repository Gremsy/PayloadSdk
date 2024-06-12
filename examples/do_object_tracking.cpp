/**
 * This example wil show you how to execute the object tracking feature on the Payload
 * Only run this example when connect with the Vio payload for now
 * This example will:
 * 1. Change the view mode to EO
 * 2. Change the Tracking Mode to Object Tracking
 * 3. Turn the OSD mode to Status
 * 4. Send some position for the bounding box
 * 5. Trigger the Object tracking feature Start/Stop
 **/

#include "stdio.h"
#include"payloadSdkInterface.h"

#include <iostream>
#include <cstdlib> // for rand() and srand()
#include <ctime>   // for time()

PayloadSdkInterface* my_payload = nullptr;
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
void all_threads_init();
void handle_tracking();
void onPayloadStatusChanged(int event, double* param);

enum tracking_cmd_t{
	TRACK_IDLE = 0,
	TRACK_ACT = 1
};

pthread_t thrd_tracking;
float track_pos_x = 0, track_pos_y = 0, track_status = 0;

int main(int argc, char *argv[]){
	printf("Starting Do Object Tracking example...\n");
	signal(SIGINT,quit_handler);

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// check connection
	my_payload->checkPayloadConnection();

	// Init the environment
	// change view mode to EO
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
	// change tracking mode to Object tracking
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE, PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING, PARAM_TYPE_UINT32);
	// change OSD mode to Status
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_OSD_MODE, PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS, PARAM_TYPE_UINT32);

	// init the status messages rate
	my_payload->setParamRate(PARAM_TRACK_POS_X, 100);
	my_payload->setParamRate(PARAM_TRACK_POS_Y, 100);
	my_payload->setParamRate(PARAM_TRACK_STATUS, 100);


	// init threads
	all_threads_init();

	// Initialize random seed
    std::srand(static_cast<unsigned int>(std::time(0)));

	// check payload messages
	while(!time_to_exit){

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

void*
start_thrd_tracking(void *args)
{
    // run the object's read thread
    handle_tracking();

    // done!
    return NULL;
}

void all_threads_init(){
	int rc = pthread_create(&thrd_tracking, NULL, &start_thrd_tracking, NULL);
	if (rc){
		std::cout << "\nError: Can not create thread!" << rc << std::endl;
		return;
	}
	std::cout << "Thread created\n" << std::endl;

}

void handle_tracking(){
	while(!time_to_exit){
		int random_w = std::rand() % 1921 - 20; // 1920 + 1 to include 1920
		int random_h = std::rand() % 1079 - 20; // 1080 + 1 to include 1080
		my_payload->setPayloadObjectTrackingParams(TRACK_ACT, random_w, random_h);

		// sleep for 2s
		usleep(2000000);

		// check tracking status
		if(track_status){
			printf("Object was tracked \n");
			printf("Keep this object for 5 seconds \n");
			usleep(5000000); // sleep for 5 secs

			// release object
			my_payload->setPayloadObjectTrackingParams(TRACK_IDLE, random_w, random_h);
			usleep(3000000); // sleep for 3 secs
		}
		else{
			printf("Lost object \n");
			printf("Try catch another object \n");
			my_payload->setPayloadObjectTrackingParams(TRACK_IDLE, random_w, random_h);
			usleep(1000000);
		}
	}
}

void onPayloadStatusChanged(int event, double* param){
	
	switch(event){
	case PAYLOAD_PARAMS:{
		// param[0]: param index
		// param[1]: value
		if(param[0] == PARAM_TRACK_POS_X){
			track_pos_x = param[1];
		}
		else if(param[0] == PARAM_TRACK_POS_Y){
			track_pos_y = param[1];
		}
		else if(param[0] == PARAM_TRACK_STATUS){
			track_status = param[1];
		}
		else{
			break;
		}

		// printf("%s, status: %.2f, x: %.2f, y: %.2f \n", __func__, track_status, track_pos_x, track_pos_y);
		break;
	}
	default: break;
	}
}