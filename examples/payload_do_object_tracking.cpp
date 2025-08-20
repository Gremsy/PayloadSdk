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

enum tracking_mode_t{
	TRACK_STOP 		= 0,
	TRACK_ACTIVE 	= 1,
	TRACK_EAGLEEYES = 2
};

enum tracking_status_t {
    TRACK_IDLE = 0,
    TRACK_TRACKED = 1,
    TRACK_LOST = 2
};

pthread_t thrd_tracking;
float track_pos_x = 0, track_pos_y = 0, track_status = 0, track_pos_w = 0, track_pos_h = 0;

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
	#ifndef ZIO
	// change view mode to EO
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
	#endif
	// change tracking mode to Object tracking
    #if defined VIO
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE, PAYLOAD_CAMERA_TRACKING_OBJ_TRACKING, PARAM_TYPE_UINT32);
	#elif defined MB1 || defined ZIO
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_OBJECT_DETECTION, PAYLOAD_CAMERA_OBJECT_DETECTION_DISABLE, PARAM_TYPE_UINT32);
    #endif
	// change OSD mode to Status
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_OSD_MODE, PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS, PARAM_TYPE_UINT32);

	// init the status messages rate
	// if you do not want to receive the message anymore, need to set rate to 0
	my_payload->setParamRate(PARAM_TRACK_POS_X, 100);
	my_payload->setParamRate(PARAM_TRACK_POS_Y, 100);
	my_payload->setParamRate(PARAM_TRACK_POS_W, 100);
	my_payload->setParamRate(PARAM_TRACK_POS_H, 100);
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
		int random_x = std::rand() % (1921 - 20); // 1920 + 1 to include 1920
		int random_y = std::rand() % (1079 - 20); // 1080 + 1 to include 1080

		printf("Active the tracker \n");
		my_payload->setPayloadObjectTrackingMode(TRACK_ACTIVE);

		printf("Start tracking new object \n");
		// if you send the postion while the tracker is not actived, the payload will move by the EagleEyes feature (only move, without tracking)
		// Zio Payload only use random_x and random_y
		my_payload->setPayloadObjectTrackingPosition(random_x, random_y, 64, 128);

		// if you want to track the object at the center of the screen, just use
		// my_payload->setPayloadObjectTrackingPosition();
		
		#ifdef ZIO
		// sleep for 2s
		usleep(2000000);
		#else
		// sleep for 200ms
		usleep(200000);
		#endif

		// check tracking status
		if(track_status == TRACK_TRACKED){
			printf("Object was tracked. Keep this object for 5 seconds... \n");
			usleep(5000000); // sleep for 5 secs

			// release object
			my_payload->setPayloadObjectTrackingMode(TRACK_STOP);
			printf("Object was released. Wait 3 seconds... \n");
			usleep(3000000); // sleep for 3 secs
		}
		else if(track_status == TRACK_IDLE){
			printf("Tracker in IDLE mode. \n");
			usleep(1000000);
		}
		else if(track_status == TRACK_LOST){
			printf("Lost object. Release the tracker then Try catch another object \n");
			my_payload->setPayloadObjectTrackingMode(TRACK_STOP);
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
		else if(param[0] == PARAM_TRACK_POS_W){
			track_pos_w = param[1];
		}
		else if(param[0] == PARAM_TRACK_POS_H){
			track_pos_h = param[1];
		}
		else if(param[0] == PARAM_TRACK_STATUS){
			track_status = (float)((int)param[1] & 0xff);
		}
		else{
			break;
		}

		printf("%s, status: %.2f, x: %.2f, y: %.2f, w: %.2f, h: %.2f \n", __func__, track_status, track_pos_x, track_pos_y, track_pos_w, track_pos_h);
		break;
	}
	default: break;
	}
}