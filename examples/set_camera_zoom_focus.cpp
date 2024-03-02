#include "stdio.h"
#include <pthread.h>
#include <cstdlib>

#include"payloadSdkInterface.h"

T_ConnInfo s_conn = {
    CONTROL_UART,
    "/dev/ttyUSB0",
    115200
};

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
	while(!time_to_exit){
		// zoom continuous
        printf("Start Zoom In! \n");
        my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_IN); // zoom in
		usleep(5000000); // sleep 5s
        printf("Stop Zoom! \n");
        my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_STOP); // stop zoom
		usleep(2000000); // sleep 2s
        printf("Start Zoom Out! \n");
        my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_OUT); // zoom out
		usleep(7000000); // sleep 7s
        printf("Stop Zoom! \n");
        my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_STOP); // stop zoom
		usleep(2000000); // sleep 2s

        // focus continuous
        printf("Start Focus In! \n");
        my_payload->setCameraFocus(FOCUS_TYPE_CONTINUOUS, FOCUS_IN); // focus in
		usleep(4000000); // sleep 4s
        printf("Stop Focus! \n");
        my_payload->setCameraFocus(FOCUS_TYPE_CONTINUOUS, FOCUS_STOP); // stop focus
		usleep(2000000); // sleep 2s
        printf("Start Focus Out! \n");
        my_payload->setCameraFocus(FOCUS_TYPE_CONTINUOUS, FOCUS_OUT); // focus out
		usleep(4000000); // sleep 4s
        printf("Stop Focus! \n");
        my_payload->setCameraFocus(FOCUS_TYPE_CONTINUOUS, FOCUS_STOP); // stop focus
		usleep(2000000); // sleep 2s

        // auto focus
        printf("Auto Focus! \n");
        my_payload->setCameraFocus(FOCUS_TYPE_AUTO); // auto focus

        printf("!--------------------! \n");
        usleep(7000000); // sleep 7s
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
