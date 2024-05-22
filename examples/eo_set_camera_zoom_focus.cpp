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
	while(!time_to_exit){
        // zoom step
        printf("Zoom In 4 times! \n");
        my_payload->setCameraZoom(ZOOM_TYPE_STEP, ZOOM_IN); // zoom in
        usleep(1000000); // sleep 1s
        my_payload->setCameraZoom(ZOOM_TYPE_STEP, ZOOM_IN); // zoom in
        usleep(1000000); // sleep 1s
        my_payload->setCameraZoom(ZOOM_TYPE_STEP, ZOOM_IN); // zoom in
        usleep(1000000); // sleep 1s
        my_payload->setCameraZoom(ZOOM_TYPE_STEP, ZOOM_IN); // zoom in
        usleep(1000000); // sleep 1s
        printf("Zoom Out 2 times! \n");
        my_payload->setCameraZoom(ZOOM_TYPE_STEP, ZOOM_OUT); // zoom out
        usleep(1000000); // sleep 1s
        my_payload->setCameraZoom(ZOOM_TYPE_STEP, ZOOM_OUT); // zoom out
        usleep(1000000); // sleep 1s

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

        // zoom range
        printf("Zoom Range 50%! \n");
        my_payload->setCameraZoom(ZOOM_TYPE_RANGE, 50.0); // zoom 50%
		usleep(3000000); // sleep 3s
        printf("Zoom Range 70%! \n");
        my_payload->setCameraZoom(ZOOM_TYPE_RANGE, 70.0); // zoom 70%
		usleep(3000000); // sleep 3s
        printf("Zoom Range 100%! \n");
        my_payload->setCameraZoom(ZOOM_TYPE_RANGE, 100.0); // zoom 100%
		usleep(3000000); // sleep 3s
        printf("Zoom Range 0%! \n");
        my_payload->setCameraZoom(ZOOM_TYPE_RANGE, 0.0);  // zoom 0%
		usleep(5000000); // sleep 5s

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
