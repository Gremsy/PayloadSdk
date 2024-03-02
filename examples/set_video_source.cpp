#include "stdio.h"
#include"payloadSdkInterface.h"

PayloadSdkInterface* my_payload = nullptr;
bool time_to_exit = false;

T_ConnInfo s_conn = {
	CONTROL_UART,
	"/dev/ttyACM0",
	115200
};

void quit_handler(int sig);

int main(int argc, char *argv[]){
	printf("Starting ConnectPayload example...\n");
	signal(SIGINT,quit_handler);

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// check connection
	my_payload->checkPayloadConnection();

	// check payload messages
	while(1){
		// change video source to EO only
		printf("change video source to EO in 3 seconds \n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
		// delay 3 seconds
		usleep(3000000);

		// change video source to EOir
		printf("change video source to EOir in 3 seconds \n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EOIR, PARAM_TYPE_UINT32);
		// delay 3 seconds
		usleep(3000000);

		// change video source to IR only
		printf("change video source to IR in 3 seconds \n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IR, PARAM_TYPE_UINT32);
		// delay 3 seconds
		usleep(3000000);

		// change video source to IReo
		printf("change video source to IReo in 3 seconds \n");
		my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IREO, PARAM_TYPE_UINT32);
		// delay 3 seconds
		usleep(3000000);
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