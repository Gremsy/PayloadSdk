/**
 * This sample will show you how to control the FFC in the thermal camera of the payload
 * 1. Set/Get FFC mode: Auto / Manual
 * 2. Trigger FFC, you will hear the "click" from the camera 
 **/

#include "stdio.h"
#include"payloadSdkInterface.h"

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

int ffc_trigger_cnt_max=5;
int ffc_trigger_cnt = 0;

int main(int argc, char *argv[]){
	printf("Starting IR FFC Control example...\n");
	signal(SIGINT,quit_handler);

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// check connection
	my_payload->checkPayloadConnection();

	printf("[IR] Forcing view source to IR...\n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IR, PARAM_TYPE_UINT32);

	// change FFC mode to Auto
	my_payload->setPayloadCameraFFCMode(FFC_MODE_AUTO);
	SDK_LOG("Change FFC to Auto, waiting for 5secs.");
	usleep(5000000);


	// change FFC mode to Manual
	my_payload->setPayloadCameraFFCMode(FFC_MODE_MANUAL);
	SDK_LOG("Change FFC to Manual, waiting for 5secs.");
	usleep(5000000);
	
	// check payload messages
	while(1){
		if(ffc_trigger_cnt < ffc_trigger_cnt_max){
			ffc_trigger_cnt++;
			SDK_LOG("Trigger FFC, %d", ffc_trigger_cnt);
			my_payload->setPayloadCameraFFCTrigg();
			usleep(3000000);
		}
		else{
			break;
		}
	}

	SDK_LOG("Done. Exit...");
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