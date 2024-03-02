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