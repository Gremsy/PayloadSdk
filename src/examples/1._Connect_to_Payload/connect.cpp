#include "stdio.h"
#include"payloadSdkInterface.h"

PayloadSdkInterface* my_payload = nullptr;
bool time_to_exit = false;

void quit_handler(int sig);

int main(int argc, char *argv[]){
	printf("Starting ConnectPayload example...\n");
	signal(SIGINT,quit_handler);

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface();

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// check connection
	while(!time_to_exit){
		mavlink_message_t msg;
		uint8_t msg_cnt = my_payload->getNewMewssage(msg);

		if(msg_cnt && msg.sysid == PAYLOAD_SYSTEM_ID && msg.compid == PAYLOAD_COMPONENT_ID){
			printf("Payload connected! \n");
			break;
		}
	}

	// check payload messages
	while(!time_to_exit){
		mavlink_message_t msg;
		uint8_t msg_cnt = my_payload->getNewMewssage(msg);
		if(msg_cnt){
			printf("Got %d message in queue \n", msg_cnt);
			printf("   --> message %d from system_id: %d with component_id: %d \n", msg.msgid, msg.sysid, msg.compid);
		}else{
			printf("No message received. \n");
			// if no message come, sleep for 500ms
			usleep(500000);
		}

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