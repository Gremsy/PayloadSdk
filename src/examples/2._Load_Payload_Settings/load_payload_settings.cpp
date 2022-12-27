#include "stdio.h"
#include <pthread.h>
#include <cstdlib>

#include"payloadSdkInterface.h"

PayloadSdkInterface* my_payload = nullptr;
bool time_to_exit = false;

pthread_t thrd_recv;

bool all_threads_init();
void *payload_recv_handle(void *threadid);
void quit_handler(int sig);

void _handle_msg_param_ext_value(mavlink_message_t* msg);

int main(int argc, char *argv[]){
	printf("Starting ConnectPayload example...\n");
	signal(SIGINT,quit_handler);

	// init thread to check receive message from payload
	all_threads_init();

	// create payloadsdk object
	my_payload = new PayloadSdkInterface();

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// check connection
	while(!time_to_exit){
		mavlink_message_t msg;
		uint8_t msg_cnt = my_payload->getNewMewssage(msg);

		if(msg_cnt && msg.sysid == 1 && msg.compid == MAV_COMP_ID_CAMERA5){
			printf("Payload connected! \n");
			break;
		}
	}

	// request to read all settings of payload
	my_payload->getPayloadCameraSettingList();

	while(!time_to_exit){
		// main loop
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

bool all_threads_init(){

	int rc = pthread_create(&thrd_recv, NULL, &payload_recv_handle, (int *)1);
	if (rc){
		std::cout << "\nError: Khong the tao thread!" << rc << std::endl;
		return false;
	}
	std::cout << "Thread created\n" << std::endl;
}

void *payload_recv_handle(void *threadid)
{
	// check payload messages
	while(!time_to_exit){
		if(my_payload != nullptr){
			mavlink_message_t msg;
			uint8_t msg_cnt = my_payload->getNewMewssage(msg);
			if(msg_cnt){
				// printf("Got %d message in queue \n", msg_cnt);
				// printf("   --> message %d from system_id: %d with component_id: %d \n", msg.msgid, msg.sysid, msg.compid);
				switch(msg.msgid){
				case MAVLINK_MSG_ID_PARAM_EXT_VALUE:{
					_handle_msg_param_ext_value(&msg);
					break;
				}
				default: break;
				}
			}else{

			}

			usleep(10000);
		}
	}
}

void _handle_msg_param_ext_value(mavlink_message_t* msg){
	// printf("%s msg_id %d \n", __func__, msg->msgid);

	mavlink_param_ext_value_t param_ext_value = {0};
	mavlink_msg_param_ext_value_decode(msg, &param_ext_value);

    uint32_t param_uint32;
    memcpy(&param_uint32, param_ext_value.param_value, sizeof(param_uint32));
    

	printf(" --> Param_id: %s, value: %d\n", param_ext_value.param_id, param_uint32);
}