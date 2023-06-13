#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <string>

#include"payloadSdkInterface.h"

PayloadSdkInterface* my_payload = nullptr;
bool time_to_exit = false;

void quit_handler(int sig);
void _handle_msg_mount_orientation(mavlink_message_t* msg);


int main(int argc, char *argv[]){
	printf("Starting Set gimbal mode example...\n");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
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
		usleep(10000);
	}

	while(!time_to_exit){
		if(my_payload != nullptr){
			mavlink_message_t msg;
			uint8_t msg_cnt = my_payload->getNewMewssage(msg);
			if(msg_cnt){
				// printf("Got %d message in queue \n", msg_cnt);
				// printf("   --> message %d from system_id: %d with component_id: %d \n", msg.msgid, msg.sysid, msg.compid);
				switch(msg.msgid){
				case MAVLINK_MSG_ID_MOUNT_ORIENTATION:{
					_handle_msg_mount_orientation(&msg);
					break;
				}
				default: break;
				}
			}else{

			}

			usleep(10000);
		}
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

void _handle_msg_mount_orientation(mavlink_message_t* msg){
    mavlink_mount_orientation_t packet;
    mavlink_msg_mount_orientation_decode(msg, &packet);
    printf("Pich: %.2f - Roll: %.2f - Yaw: %.2f\n", packet.pitch, packet.roll, packet.yaw);
}