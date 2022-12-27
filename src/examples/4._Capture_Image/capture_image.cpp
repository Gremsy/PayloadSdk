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

typedef enum{
	idle = 0,
	check_storage,
	check_capture_status,
	check_camera_mode,
	change_camera_mode,
	do_capture,
	wait_capture_done,
}capture_sequence_t;

capture_sequence_t my_capture = idle;
uint8_t image_to_capture = 3;

void _handle_msg_param_ext_value(mavlink_message_t* msg);
void _handle_msg_command_ack(mavlink_message_t* msg);
void _handle_msg_storage_information(mavlink_message_t* msg);
void _handle_msg_camera_capture_status(mavlink_message_t* msg);
void _handle_msg_camera_settings(mavlink_message_t* msg);

int main(int argc, char *argv[]){
	printf("Starting CaptureImage example...\n");
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

	// set payload to video mode for testing
	my_payload->setPayloadCameraMode(CAMERA_MODE_VIDEO);

	my_capture = check_storage;
	while(!time_to_exit){
		// to caputre image with payload, follow this sequence
		switch(my_capture){
		case idle:{
			// do nothing;
			break;
		}
		case check_storage:{
			my_payload->getPayloadStorage();
			break;
		}
		case check_capture_status:{
			my_payload->getPayloadCaptureStatus();
			break;
		}
		case check_camera_mode:{
			my_payload->getPayloadCameraMode();
			break;
		}
		case change_camera_mode:{
			my_payload->setPayloadCameraMode(CAMERA_MODE_IMAGE);
			my_capture = check_camera_mode;
			break;
		}
		case do_capture:{
			my_payload->setPayloadCameraCaptureImage();
			my_capture = wait_capture_done;
			break;
		}
		case wait_capture_done:{
			my_payload->getPayloadCaptureStatus();
			break;
		}
		default: break;
		}

		usleep(300000); // sleep 0.3s
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
		std::cout << "\nError: Can not create thread!" << rc << std::endl;
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
				case MAVLINK_MSG_ID_COMMAND_ACK:{
					_handle_msg_command_ack(&msg);
					break;
				} 
				case MAVLINK_MSG_ID_STORAGE_INFORMATION:{
					_handle_msg_storage_information(&msg);
					break;
				} 
				case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:{
					_handle_msg_camera_capture_status(&msg);
					break;
				}
				case MAVLINK_MSG_ID_CAMERA_SETTINGS:{
					_handle_msg_camera_settings(&msg);
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

void _handle_msg_command_ack(mavlink_message_t* msg){
	mavlink_command_ack_t cmd_ack = {0};

	mavlink_msg_command_ack_decode(msg, &cmd_ack);

	// printf("Got ACK for command %d with status %d\n", cmd_ack.command, cmd_ack.progress);
}

void _handle_msg_storage_information(mavlink_message_t* msg){
	mavlink_storage_information_t storage_info = {0};

	mavlink_msg_storage_information_decode(msg, &storage_info);

	if(my_capture == check_storage){
		printf("Got payload storage info: total: %.2f MB, used: %.2f MB, available: %.2f MB \n", 
			storage_info.total_capacity, storage_info.used_capacity, storage_info.available_capacity);

		// if payload have enough space, check capture status
		if(storage_info.available_capacity >= 10.0){
			my_capture = check_capture_status;
			printf("   ---> Storage ready, check capture status \n");
		}else{
			printf("   ---> Payload's storage is not ready \n");
			my_capture = idle;
		}
	}
}

void _handle_msg_camera_capture_status(mavlink_message_t* msg){
	mavlink_camera_capture_status_t capture_status = {0};

	mavlink_msg_camera_capture_status_decode(msg, &capture_status);

	if(my_capture == check_capture_status){
		printf("Got payload capture status: image_status: %d, video_status: %d \n", capture_status.image_status, capture_status.video_status);

		// if image status is idle, do capture
		if(capture_status.image_status == 0 ){
			my_capture = check_camera_mode;
			printf("   ---> Payload is idle, Check camera mode \n");
		}else{
			printf("   ---> Payload is busy \n");
			my_capture = idle;
		}
	}else if(my_capture == wait_capture_done){
		if(capture_status.image_status == 0 ){
			my_capture = check_storage;
			printf("   ---> Payload is completed capture image, Do next sequence %d\n", --image_to_capture);
			if(image_to_capture == 0)
				my_capture = idle;
		}else{
			printf("   ---> Payload is busy \n");
		}
	}
}

void _handle_msg_camera_settings(mavlink_message_t* msg){
	mavlink_camera_settings_t camera_setting = {0};

	mavlink_msg_camera_settings_decode(msg, &camera_setting);

	if(my_capture == check_camera_mode){
		printf("Got camera mode: %d \n", camera_setting.mode_id);

		if(camera_setting.mode_id == CAMERA_MODE_IMAGE){
			my_capture = do_capture;
			printf("   ---> Payload in Image mode, do capture image \n");
		}else{
			my_capture = change_camera_mode;
			printf("   ---> Payload in Video mode, change camera mode \n");
		}
	}
}