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

void quit_handler(int sig);

void onPayloadStatusChanged(int event, double* param);

typedef enum{
	idle = 0,
	check_storage,
	check_capture_status,
	check_camera_mode,
	change_camera_mode,
	do_record_video,
	video_in_recording,
	stop_record_video,
	wait_record_done,
}capture_sequence_t;

capture_sequence_t my_capture = idle;
uint8_t time_to_record = 10;


int main(int argc, char *argv[]){
	printf("Starting RecordVideo example...\n");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// check connection
	my_payload->checkPayloadConnection();
	
	// set payload to video mode for testing
	my_payload->setPayloadCameraMode(CAMERA_MODE_VIDEO);

	// set record source
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RECORD_SRC, PAYLOAD_CAMERA_RECORD_EO, PARAM_TYPE_UINT32);

	my_capture = check_storage;
	while(1){
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
			my_payload->setPayloadCameraMode(CAMERA_MODE_VIDEO);
			my_capture = check_camera_mode;
			break;
		}
		case do_record_video:{
			my_payload->setPayloadCameraRecordVideoStart();
			time_to_record = 10;
			printf("Payload is recording video in %ds, wait... \n", time_to_record);
			
			my_capture = video_in_recording;
			break;
		}
		case video_in_recording:{
			usleep(700000); // 0.7s
			time_to_record--;
			printf("%d \n", time_to_record);
			if(time_to_record == 0){
				my_capture = stop_record_video;
			}
			break;
		}
		case stop_record_video:{
			my_payload->setPayloadCameraRecordVideoStop();
			my_capture = wait_record_done;
			break;
		}
		case wait_record_done:{
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


void onPayloadStatusChanged(int event, double* param){

	switch(event){
	case PAYLOAD_CAM_CAPTURE_STATUS:{
		// param[0]: image_status
		// param[1]: video_status
		// param[2]: image_count
		// param[3]: recording_time_ms

		if(my_capture == check_capture_status){
			printf("Got payload capture status: image_status: %.2f, video_status: %.2f \n", param[0], param[1]);

			// if video status is idle, do capture
			if(param[1] == 0 ){
				my_capture = check_camera_mode;
				printf("   ---> Payload is idle, Check camera mode \n");
			}else{
				printf("   ---> Payload is busy \n");
				my_capture = idle;
			}
		}else if(my_capture == wait_record_done){

			if(param[1] == 0 ){
				printf("   ---> Payload is completed record video\n");

				// close payload interface
				try {
					my_payload->sdkQuit();
				}
				catch (int error){}
				
				exit(0);
			}else{
				printf("   ---> Payload is busy. Wait... \n");
			}
		}
		break;
	}
	case PAYLOAD_CAM_STORAGE_INFO:{
		// param[0]: total_capacity
		// param[1]: used_capacity
		// param[2]: available_capacity
		// param[3]: status

		if(my_capture == check_storage){
			printf("Got payload storage info: total: %.2f MB, used: %.2f MB, available: %.2f MB \n", 
				param[0], param[1], param[2]);

			// if payload have enough space, check capture status
			if(param[2] >= 10.0){
				my_capture = check_capture_status;
				printf("   ---> Storage ready, check capture status \n");
			}else{
				printf("   ---> Payload's storage is not ready \n");
				my_capture = idle;
			}
		}
		break;
	}
	case PAYLOAD_CAM_SETTINGS:{
		// param[0]: mode_id
		// param[1]: zoomLevel
		// param[2]: focusLevel
		if(my_capture == check_camera_mode){
			printf("Got camera mode: %.2f \n", param[0]);

			if(param[0] == CAMERA_MODE_VIDEO){
				my_capture = do_record_video;
				printf("   ---> Payload in Image mode, do capture image \n");
			}else{
				my_capture = change_camera_mode;
				printf("   ---> Payload in Video mode, change camera mode \n");
			}
		}
		break;
	}
	default: break;
	}
}