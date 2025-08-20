#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <string>

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
void onPayloadStatusChanged(int event, double* param);


int main(int argc, char *argv[]){
	SDK_LOG("Starting Set gimbal mode example...");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	SDK_LOG("Waiting for payload signal! ");

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// check connection
	my_payload->checkPayloadConnection();

#if 1
	// set message rate for auto-sending the status
	// the reply from the payload will be DEBUG messages

	my_payload->setParamRate(PARAM_EO_ZOOM_LEVEL, 1000);
	my_payload->setParamRate(PARAM_IR_ZOOM_LEVEL, 1000);

    #if defined VIO
		// set the interval for param update
		my_payload->setParamRate(PARAM_LRF_RANGE, 100);
		my_payload->setParamRate(PARAM_LRF_OFSET_X, 100);
		my_payload->setParamRate(PARAM_LRF_OFSET_Y, 100);

		my_payload->setParamRate(PARAM_TARGET_COOR_LON, 1000);
		my_payload->setParamRate(PARAM_TARGET_COOR_LAT, 1000);
		my_payload->setParamRate(PARAM_TARGET_COOR_ALT, 1000);

		my_payload->setParamRate(PARAM_CAM_VIEW_MODE, 1000);
		my_payload->setParamRate(PARAM_CAM_REC_SOURCE, 1000);
		my_payload->setParamRate(PARAM_CAM_IR_TYPE, 1000);
		my_payload->setParamRate(PARAM_CAM_IR_PALETTE_ID, 1000);
		
		my_payload->setParamRate(PARAM_GIMBAL_MODE, 1000);

		my_payload->setParamRate(PARAM_PAYLOAD_GPS_LON, 1000);
		my_payload->setParamRate(PARAM_PAYLOAD_GPS_LAT, 1000);
		my_payload->setParamRate(PARAM_PAYLOAD_GPS_ALT, 1000);

		my_payload->setParamRate(PARAM_CAM_IR_FFC_MODE, 1000);

		my_payload->setParamRate(PARAM_IR_TEMP_MAX, 1000);
	    my_payload->setParamRate(PARAM_IR_TEMP_MIN, 1000);
	    my_payload->setParamRate(PARAM_IR_TEMP_MEAN, 1000);
	#elif defined ZIO
		my_payload->setParamRate(PARAM_GIMBAL_MODE, 1000);

		my_payload->setParamRate(PARAM_PAYLOAD_GPS_LON, 1000);
		my_payload->setParamRate(PARAM_PAYLOAD_GPS_LAT, 1000);
		my_payload->setParamRate(PARAM_PAYLOAD_GPS_ALT, 1000);
    #endif /* VIO */

#else
	// request the param one-by-one
	// the reply from the payload will be PARAM_VALUE 
	for (int i =0; i < PARAM_COUNT; i++){
		my_payload->requestParamValue(i);
		usleep(100000);
	}
#endif

	while(!time_to_exit){

		usleep(10000);
	}

    
	return 0;
}

void quit_handler( int sig ){
    SDK_LOG("");
    SDK_LOG("TERMINATING AT USER REQUEST ");
    SDK_LOG("");

    time_to_exit = true;

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    // end program here
    exit(0);
}

  
void onPayloadStatusChanged(int event, double* param){
	// return;
	switch(event){
	case PAYLOAD_GB_ATTITUDE:{
		// param[0]: pitch
		// param[1]: roll
		// param[2]: yaw

		// SDK_LOG("Pich: %.2f - Roll: %.2f - Yaw: %.2f", param[0], param[1], param[2]);
		break;
	}
	case PAYLOAD_PARAMS:{
		// param[0]: param index
		// param[1]: value
		if(param[0] == PARAM_EO_ZOOM_LEVEL){
			SDK_LOG("Payload EO_ZOOM_LEVEL: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_IR_ZOOM_LEVEL){
			SDK_LOG("Payload IR_ZOOM_LEVEL: %.2f ", param[1]);
		}
    // #if defined VIO
		else if(param[0] == PARAM_LRF_RANGE){
			SDK_LOG("Payload LRF_RANGE: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_LRF_OFSET_X){
			SDK_LOG("Payload PARAM_LRF_OFSET_X: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_LRF_OFSET_Y){
			SDK_LOG("Payload PARAM_LRF_OFSET_Y: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_TARGET_COOR_LON){
			SDK_LOG("Payload PARAM_TARGET_COOR_LON: %.6f ", param[1]);
		}
		else if(param[0] == PARAM_TARGET_COOR_LAT){
			SDK_LOG("Payload PARAM_TARGET_COOR_LAT: %.6f ", param[1]);
		}
		else if(param[0] == PARAM_TARGET_COOR_ALT){
			SDK_LOG("Payload PARAM_TARGET_COOR_ALT: %.6f ", param[1]);
		}

		else if(param[0] == PARAM_PAYLOAD_GPS_LON){
			SDK_LOG("Payload PARAM_PAYLOAD_GPS_LON: %.6f ", param[1]);
		}
		else if(param[0] == PARAM_PAYLOAD_GPS_LAT){
			SDK_LOG("Payload PARAM_PAYLOAD_GPS_LAT: %.6f ", param[1]);
		}
		else if(param[0] == PARAM_PAYLOAD_GPS_ALT){
			SDK_LOG("Payload PARAM_PAYLOAD_GPS_ALT: %.6f ", param[1]);
		}

		else if(param[0] == PARAM_CAM_VIEW_MODE){
			SDK_LOG("Payload PARAM_CAM_VIEW_MODE: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_CAM_REC_SOURCE){
			SDK_LOG("Payload PARAM_CAM_REC_SOURCE: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_CAM_IR_TYPE){
			SDK_LOG("Payload PARAM_CAM_IR_TYPE: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_CAM_IR_PALETTE_ID){
			SDK_LOG("Payload PARAM_CAM_IR_PALETTE_ID: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_CAM_IR_FFC_MODE){
			SDK_LOG("Payload PARAM_CAM_IR_FFC_MODE: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_GIMBAL_MODE){
			SDK_LOG("Payload PARAM_GIMBAL_MODE: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_IR_TEMP_MAX){
			SDK_LOG("Payload PARAM_IR_TEMP_MAX: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_IR_TEMP_MIN){
			SDK_LOG("Payload PARAM_IR_TEMP_MIN: %.2f ", param[1]);
		}
		else if(param[0] == PARAM_IR_TEMP_MEAN){
			SDK_LOG("Payload PARAM_IR_TEMP_MEAN: %.2f ", param[1]);
		}
    // #endif /* VIO */
		break;
	}
	default: break;
	}
}