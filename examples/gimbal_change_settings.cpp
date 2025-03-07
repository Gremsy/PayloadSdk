/**
 * This sample will show you how to change the gimbal's param
 * You can refer to the sample load_gimbal_settings.cpp to get all params index and id_string. After that, you can change value for the param correctly.
 * This sample only support for the Vio and OrusL payload for now 
 **/

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

double STIFF_TILT_VALUE = 0;
uint8_t step_num = 0;

void onPayloadParamChanged(int event, char* param_char, double* param);
void quit_handler(int sig);

int main(int argc, char *argv[]){
	printf("Starting SetPayloadGimbalSettings example...\n\n");
	printf("This sample will: \n");
	printf(" 1. Download the current value of param STIFF_TILT \n");
	printf(" 2. Change the value of param STIFF_TILT to 50\n");
	printf(" 3. Download the value of param STIFF_TILT to verify the changed\n");
	printf(" 4. Change the value for STIFF_TILT back to the original\n");
	printf(" 5. Download the value of param STIFF_TILT to verify the changed\n\n");
	signal(SIGINT,quit_handler);
	
	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadParamChanged(onPayloadParamChanged);

	// check connection
	my_payload->checkPayloadConnection();
	usleep(500000);

	while(1){
		switch(step_num){
		case 0:{
			// step 1
			my_payload->getPayloadGimbalSettingByID("STIFF_TILT");
			usleep(1000000);

			step_num= 1;
			break;
		}
		case 1:{
			// step 2
			my_payload->setPayloadGimbalParamByID("STIFF_TILT", 50);
			usleep(1000000);

			step_num= 2;
			break;
		}
		case 2:{
			// step 3
			my_payload->getPayloadGimbalSettingByID("STIFF_TILT");
			usleep(1000000);

			step_num= 3;
			break;
		}
		case 3:{
			// step 4
			my_payload->setPayloadGimbalParamByID("STIFF_TILT", STIFF_TILT_VALUE);
			usleep(1000000);

			step_num= 4;
			break;
		}
		case 4:{
			// step 5
			my_payload->getPayloadGimbalSettingByID("STIFF_TILT");
			usleep(1000000);

			step_num= 5;
			break;
		}
		default: break;
		}

		if(step_num >= 5) break;
	}

	printf("Done. Exit\n");
	usleep(1000000);

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

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

void onPayloadParamChanged(int event, char* param_char, double* param){
	switch(event){
	case PAYLOAD_CAM_PARAMS:{
		// param[0]: param_index
		// param[1]: value
		printf(" --> Payload_param: %s, value: %.2f\n", param_char, param[1]);
		break;
	}
	case PAYLOAD_GB_PARAMS:{
		// param[0]: param_index
		// param[1]: value
		SDK_LOG("--> Gimbal_param: index: %.f, id: %s, value: %.f", param_char, param[0], param[1]);

		if((strcmp(param_char, "STIFF_TILT") == 0) && step_num == 0)
			STIFF_TILT_VALUE = param[1];
		break;
	}
	
	default: break;
	}
}