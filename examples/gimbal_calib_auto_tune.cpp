/**
 * This sample will show how to perfrom the gimbal auto tune process
 * Only support gimbal's firmware 790.6 or higher
 * The sequence is:
 * 1. Seding to auto tune command to the gimbal, uses command_long (MAV_CMD_USER_3)
 * 2. Checking the ACK feedback from the gimbal for the process's status
 **/

#include "stdio.h"
#include"payloadSdkInterface.h"

#include <iostream>
#include <chrono>

PayloadSdkInterface* my_payload = nullptr;
bool is_exit = false;
bool is_calibration_runing = false;

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
void onPayloadStatusChanged(int event, double* param);
void onPayloadParamChanged(int event, char* param_char, double* param);

int main(int argc, char *argv[]){
	printf("Starting Gimbal Auto tune example...\n");
	signal(SIGINT,quit_handler);

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);
	my_payload->regPayloadParamChanged(onPayloadParamChanged);

	// check connection
	my_payload->checkPayloadConnection();

	// Sendding the auto tune command
	is_calibration_runing = false;
	my_payload->sendPayloadGimbalAutoTune(true);

	// waiting the calib process done
	while(!is_exit){
		// do nothing
		usleep(1000000);
	}

	// Loading the params to verify
	SDK_LOG("Waiting for the gimbal rebooted...20s");
	usleep(20000000); // waiting for the gimbal reboot
	SDK_LOG("Load the Stiffness/Holdstrength values...");
	my_payload->getPayloadGimbalSettingByID("STIFF_TILT");
	my_payload->getPayloadGimbalSettingByID("STIFF_ROLL");
	my_payload->getPayloadGimbalSettingByID("STIFF_PAN");
	my_payload->getPayloadGimbalSettingByID("PWR_TILT");
	my_payload->getPayloadGimbalSettingByID("PWR_ROLL");
	my_payload->getPayloadGimbalSettingByID("PWR_PAN");
	usleep(5000000);

	SDK_LOG("Exit the example!");

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
	case PAYLOAD_ACK:{
		if(param[0] == MAV_CMD_USER_3){
			// param[0]: command
			// param[1]: result
			// param[2]: progress
			SDK_LOG("Got PAYLOAD_ACK for command %.2f with status %.2f, progress: %.2f", param[0], param[1], param[2]);

			// if(param[1] == MAV_RESULT_ACCEPTED){
			// 	is_calibration_runing = true;
			// 	usleep(1000000); // waiting for the calib init
			// }

			if(param[1] == MAV_RESULT_ACCEPTED){
				if(is_calibration_runing){
					SDK_LOG("The Auto tune done!");
					usleep(1000000);

					is_exit= true;
				}
				else{
					is_calibration_runing = true;
					SDK_LOG("The Auto tune is starting!");
				}
			}
			else if(param[1] == MAV_RESULT_IN_PROGRESS){
				is_calibration_runing = true;
				SDK_LOG("The Auto tune is processing...at %.2f%", param[2]);
			}
			else if(param[1] == MAV_RESULT_FAILED){
				is_calibration_runing = false;
				SDK_LOG("The Auto tune is FAILED");

				is_exit= true;
			}
		}
		break;
	}
	default: break;
	}
}

void onPayloadParamChanged(int event, char* param_char, double* param){
	switch(event){
	case PAYLOAD_CAM_PARAMS:{
		// param[0]: param_index
		// param[1]: value
		SDK_LOG(" --> Payload_param: %s, value: %.2f", param_char, param[1]);
		break;
	}
	case PAYLOAD_GB_PARAMS:{
		// param[0]: param_index
		// param[1]: value
		SDK_LOG("--> Gimbal_param: index: %.f, id: %s, value: %.f", param_char, param[0], param[1]);
		break;
	}
	
	default: break;
	}
}