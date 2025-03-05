/**
 * The calibration is very sensitivity. Please carefully consider before doing calibration. 
 * PLEASE IGNORE THIS SAMPLE IF YOU ARE NOT FULLY UNDERSTANDING WHAT YOU WILL DO!!!
 * This sample will show you how to send command to do gimbal calibration
 * 1. Calib Gyro
 * 2. Calib Accel
 * 3. Auto tune
 * 4. Calib motor
 * 5. Search home 
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

enum calib_type_t{
	CALIB_GYRO=0,
	CALIB_ACCEL,
	AUTO_TUNE,
	CALIB_MOTOR,
	SEARCH_HOME,
};

calib_type_t myCalib = CALIB_GYRO;
bool is_calibration_runing = false;
bool is_exit = false;

void onPayloadStatusChanged(int event, double* param);
void onPayloadParamChanged(int event, char* param_char, double* param);

int main(int argc, char *argv[]){
	printf("Starting Do Calib gimbal example...\n");
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


	// set the calib type
	myCalib = CALIB_GYRO;

	switch(myCalib){
	case CALIB_GYRO:{
		is_calibration_runing = false;
		my_payload->sendPayloadGimbalCalibGyro();
		SDK_LOG("Calib gyro command was sent. Waiting for the calibration done...");
		break;
	}
	case CALIB_ACCEL:{
		is_calibration_runing = false;
		my_payload->sendPayloadGimbalCalibAccel();
		SDK_LOG("Calib accel command was sent. Waiting for the calibration done...");
		break;
	}
	case CALIB_MOTOR:{
		is_calibration_runing = false;
		my_payload->sendPayloadGimbalCalibMotor();
		SDK_LOG("Calib motor command was sent. Waiting for the calibration done...");
		break;
	}
	case AUTO_TUNE:{
		is_calibration_runing = false;
		my_payload->sendPayloadGimbalAutoTune(true);
		SDK_LOG("Auto tune command was sent. Waiting for the process done...");
		break;
	}
	case SEARCH_HOME:{
		is_calibration_runing = false;
		my_payload->sendPayloadGimbalSearchHome();
		SDK_LOG("Searching Home command was sent. Waiting for the calibration done...");
		break;
	}
	default: break;
	}

	// waiting the calib process done
	while(!is_exit){
		// do nothing
		usleep(1000000);
	}

	// load params to verify
	switch(myCalib){
	case CALIB_GYRO:{
		SDK_LOG("Load the Gyro offset values...");
		my_payload->getPayloadGimbalSettingByID("GYROX_OFFSET");
		my_payload->getPayloadGimbalSettingByID("GYROY_OFFSET");
		my_payload->getPayloadGimbalSettingByID("GYROZ_OFFSET");
		break;
	}
	case CALIB_ACCEL:{
		SDK_LOG("Load the accel offset values...");
		my_payload->getPayloadGimbalSettingByID("ACCELX_OFFSET");
		my_payload->getPayloadGimbalSettingByID("ACCELY_OFFSET");
		my_payload->getPayloadGimbalSettingByID("ACCELZ_OFFSET");
		break;
	}
	case CALIB_MOTOR:{
		
		break;
	}
	case AUTO_TUNE:{
		SDK_LOG("Waiting for the gimbal rebooted...20s");
		usleep(20000000); // waiting for the gimbal reboot
		SDK_LOG("Load the Stiffness/Holdstrength values...");
		my_payload->getPayloadGimbalSettingByID("STIFF_TILT");
		my_payload->getPayloadGimbalSettingByID("STIFF_ROLL");
		my_payload->getPayloadGimbalSettingByID("STIFF_PAN");
		my_payload->getPayloadGimbalSettingByID("PWR_TILT");
		my_payload->getPayloadGimbalSettingByID("PWR_ROLL");
		my_payload->getPayloadGimbalSettingByID("PWR_PAN");
		break;
	}
	case SEARCH_HOME:{
		break;
	}
	default: break;
	}

	usleep(1000000);
	SDK_LOG("Exit.");

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
	// return;
	switch(event){
	case PAYLOAD_ACK:{

		// SDK_LOG("Got ack from %.f, result %.f, progress: %.f", param[0], param[1], param[2]);

		switch(myCalib){
		case CALIB_GYRO:{
			if(param[0] == MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION){
				if(param[1] == MAV_RESULT_ACCEPTED){
					is_calibration_runing = true;
				}

				if(param[2] == MAV_RESULT_ACCEPTED){
					if(is_calibration_runing){
						SDK_LOG("The gyro calibration done!");
						is_exit= true;
					}
				}
				else if(param[2] == MAV_RESULT_IN_PROGRESS){
					is_calibration_runing = true;
					SDK_LOG("The gyro calibration is processing...");
				}
			}
			break;
		}
		case CALIB_ACCEL:{
			if(param[0] == MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION){
				if(param[1] == MAV_RESULT_ACCEPTED){
					is_calibration_runing = true;
				}

				if(param[2] == MAV_RESULT_ACCEPTED){
					if(is_calibration_runing){
						SDK_LOG("The accel calibration done!");
						is_exit= true;
					}
				}
				else if(param[2] == MAV_RESULT_IN_PROGRESS){
					is_calibration_runing = true;
					SDK_LOG("The accel calibration is processing...");
				}
			}
			break;
		}
		case CALIB_MOTOR:{
			if(param[0] == MAV_CMD_DO_SET_HOME){
				if(param[1] == MAV_RESULT_ACCEPTED){
					is_calibration_runing = true;
					usleep(1000000); // waiting for the calib init
				}

				if(param[2] == MAV_RESULT_ACCEPTED){
					if(is_calibration_runing){
						SDK_LOG("The motor calibration done!");

						is_exit= true;
					}
				}
				else if(param[2] == MAV_RESULT_IN_PROGRESS){
					is_calibration_runing = true;
					SDK_LOG("The motor calibration is processing...");
				}
			}
			break;
		}
		case AUTO_TUNE:{
			if(param[0] == MAV_CMD_USER_3){
				if(param[1] == MAV_RESULT_ACCEPTED){
					is_calibration_runing = true;
					usleep(1000000); // waiting for the calib init
				}

				if(param[2] == MAV_RESULT_ACCEPTED){
					if(is_calibration_runing){
						SDK_LOG("The Auto tune done!");
						usleep(1000000);

						is_exit= true;
					}
				}
				else if(param[2] == MAV_RESULT_IN_PROGRESS){
					is_calibration_runing = true;
					SDK_LOG("The Auto tune is processing...");
				}
			}
			break;
		}
		case SEARCH_HOME:{
			if(param[0] == MAV_CMD_DO_SET_HOME){
				if(param[1] == MAV_RESULT_ACCEPTED){
					is_calibration_runing = true;
					usleep(1000000); // waiting for the calib init
				}

				if(param[2] == MAV_RESULT_ACCEPTED){
					if(is_calibration_runing){
						SDK_LOG("The SearchHome done!");

						is_exit= true;
					}
				}
				else if(param[2] == MAV_RESULT_IN_PROGRESS){
					is_calibration_runing = true;
					SDK_LOG("The SearchHome is processing...");
				}
			}
			break;
		}
		default: break;
		}
		break;
	}
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