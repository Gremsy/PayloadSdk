#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <string>
using namespace std;

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
bool is_exit = false;

void quit_handler(int sig);
void onPayloadParamChanged(int event, char* param_char, double* param_double);

int main(int argc, char *argv[]){
	printf("Starting Set gimbal mode example...\n");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// register callback function
	my_payload->regPayloadParamChanged(onPayloadParamChanged);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	my_payload->checkPayloadConnection();
	
#if 0
	/** From the software v307.54
		- We will no longer user the command_long for setting the gimbal's mode
		- We will use the flag in the message MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE (284) for the gimbal mode directly
		- The camera definition file will be changed to remove all option for gimbal's mode
	**/

	printf("Gimbal set mode LOCK, delay in 5 secs \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_LOCK, PARAM_TYPE_UINT32);
	usleep(5000000);

	printf("Gimbal set mode FOLLOW, delay in 5 secs \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, PARAM_TYPE_UINT32);
	usleep(5000000);

	printf("Gimbal set mode MAPPING, delay in 5 secs \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING, PARAM_TYPE_UINT32);
	usleep(5000000);

	printf("Gimbal set mode OFF, delay in 5 secs \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_OFF, PARAM_TYPE_UINT32);
	usleep(5000000);

	printf("Gimbal set mode RESET, delay in 5 secs \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_RESET, PARAM_TYPE_UINT32);
	usleep(5000000);
#else
	uint16_t _flags = 0;
	
	// Need to get the gimbal status flags
	printf("Check the current gimbal status flags \n");
	_flags = my_payload->getGimbalDeviceStatusFlags();

	while(!is_exit){
		printf("Change gimbal mode to LOCK \n");
		_flags |= GIMBAL_DEVICE_FLAGS_YAW_LOCK;
		_flags &= ~(1 << 14);					// xoa co
		_flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL;	// xoa co
		_flags &= ~GIMBAL_DEVICE_FLAGS_RETRACT; // xoa co
		my_payload->setGimbalMode(_flags);
		usleep(3000000);

		printf("Change gimbal mode to FOLLOW \n");
		_flags &= ~GIMBAL_DEVICE_FLAGS_YAW_LOCK;
		_flags &= ~(1 << 14);					// xoa co
		_flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL;	// xoa co
		_flags &= ~GIMBAL_DEVICE_FLAGS_RETRACT; // xoa co
		my_payload->setGimbalMode(_flags);
		usleep(3000000);

		printf("Change gimbal power to OFF \n");
		_flags |= GIMBAL_DEVICE_FLAGS_RETRACT;
		_flags &= ~(1 << 14);					// xoa co
		_flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL;	// xoa co
		my_payload->setGimbalMode(_flags);
		usleep(3000000);

		printf("Change gimbal power to ON \n");
		_flags &= ~GIMBAL_DEVICE_FLAGS_RETRACT;
		_flags &= ~(1 << 14);					// xoa co
		_flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL;	// xoa co
		my_payload->setGimbalMode(_flags);
		usleep(8000000);

		printf("Move gimbal to 20 degrees yaw \n");
		my_payload->setGimbalSpeed(0, 0 , 20, INPUT_ANGLE);
		usleep(3000000);

		printf("Set gimbal return HOME \n");
		_flags |= GIMBAL_DEVICE_FLAGS_NEUTRAL;
		_flags &= ~GIMBAL_DEVICE_FLAGS_RETRACT; // xoa co
		_flags &= ~(1 << 14);					// xoa co mapping
		my_payload->setGimbalMode(_flags);
		usleep(100000);
		// reset the control bit
		_flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL;	// xoa co
		my_payload->setGimbalMode(_flags);
		usleep(3000000);

		printf("Set gimbal to Mapping mode \n");
		_flags |= (1 << 14);
		_flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL;	// xoa co
		_flags &= ~GIMBAL_DEVICE_FLAGS_RETRACT; // xoa co
		my_payload->setGimbalMode(_flags);
		usleep(3000000);

		printf("Set gimbal return HOME \n");
		_flags |= GIMBAL_DEVICE_FLAGS_NEUTRAL;
		_flags &= ~GIMBAL_DEVICE_FLAGS_RETRACT; // xoa co
		_flags &= ~(1 << 14);					// xoa co mapping
		my_payload->setGimbalMode(_flags);
		usleep(100000);
		// reset the HOME bit
		_flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL;	// xoa co
		my_payload->setGimbalMode(_flags);
		usleep(3000000);
	}

#endif

	while(true){
		// main loop
		usleep(10000000);
	}

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

    is_exit = true;
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
	case PAYLOAD_GB_ATTITUDE:{
		// param[0]: param_index
		// param[1]: value
		printf(" --> Gimbal Mode: %s\n", param_char);
		break;
	}
	default: break;
	}
}