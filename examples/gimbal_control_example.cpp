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


class gimbalSetModeExample : public PayloadSdkInterface
{
private:
	float currentPitch;
	float currentRoll;
	float currentYaw;
public:
    /**
	 * set gimbal stop and re-center the yaw angle.
     **/
	void setRecenterYaw();

	/**
	 * set gimbal stop and re-center the pitch angle.
	 **/
	void setRecenterPitch();
	
	/**
	 * Update all current angles of the gimbal.
	 * @param currentPitch: Current pitch angle
	 * @param currentRoll: Current roll angle
	 * @param currentYaw: Current yaw angle
	 **/
	void updateAllCurrentAngle(float currentPitch, float currentRoll, float currentYaw);
};

void 
gimbalSetModeExample::
setRecenterYaw()
{
	// Keep gimbal stop, delay in 1.5secs
	setGimbalSpeed(0, 0, 0, INPUT_SPEED);
	usleep(1500000);

	// Re-center the yaw angle
	setGimbalSpeed(this->currentPitch, this->currentRoll, 0, INPUT_ANGLE);

	// Wait until the yaw is nearly dead zone
	while(fabs(this->currentYaw) >= 0.2);
}

void 
gimbalSetModeExample::
setRecenterPitch()
{
	// Keep gimbal stop, delay in 1.5secs
	setGimbalSpeed(0, 0, 0, INPUT_SPEED);
	usleep(1500000);

	// Re-center the pitch angle
	setGimbalSpeed(0, this->currentRoll, this->currentYaw, INPUT_ANGLE);

	// Wait until the pitch is nearly dead zone
	while(fabs(this->currentPitch) >= 0.2);
}

void 
gimbalSetModeExample::
updateAllCurrentAngle(float currentPitch, float currentRoll, float currentYaw)
{
	this->currentPitch = currentPitch;
	this->currentRoll = currentRoll;	
	this->currentYaw = currentYaw;
}

gimbalSetModeExample *my_payload = nullptr;
void quit_handler(int sig);
static void onPayloadStatusChanged(int event, double* param);
static void onPayloadParamChanged(int event, char* param_char, double* param_double);

int main(int argc, char *argv[]){
	printf("Starting Set gimbal mode example...\n");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new gimbalSetModeExample();

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// register callback function for parameter changes
	my_payload->regPayloadParamChanged(onPayloadParamChanged);

	// check connection
	my_payload->checkPayloadConnection();
	
	// set FOLLOW mode
	printf("Gimbal set mode FOLLOW, delay in 3 secs \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, PARAM_TYPE_UINT32);
	usleep(3000000);

	printf("Move gimbal angular velocity: pitch = -20 deg/s, yaw = 50 deg/s, delay in 3 secs \n");
	my_payload->setGimbalSpeed(-20, 0, 50, INPUT_SPEED);
	usleep(3000000);

	printf("Gimbal set re-center YAW ------ \n");
	my_payload->setRecenterYaw();

	printf("Gimbal set re-center PITCH ------ \n");
	my_payload->setRecenterPitch();

	printf("Move gimbal angular position: pitch = -40 deg, yaw = 90 deg, delay in 3 secs \n");
	my_payload->setGimbalSpeed(-40, 0, 90, INPUT_ANGLE);
	usleep(3000000);

	printf("Gimbal set re-center PITCH ------ \n");
	my_payload->setRecenterPitch();

	printf("Gimbal set re-center YAW ------ \n");
	my_payload->setRecenterYaw();

	printf("Delay in 3 secs before changing to LOCK MODE\n");
	usleep(3000000);	

	// set LOCK mode
	printf("Gimbal set mode LOCK, delay in 3 secs \n");	
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_LOCK, PARAM_TYPE_UINT32);
	usleep(3000000);

	printf("Move gimbal angular velocity: pitch = 20 deg/s, yaw = -50 deg/s, delay in 3 secs \n");
	my_payload->setGimbalSpeed(20, 0, -50, INPUT_SPEED);
	usleep(3000000);	

	printf("Gimbal set re-center PITCH ------ \n");
	my_payload->setRecenterPitch();

	printf("Gimbal set re-center YAW ------ \n");
	my_payload->setRecenterYaw();

	printf("Move gimbal angular position: pitch = 40 deg, yaw = -90 deg, delay in 3 secs \n");
	my_payload->setGimbalSpeed(40, 0, -90, INPUT_ANGLE);
	usleep(3000000);

	printf("Gimbal set re-center PITCH ------ \n");
	my_payload->setRecenterPitch();

	printf("Gimbal set re-center YAW ------ \n");
	my_payload->setRecenterYaw();
	usleep(3000000);	

	printf("Delay in 3 secs before changing to FOLLOW MODE\n");
	usleep(3000000);	

	// set FOLLOW mode
	printf("Gimbal set mode FOLLOW, delay in 3 secs \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, PARAM_TYPE_UINT32);
	usleep(3000000);

	printf("Move gimbal angular velocity: pitch = -20 deg/s, yaw = 50 deg/s, delay in 3 secs \n");
	my_payload->setGimbalSpeed(-20, 0, 50, INPUT_SPEED);
	usleep(3000000);

	printf("Gimbal set re-center YAW ------ \n");
	my_payload->setRecenterYaw();

	printf("Gimbal set re-center PITCH ------ \n");
	my_payload->setRecenterPitch();

	printf("Move gimbal angular position: pitch = -40 deg, yaw = 90 deg, delay in 3 secs \n");
	my_payload->setGimbalSpeed(-40, 0, 90, INPUT_ANGLE);
	usleep(3000000);

	printf("Gimbal set re-center PITCH ------ \n");
	my_payload->setRecenterPitch();

	printf("Gimbal set re-center YAW ------ \n");
	my_payload->setRecenterYaw();

	printf("Delay in 3 secs before changing to MAPPING MODE\n");
	usleep(3000000);	

	// set MAPPING mode
	printf("Gimbal set mode MAPPING, delay in 5 secs \n");	
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING, PARAM_TYPE_UINT32);
	usleep(5000000);

	// set RESET mode
	printf("Gimbal set mode RESET, delay in 5 secs \n");	
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_RESET, PARAM_TYPE_UINT32);
	usleep(5000000);	

	// set OFF mode
	printf("Gimbal set mode OFF, delay in 5 secs \n");	
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_OFF, PARAM_TYPE_UINT32);	
	usleep(5000000);	
		
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

static void onPayloadStatusChanged(int event, double* param)
{
	switch(event)
	{
		case PAYLOAD_GB_ATTITUDE:
		{
			float pitch = param[0];
			float roll = param[1];
			float yaw = param[2];
//			printf("Gimbal attitude: pitch = %.2f, roll = %.2f, yaw = %.2f\n", pitch, roll, yaw);
			// Update current angles
			my_payload->updateAllCurrentAngle(pitch, roll, yaw);
			break;
		}
		default: break;
	}
}

void onPayloadParamChanged(int event, char* param_char, double* param_double)
{
	switch(event){
	case PAYLOAD_GB_ATTITUDE:
	{
//		printf("GB MODE: %s\n", param_char);
		break;
	}
	default: break;
	}
}