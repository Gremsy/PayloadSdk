// This sample will show you how to create a profile for the isotherm on the IR camera
// This sample only use for Vio F1
// This sample will show you how to enable the isotherm region 0 for human detect (the detected temperature will around 35 - 40 Celsius degrees)
// This sample only support the software version v3.0.3 or higher

#include "stdio.h"
#include"payloadSdkInterface.h"

// define params ID and values
// The ID and value can be found on the camera definition file version 12 or higher

#define ID_IR_GAIN		 			"C_T_G"
#define IR_GAIN_HIGH		 		0 	// -50℃~150℃ 
#define IR_GAIN_LOW		 			1   // -50℃~550℃ 

#define ID_ISOTHERM_UNIT 			"C_T_ISO_U"
#define UNIT_KELVIN		 			0
#define UNIT_CELSIUS 				1

#define ID_ISOTHERM_MODE 			"C_T_ISO_M"
#define VALUE_ISOTHERM_MODE_ENABLE 			1
#define VALUE_ISOTHERM_MODE_DISABLE 		0

#define ID_ISOTHERM_RG0_MODE 		"C_T_RG0_CLR_M"
#define ID_ISOTHERM_RG0_TEMP 		"C_T_RG0_TMP"
#define ID_ISOTHERM_RG0_COLOR_LOW 	"C_T_RG0_CLR_MN"
#define ID_ISOTHERM_RG0_COLOR_HIGH 	"C_T_RG0_CLR_MX"

#define ID_ISOTHERM_RG1_MODE 		"C_T_RG1_CLR_M"
#define ID_ISOTHERM_RG1_TEMP 		"C_T_RG1_TMP"
#define ID_ISOTHERM_RG1_COLOR_LOW 	"C_T_RG1_CLR_MN"
#define ID_ISOTHERM_RG1_COLOR_HIGH 	"C_T_RG1_CLR_MX"

#define ID_ISOTHERM_RG2_MODE 		"C_T_RG2_CLR_M"
#define ID_ISOTHERM_RG2_TEMP 		"C_T_RG2_TMP"
#define ID_ISOTHERM_RG2_COLOR_LOW 	"C_T_RG2_CLR_MN"
#define ID_ISOTHERM_RG2_COLOR_HIGH 	"C_T_RG2_CLR_MX"

#define ID_ISOTHERM_RG3_MODE 		"C_T_RG3_CLR_M"
#define ID_ISOTHERM_RG3_TEMP 		"C_T_RG3_TMP"
#define ID_ISOTHERM_RG3_COLOR_LOW 	"C_T_RG3_CLR_MN"
#define ID_ISOTHERM_RG3_COLOR_HIGH 	"C_T_RG3_CLR_MX"

#define ID_ISOTHERM_RG4_MODE 		"C_T_RG4_CLR_M"
#define ID_ISOTHERM_RG4_TEMP 		"C_T_RG4_TMP"
#define ID_ISOTHERM_RG4_COLOR_LOW 	"C_T_RG4_CLR_MN"
#define ID_ISOTHERM_RG4_COLOR_HIGH 	"C_T_RG4_CLR_MX"

#define ID_ISOTHERM_RG5_MODE 		"C_T_RG5_CLR_M"
#define ID_ISOTHERM_RG5_COLOR_LOW 	"C_T_RG5_CLR_MN"
#define ID_ISOTHERM_RG5_COLOR_HIGH 	"C_T_RG5_CLR_MX"

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

void onPayloadStatusChanged(int event, double* param);
void quit_handler(int sig);

void enable_human_detect_profile();
void enable_fires_detect_profile();

int main(int argc, char *argv[]){
	printf("Starting IR Isotherm Profile example...\n");
	signal(SIGINT,quit_handler);

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// check connection
	my_payload->checkPayloadConnection();

	// Subscribe to status updates so we can read acknowledgements
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// enable the profile
	enable_human_detect_profile();
	usleep(10000000);

	// check payload messages
	while(1){
		// we will change the profile between "Human" and "Fires" every 10 seconds

		printf("Change profile to Human Detect \n");
		enable_fires_detect_profile();
		usleep(10000000); // sleep for 10 seconds

		printf("Change profile to Fires Detect \n");
		enable_human_detect_profile();
		usleep(10000000); // sleep for 10 seconds
	}

	return 0;
}

void onPayloadStatusChanged(int event, double* param){
    
    switch(event){
	case PAYLOAD_PARAM_EXT_ACK:{
		printf("[Status] Extended ACK received. The result is: %.2f\n", param[0]);
		break;
	}
    default: break;
    }
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

void enable_human_detect_profile(){
	// ensure the instance is not null
	if(my_payload == nullptr) return;

	// enable the isotherm
	printf("Enable the isotherm \n"); 
	my_payload->setPayloadCameraParam(ID_ISOTHERM_MODE, VALUE_ISOTHERM_MODE_ENABLE, PARAM_TYPE_UINT32);

	// disable all isotherm region to clear the previous settings
	printf("Disable all isotherm region to clear the previous settings \n");
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG0_MODE, 0, PARAM_TYPE_UINT32);
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG1_MODE, 0, PARAM_TYPE_UINT32);
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG2_MODE, 0, PARAM_TYPE_UINT32);
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG3_MODE, 0, PARAM_TYPE_UINT32);
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG4_MODE, 0, PARAM_TYPE_UINT32);
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG5_MODE, 0, PARAM_TYPE_UINT32);

	printf("Set the IR gain to High \n");
	my_payload->setPayloadCameraParam(ID_IR_GAIN, IR_GAIN_HIGH, PARAM_TYPE_UINT32);

	printf("Set the Unit to Celsius \n");
	my_payload->setPayloadCameraParam(ID_ISOTHERM_UNIT, UNIT_CELSIUS, PARAM_TYPE_UINT32);

	// Config for region 1
	// Because the region 0 will default the temperature from minimum (Kelvin), we can not change this temp, so we will use the region 1 for this profile
	// The profile will be:
	// - Temperature 35 - 40 in Celsius degrees
	// - Color from Black to Red, in Linear RGB

	printf("Apply the settings for region 1 \n");

	// set the low temp to 35 Celsius
	// the low temp for region 1 is the high temp from region 0.
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG0_TEMP, 35, PARAM_TYPE_UINT32);

	// set the high temp to 40 Celsius
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG1_TEMP, 40, PARAM_TYPE_UINT32);

	// set the color mode fro region 1. Linear RGB is 2
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG1_MODE, 2, PARAM_TYPE_UINT32);

	// set the low color. The index of Black is 0
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG1_COLOR_LOW, 0, PARAM_TYPE_UINT32);

	// set the high color. The index of Red is 9
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG1_COLOR_HIGH, 9, PARAM_TYPE_UINT32);

	printf("Now the region 1 was configured for detecting the human's body temperature or any object has the temperature from 35 to 40 in Celsius degrees. \n");
	printf("Done. Exit \n");
}

void enable_fires_detect_profile(){
	// ensure the instance is not null
	if(my_payload == nullptr) return;

	// We will assume that the fires have the temperature range from 100 to 140 in Celsius
	// We still can use the "High" gain for this case because of the "highest" temperature is 140
	// If the highest temperature is over 150, we need to change to "Low" gain

	// enable the isotherm
	printf("Enable the isotherm \n"); 
	my_payload->setPayloadCameraParam(ID_ISOTHERM_MODE, VALUE_ISOTHERM_MODE_ENABLE, PARAM_TYPE_UINT32);

	// disable all isotherm region to clear the previous settings
	printf("Disable all isotherm region to clear the previous settings \n");
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG0_MODE, 0, PARAM_TYPE_UINT32);
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG1_MODE, 0, PARAM_TYPE_UINT32);
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG2_MODE, 0, PARAM_TYPE_UINT32);
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG3_MODE, 0, PARAM_TYPE_UINT32);
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG4_MODE, 0, PARAM_TYPE_UINT32);
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG5_MODE, 0, PARAM_TYPE_UINT32);

	printf("Set the IR gain to High \n");
	my_payload->setPayloadCameraParam(ID_IR_GAIN, IR_GAIN_HIGH, PARAM_TYPE_UINT32);

	printf("Set the Unit to Celsius \n");
	my_payload->setPayloadCameraParam(ID_ISOTHERM_UNIT, UNIT_CELSIUS, PARAM_TYPE_UINT32);

	// Config for region 1
	// Because the region 0 will default the temperature from minimum (Kelvin), we can not change this temp, so we will use the region 1 for this profile
	// The profile will be:
	// - Temperature 35 - 40 in Celsius degrees
	// - Color from Black to Red, in Linear RGB

	printf("Apply the settings for region 1 \n");

	// set the low temp to 100 Celsius
	// the low temp for region 1 is the high temp from region 0.
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG0_TEMP, 100, PARAM_TYPE_UINT32);

	// set the high temp to 140 Celsius
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG1_TEMP, 140, PARAM_TYPE_UINT32);

	// set the color mode fro region 1. Linear RGB is 2
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG1_MODE, 2, PARAM_TYPE_UINT32);

	// set the low color. The index of Black is 0
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG1_COLOR_LOW, 0, PARAM_TYPE_UINT32);

	// set the high color. The index of Magenta is 9
	my_payload->setPayloadCameraParam(ID_ISOTHERM_RG1_COLOR_HIGH, 10, PARAM_TYPE_UINT32);

	printf("Now the region 1 was configured for detecting the fires or any object has the temperature from 100 to 140 in Celsius degrees. \n");
	printf("Done. Exit \n");
}