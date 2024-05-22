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
bool time_to_exit = false;

void quit_handler(int sig);

int main(int argc, char *argv[]){
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	my_payload->checkPayloadConnection();

    usleep(1000000);

	printf("Starting set palete example...\n");
	while(!time_to_exit){
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_1 , PARAM_TYPE_UINT32);
        printf(" --> SET:      F1: WhiteHot         |       G1: WhiteHot\n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_2 , PARAM_TYPE_UINT32);
        printf(" --> SET:      F1: BlackHot         |       G1: Fulgurite\n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_3 , PARAM_TYPE_UINT32);
        printf(" --> SET:      F1: Rainbow         |       G1: IronRed\n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_4 , PARAM_TYPE_UINT32);
        printf(" --> SET:      F1: RainbowHC         |       G1: HotIron\n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_5 , PARAM_TYPE_UINT32);
        printf(" --> SET:      F1: Ironbow         |       G1: Medical\n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_6 , PARAM_TYPE_UINT32);
        printf(" --> SET:      F1: Lava         |       G1: Arctic\n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_7 , PARAM_TYPE_UINT32);
        printf(" --> SET:      F1: Arctic         |       G1: Rainbow1\n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_8 , PARAM_TYPE_UINT32);
        printf(" --> SET:      F1: Globow         |       G1: Rainbow2\n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_9 , PARAM_TYPE_UINT32);
        printf(" --> SET:      F1: Gradedfire         |       G1: Tint\n");
        usleep(2000000); // sleep 2s
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_10 , PARAM_TYPE_UINT32);
        printf(" --> SET:      F1: Hottest         |       G1: BlackHot\n");
        exit(0);
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
