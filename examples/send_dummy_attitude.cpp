/**
 * This sample will send the attitude of the drone to the payload
 **/

#include "stdio.h"
#include"payloadSdkInterface.h"

PayloadSdkInterface* my_payload = nullptr;

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

int main(int argc, char *argv[]){
	printf("Starting Send Dummy Attitude fro Anti-Drift function example...\n");
	signal(SIGINT,quit_handler);

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// check connection
	my_payload->checkPayloadConnection();

	int msg_cnt = 0;
	int boot_time_ms = 0;


	while(1){
		// send sample attitude data
		mavlink_attitude_t att;
		att.time_boot_ms = boot_time_ms;
		boot_time_ms += 100;	// need to add boot_time_ms of your system here
		att.roll = 0;
		att.pitch = 0;
		att.yaw = 0;
		att.rollspeed = 0;
		att.pitchspeed = 0;
		att.yawspeed = 0;


		my_payload->sendDroneAttitude(att);
		printf("Send attitude data to payload: %d\n", msg_cnt++);

		usleep(50000); // 50ms, 20Hz, at least
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