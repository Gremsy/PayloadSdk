/**
 * This sample will send the GPS position to the payload
 * The GPS information will include:
 * 1. Lattitude
 * 2. Longtitude
 * 3. Attitude
 * 4. Heading
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
	printf("Starting SendGPS example...\n");
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
		// send sample gps data
		mavlink_global_position_int_t gps;
		gps.time_boot_ms = boot_time_ms;
		boot_time_ms += 100;	// need to add boot_time_ms of your system here
		gps.lat = 40.730610 * pow(10, 7);	// the location of NewYork city
		gps.lon = -73.935242 * pow(10, 7);
		gps.alt = 50 * pow(10, 3);
		gps.relative_alt = 0;
		gps.vx = 0;		// don't use
		gps.vy = 0; 	// don't use
		gps.vz = 0;		// don't use
		gps.hdg = 90;	// the heading of GPS

		my_payload->sendPayloadGPSPosition(gps);
		printf("Send GPS data to payload: %d\n", msg_cnt++);

		usleep(100000); // 100ms, 10Hz
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