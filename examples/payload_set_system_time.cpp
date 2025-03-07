/**
 * This sample will send the SYSTEM TIME to the payload
 **/

#include "stdio.h"
#include"payloadSdkInterface.h"

#include <iostream>
#include <chrono>

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

// Function to get the epoch time
long long getEpochTimeInMicroseconds() {
    // Get current time point
    auto now = std::chrono::system_clock::now();

    // Convert to time since epoch in microseconds
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    
    return duration.count();
}

int main(int argc, char *argv[]){
	printf("Starting SendSystemTime example...\n");
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
		// send sample sysytem time
		mavlink_system_time_t sys_time;
		sys_time.time_boot_ms = boot_time_ms;
		boot_time_ms += 100;	// need to add boot time of your system here
		sys_time.time_unix_usec = getEpochTimeInMicroseconds();	// in epoc time

		my_payload->sendPayloadSystemTime(sys_time);
		printf("Send Sytem Time to payload: %d, %ld\n", msg_cnt++, sys_time.time_unix_usec);

		usleep(1000000); // 1000ms, 1Hz
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