/**
 * This sample will send the GPS position to the payload
 * 
 * Logic:
 * 1. Always send GPS_RAW_INT to allow GPS to get fix
 * 2. Only send GLOBAL_POSITION_INT when GPS fix is successful (GPS_FIX_TYPE_DGPS)
 * 3. GPS coordinates change continuously to simulate movement
 * 
 * The GPS information will include:
 * 1. Latitude (changing continuously)
 * 2. Longitude (changing continuously)
 * 3. Altitude (changing continuously)
 * 4. Heading
 * 5. GPS Fix Type
 * 
 * This example simulates GPS fix progression and movement:
 * - Messages 0-19:  NO_FIX (send GPS_RAW_INT only)
 * - Messages 20+:   DGPS (send GPS_RAW_INT + GLOBAL_POSITION_INT)
 * 
 * Movement simulation: From New York to Philadelphia
 * 
 * In a real application, you would get the GPS fix status from your actual GPS hardware.
 **/

#include "stdio.h"
#include"payloadSdkInterface.h"
#include <chrono>
#include <math.h>

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

// Simulate GPS fix status - in real application, this would come from actual GPS hardware
uint8_t current_gps_fix_type = GPS_FIX_TYPE_NO_FIX;

// GPS coordinates for movement simulation
struct GPSCoordinate {
    double lat;
    double lon;
    double alt;
    double heading;
};

// Start point: New York City
GPSCoordinate start_pos = {40.730610, -73.935242, 50.0, 225.0};  // Heading Southwest
// End point: Philadelphia  
GPSCoordinate end_pos = {39.952583, -75.165222, 80.0, 225.0};

// Function to calculate current GPS position based on movement
GPSCoordinate getCurrentGPSPosition(int msg_cnt) {
    GPSCoordinate current_pos;
    
    // Movement starts after 100 messages to allow GPS fix
    int movement_start = 100;
    int total_movement_time = 500;  // 500 messages to complete the journey
    
    if (msg_cnt < movement_start) {
        // Stay at start position
        current_pos = start_pos;
    } else if (msg_cnt < movement_start + total_movement_time) {
        // Calculate interpolation factor (0.0 to 1.0)
        double progress = (double)(msg_cnt - movement_start) / total_movement_time;
        
        // Linear interpolation between start and end positions
        current_pos.lat = start_pos.lat + (end_pos.lat - start_pos.lat) * progress;
        current_pos.lon = start_pos.lon + (end_pos.lon - start_pos.lon) * progress;
        current_pos.alt = start_pos.alt + (end_pos.alt - start_pos.alt) * progress;
        current_pos.heading = start_pos.heading;
        
        // Add some random variation to make it more realistic
        double variation = 0.0001;  // Small random variation
        current_pos.lat += (rand() % 100 - 50) * variation / 100.0;
        current_pos.lon += (rand() % 100 - 50) * variation / 100.0;
        current_pos.alt += (rand() % 10 - 5) * 0.5;  // ±2.5m altitude variation
    } else {
        // Stay at end position
        current_pos = end_pos;
    }
    
    return current_pos;
}

// Function to simulate GPS fix progression
void updateGPSFixStatus(int msg_cnt) {
    // Simulate GPS fix progression: NO_FIX -> DGPS
    if (msg_cnt < 20) {
        current_gps_fix_type = GPS_FIX_TYPE_NO_FIX;
    } else {
        current_gps_fix_type = GPS_FIX_TYPE_DGPS;
    }
}

int main(int argc, char *argv[]){
	printf("Starting SendGPS example with moving coordinates...\n");
	printf("Route: New York City -> Philadelphia\n");
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
		// Update GPS fix status simulation
		updateGPSFixStatus(msg_cnt);
		
		// Get current GPS position (changing continuously)
		GPSCoordinate current_gps = getCurrentGPSPosition(msg_cnt);
		
		printf("GPS Fix Type: %d (%s) | Lat: %.6f, Lon: %.6f, Alt: %.1fm\n", 
			current_gps_fix_type,
			current_gps_fix_type == GPS_FIX_TYPE_NO_FIX ? "NO_FIX" :
			current_gps_fix_type == GPS_FIX_TYPE_DGPS ? "DGPS" : "UNKNOWN",
			current_gps.lat, current_gps.lon, current_gps.alt);

		// Always send GPS_RAW_INT to allow GPS to get fix
		{
			// Get current time in microseconds
			auto now = std::chrono::system_clock::now();
			auto time_usec = std::chrono::duration_cast<std::chrono::microseconds>(
				now.time_since_epoch()).count();

			// send GPS RAW data to allow GPS fix
			mavlink_gps_raw_int_t gps_raw;
			gps_raw.time_usec = time_usec;
			gps_raw.fix_type = current_gps_fix_type;  // Current fix type
			gps_raw.lat = current_gps.lat * pow(10, 7);		// Current latitude
			gps_raw.lon = current_gps.lon * pow(10, 7);		// Current longitude
			gps_raw.alt = current_gps.alt * pow(10, 3);		// Current altitude in mm
			gps_raw.eph = 100;						// HDOP * 100 (1.0 HDOP)
			gps_raw.epv = 150;						// VDOP * 100 (1.5 VDOP)
			gps_raw.vel = (msg_cnt > 100 && msg_cnt < 600) ? 500 : 0;  // 5 m/s when moving
			gps_raw.cog = current_gps.heading * 100;		// course over ground in cdeg
			gps_raw.satellites_visible = (current_gps_fix_type == GPS_FIX_TYPE_DGPS) ? 8 : 3;  // More sats when better fix
			gps_raw.alt_ellipsoid = current_gps.alt * pow(10, 3); // altitude above ellipsoid in mm
			gps_raw.h_acc = 2000;					// horizontal accuracy in mm (2m)
			gps_raw.v_acc = 3000;					// vertical accuracy in mm (3m)
			gps_raw.vel_acc = 100;					// velocity accuracy in mm/s
			gps_raw.hdg_acc = 500;					// heading accuracy in degE5 (0.005 deg)
			gps_raw.yaw = current_gps.heading * 100;		// yaw in cdeg

			my_payload->sendPayloadGPSRawInt(gps_raw);
			printf("Send GPS_RAW_INT to payload: %d (Fix Type: %d)\n", msg_cnt, current_gps_fix_type);
		}

		// Only send GLOBAL_POSITION_INT when GPS fix is successful (DGPS)
		if (current_gps_fix_type == GPS_FIX_TYPE_DGPS) {
			// send sample gps data using original function
			mavlink_global_position_int_t gps;
			gps.time_boot_ms = boot_time_ms;
			boot_time_ms += 100;	// need to add boot_time_ms of your system here
			gps.lat = current_gps.lat * pow(10, 7);		// Current latitude
			gps.lon = current_gps.lon * pow(10, 7);		// Current longitude
			gps.alt = current_gps.alt * pow(10, 3);		// Current altitude
			gps.relative_alt = 0;
			gps.vx = 0;		// don't use
			gps.vy = 0; 	// don't use
			gps.vz = 0;		// don't use
			gps.hdg = current_gps.heading;	// Current heading

			my_payload->sendPayloadGPSPosition(gps);
			printf("Send GLOBAL_POSITION_INT to payload: %d (DGPS Fix Available)\n", msg_cnt);
		}

		msg_cnt++;
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