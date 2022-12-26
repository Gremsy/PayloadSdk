/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.h
 *
 * @brief Autopilot interface definition
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "generic_port.h"

#include <stdio.h>
#include <iostream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <mutex>
#include <queue>

#include "user_mavlink_common.h"
// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
uint64_t get_time_usec();
uint64_t get_time_msec();
void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;
	uint64_t command_long;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
		battery_status = 0;
		radio_status = 0;
		local_position_ned = 0;
		global_position_int = 0;
		position_target_local_ned = 0;
		position_target_global_int = 0;
		highres_imu = 0;
		attitude = 0;
		command_long = 0;
	}

};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Radio Status
	mavlink_radio_status_t radio_status;

	// Local Position
	mavlink_local_position_ned_t local_position_ned;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;

	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;

	// HiRes IMU
	mavlink_highres_imu_t highres_imu;

	// Attitude
	mavlink_attitude_t attitude;

	// Command long
	mavlink_command_long_t command_long;

/* camera protocol */
	


	// Time Stamps
	Time_Stamps time_stamps;

	void
	reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

class Autopilot_Interface
{

public:

	Autopilot_Interface(){};
	Autopilot_Interface(Generic_Port *port_, int sys_id, int comp_id, int version, int channel);
	~Autopilot_Interface();

	char reading_status;
	char writing_status;
	char control_status;
    uint64_t write_count;

    int system_id;
	int autopilot_id;
	int companion_id;

	Mavlink_Messages current_messages;
	int current_message_id;
	mavlink_set_position_target_local_ned_t initial_position;

	void read_messages();
	int  write_message(mavlink_message_t message);

	void start();
	void stop();

	void start_read_thread();
	void start_write_thread(void);

	void handle_quit( int sig );

	int SYS_ID = 10;
	int SYS_COMP = MAV_COMP_ID_CAMERA;
	int TARGET_ID = 1;
	int TARGET_COMP = 254;

	void set_system_id(int _sys_id);
	int8_t get_nxt_message(mavlink_message_t& _message);
	void push_message_to_queue(mavlink_message_t message);
	virtual void w_heartbeat();


protected:

	Generic_Port *port;

	bool time_to_exit;

	pthread_t read_tid;
	pthread_t write_tid;

	void read_thread();
	void write_thread(void);

	// mavlink message queue
	pthread_mutex_t read_queue_mutex = PTHREAD_MUTEX_INITIALIZER;
	std::queue<mavlink_message_t> m_mav_message;

	pthread_mutex_t write_queue_mutex = PTHREAD_MUTEX_INITIALIZER;
	std::queue<mavlink_message_t> m_mav_write_message;

	int cnt=0;

private:

};


#endif // AUTOPILOT_INTERFACE_H_


