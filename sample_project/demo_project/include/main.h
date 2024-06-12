#ifndef __MAIN_H
#define __MAIN_H

#include "log.h"
#include "payloadSdkInterface.h"
#include <cstdlib>
#include <pthread.h>
#include <iostream>
#include <chrono>

#if defined GHADRON
#include "ghadron_sdk.h"
#else
#include "vio_sdk.h"
#endif

/**/
PayloadSdkInterface* my_payload = nullptr;

bool time_to_exit = false;
/*!< Private prototype*/
static uint64_t _get_time_usec(){
 	auto currentTime = std::chrono::high_resolution_clock::now();
 	auto duration = currentTime.time_since_epoch();
 	return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}
static uint64_t _get_time_msec(){
	auto currentTime = std::chrono::system_clock::now();
	auto duration = currentTime.time_since_epoch();
	return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}
/*!< Private typedef*/
typedef enum{
	STATE_IDLE = 0,
	STATE_ERROR,
	STATE_WAIT_TO_CONNECT_PAYLOAD,
	STATE_SET_PAYLOAD_PARAM,
	STATE_LOAD_PAYLOAD_PARAM,
	STATE_START_MOVEMENT,
	STATE_MOVEMENT_0,
	STATE_MOVEMENT_1,
	STATE_MOVEMENT_2,
	STATE_MOVEMENT_3,
	STATE_MOVEMENT_4,
	STATE_MOVEMENT_5,
	STATE_MOVEMENT_6,
	STATE_MOVEMENT_7,
	STATE_MOVEMENT_8,
	STATE_DONE,
} E_sample_process_state;

const char* state_name[STATE_DONE + 1] = {
	"STATE_IDLE = 0",
	"STATE_ERROR",
	"STATE_WAIT_TO_CONNECT_PAYLOAD",
	"STATE_SET_PAYLOAD_PARAM",
	"STATE_LOAD_PAYLOAD_PARAM",
	"STATE_START_MOVEMENT",
	"STATE_MOVEMENT_0",
	"STATE_MOVEMENT_1",
	"STATE_MOVEMENT_2",
	"STATE_MOVEMENT_3",
	"STATE_MOVEMENT_4",
	"STATE_MOVEMENT_5",
	"STATE_MOVEMENT_6",
	"STATE_MOVEMENT_7",
	"STATE_MOVEMENT_8",
	"STATE_DONE"	
};

typedef struct{
	E_sample_process_state _state = STATE_IDLE;
	uint64_t _time_usec = _get_time_usec();
} T_psdk_process_state;



#endif 