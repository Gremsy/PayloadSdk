#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <future>
#include <string>
using namespace std;

#include"payloadSdkInterface.h"

#define AUTOTUNEPARAM 6
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

pthread_t thrd_recv;
pthread_t thrd_gstreamer;

bool gstreamer_start();
void gstreamer_terminate();
void *start_loop_thread(void *threadid);


bool all_threads_init();
void quit_handler(int sig);

std::mutex            mutex_ack{};
std::mutex            mutex_params{};
using CommandResultCallback = std::function<void(double result[2])>;
using std::chrono::seconds;
using std::chrono::minutes;
CommandResultCallback callbackack;

enum AutoTuneState : uint8_t{
    AUTOTUNE_START      = 0,
    AUTOTUNE_WAIT_AUTOTUNE,
    AUTOTUNE_END
};



void onPayloadStatusChanged(int event, double* param){
	
	switch(event){
	case PAYLOAD_GB_ACK:{

        std::lock_guard<std::mutex> lock(mutex_ack);
        if(callbackack != nullptr){
            callbackack(param);
        }
		break;
	}
	default: break;
	}
}
// send auto tune sync
uint8_t send_auto_tune_sync(bool isStartAutoTune, PayloadSdkInterface* _my_payload){

    if(_my_payload == nullptr){
        return MAV_RESULT_FAILED;
    }

    uint16_t cmdAutoTune = MAV_CMD_USER_3;
    auto prom = std::make_shared<std::promise<uint8_t>>();
    auto res = prom->get_future();
    {
        std::lock_guard<std::mutex> lock(mutex_ack);
        callbackack = [&](double result[2]){
            if (static_cast<uint16_t>(result[0]) == cmdAutoTune)
            {
                prom->set_value(static_cast<uint8_t>(result[1]));
            }
        };
    }
    
    _my_payload->setGimbalAutoTune(isStartAutoTune);
    if (res.wait_for(seconds(1)) == std::future_status::timeout) {
        {
            std::lock_guard<std::mutex> lock(mutex_ack);
            callbackack = nullptr;
        }
        return MAV_RESULT_ENUM_END;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_ack);
        callbackack = nullptr;
    }
    return res.get();
}
// wait auto tune sync
uint8_t wait_auto_tune_sync(PayloadSdkInterface* _my_payload)
{
    if(_my_payload == nullptr){
        return MAV_RESULT_FAILED;
    }

    uint16_t cmdAutoTune = MAV_CMD_USER_3;
    auto prom = std::make_shared<std::promise<uint8_t>>();
    auto res = prom->get_future();
    {
        std::lock_guard<std::mutex> lock(mutex_ack);
        callbackack = [&](double result[2]){
            if (static_cast<uint16_t>(result[0]) == cmdAutoTune)
            {
                prom->set_value(static_cast<uint8_t>(result[1]));
            }
        };
    }

    if (res.wait_for(minutes(3)) == std::future_status::timeout) {
        {
            std::lock_guard<std::mutex> lock(mutex_ack);
            callbackack = nullptr;
        }
        return MAV_RESULT_ENUM_END;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_ack);
        callbackack = nullptr;
    }
    return res.get();
}


int main(int argc, char *argv[]){
	printf("Starting Set gimbal mode example...\n");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

    // init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

    // check connection
    my_payload->checkPayloadConnection();

    // auto tune
    uint8_t total_e = 0;
    AutoTuneState state = AUTOTUNE_START;
    bool time_to_exit = false;

    while (!time_to_exit)
    {
        switch (state)
        {
        case AUTOTUNE_START:
        {
            uint8_t ret = send_auto_tune_sync(true, my_payload);
            if(ret == MAV_RESULT_ACCEPTED){
                printf("Start auto tune!\n");
                state = AUTOTUNE_WAIT_AUTOTUNE;
            }
            else{  
                total_e++;
            }
        }
            break;
        case AUTOTUNE_WAIT_AUTOTUNE:
        {
            
            uint8_t ret = wait_auto_tune_sync(my_payload);
            if(ret == MAV_RESULT_ACCEPTED){
                printf("Gimbal auto tune done!\n");
                state = AUTOTUNE_END;
            }
            else{  
                total_e++;
            }
        }   
            break;
        case AUTOTUNE_END:
        {
            // check connection
            my_payload->checkPayloadConnection();
            time_to_exit = true;
        }
            break;
        default:
            break;
        }
        
        if (total_e > 3)
        {
            printf("Error: %d\n", total_e);
            time_to_exit = true;
            break;
        }
        
        usleep(100000);
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

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    // end program here
    exit(0);
}