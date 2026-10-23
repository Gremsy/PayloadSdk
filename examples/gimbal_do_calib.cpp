/**
 * Unified gimbal calibration sample: CALIB_GYRO, AUTO_TUNE
 *
 * Usage:
 *     ./gimbal_calib gyro
 *     ./gimbal_calib autotune
 *
 * The calibration is very sensitivity. Please carefully consider before doing calibration. 
 * PLEASE IGNORE THIS SAMPLE IF YOU ARE NOT FULLY UNDERSTANDING WHAT YOU WILL DO!!!
 * This sample will show you how to send command to do gimbal calibration
 **/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "payloadSdkInterface.h"

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

enum calib_type_t{
    CALIB_GYRO = 0,
    AUTO_TUNE,
};

// ---------------------------------------------------------------------------
// state shared between the main thread and the SDK callback thread
// ---------------------------------------------------------------------------
calib_type_t myCalib = CALIB_GYRO;
double       g_cmd_id = 0;                  // command id of the running mode (set before sending)

std::atomic<bool> is_exit{false};
std::atomic<bool> is_calibration_runing{false};
std::atomic<bool> is_success{false};
std::atomic<int>  g_last_result{-1};
std::atomic<int>  g_last_progress{-1};
std::atomic<long long> last_activity_ms{0};

// gimbal params received (id -> value)
std::mutex g_mtx;
std::map<std::string, double> g_params;

// timeouts
static const long long START_TIMEOUT_MS = 5000;

void quit_handler(int sig);
void onPayloadStatusChanged(int event, double* param);
void onPayloadParamChanged(int event, char* param_char, double* param);

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
static long long now_ms(){
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static const char* result_str(int r){
    switch(r){
    case 0: return "ACCEPTED";
    case 1: return "TEMPORARILY_REJECTED";
    case 2: return "DENIED";
    case 3: return "UNSUPPORTED";
    case 4: return "FAILED";
    case 5: return "IN_PROGRESS";
    case 6: return "CANCELLED";
    default: return "UNKNOWN";
    }
}

static const char* mode_name(calib_type_t m){
    switch(m){
    case CALIB_GYRO:  return "gyro calibration";
    case AUTO_TUNE:   return "auto tune";
    }
    return "?";
}

static long long stall_timeout_ms(calib_type_t m){
    switch(m){
    case CALIB_GYRO:  return 30000;
    case AUTO_TUNE:   return 60000;
    }
    return 60000;
}

// Read a list of gimbal params. Requests are spaced out and the ones that did not
// answer are requested again (the gimbal / UDP can drop some of them).
static bool read_params(const std::vector<std::string>& ids, const char* label){
    SDK_LOG("Reading gimbal params (%s)...", label);

    {
        std::lock_guard<std::mutex> lk(g_mtx);
        g_params.clear();
    }

    for(int attempt = 0; attempt < 3; attempt++){
        for(const std::string& id : ids){
            bool have;
            {
                std::lock_guard<std::mutex> lk(g_mtx);
                have = g_params.count(id) > 0;
            }
            if(!have){
                char id_buf[32];
                snprintf(id_buf, sizeof(id_buf), "%s", id.c_str());
                my_payload->getPayloadGimbalSettingByID(id_buf);
                usleep(200000);
            }
        }
        usleep(1000000);

        std::lock_guard<std::mutex> lk(g_mtx);
        if(g_params.size() >= ids.size()) return true;
    }

    SDK_LOG("WARNING: not all params were received (%s)", label);
    return false;
}

static std::map<std::string, double> snapshot_params(){
    std::lock_guard<std::mutex> lk(g_mtx);
    return g_params;
}

static void usage(const char* prog){
    printf("Usage: %s <gyro|autotune>\n", prog);
    printf("  gyro     : gyro calibration\n");
    printf("  autotune : gimbal auto tune (firmware 790.6 or higher)\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]){
    if(argc < 2){
        usage(argv[0]);
        return 2;
    }
    if(strcmp(argv[1], "gyro") == 0)          myCalib = CALIB_GYRO;
    else if(strcmp(argv[1], "autotune") == 0) myCalib = AUTO_TUNE;
    else{
        usage(argv[0]);
        return 2;
    }

    g_cmd_id = (myCalib == AUTO_TUNE) ? (double)MAV_CMD_USER_3
                                      : (double)MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;

    printf("Starting gimbal example: %s\n", mode_name(myCalib));
    signal(SIGINT, quit_handler);

    // creat payloadsdk object
    my_payload = new PayloadSdkInterface(s_conn);

    // init payload
    my_payload->sdkInitConnection();
    printf("Waiting for payload signal! \n");

    // register callback function
    my_payload->regPayloadStatusChanged(onPayloadStatusChanged);
    my_payload->regPayloadParamChanged(onPayloadParamChanged);

    // check connection
    my_payload->checkPayloadConnection();

    const std::vector<std::string> tune_ids = {
        "STIFF_TILT", "STIFF_ROLL", "STIFF_PAN", "PWR_TILT", "PWR_ROLL", "PWR_PAN"
    };
    std::map<std::string, double> before;

    // auto tune: record the current stiffness / hold strength first, so they can be restored
    if(myCalib == AUTO_TUNE){
        if(read_params(tune_ids, "before auto tune")){
            before = snapshot_params();
        }
        else{
            before = snapshot_params();
        }
    }

    // give the operator a last chance to cancel
    SDK_LOG("Keep the gimbal STILL. Starting %s in 3 s (Ctrl+C to cancel)...", mode_name(myCalib));
    usleep(3000000);

    // reset the state, then send the command
    is_calibration_runing = false;
    is_success = false;
    is_exit = false;
    g_last_result = -1;
    g_last_progress = -1;
    last_activity_ms = now_ms();

    switch(myCalib){
    case CALIB_GYRO:
        my_payload->sendPayloadGimbalCalibGyro();
        SDK_LOG("Calib gyro command was sent. Waiting for the calibration done...");
        break;
    case AUTO_TUNE:
        my_payload->sendPayloadGimbalAutoTune(true);
        SDK_LOG("Auto tune command was sent. Waiting for the process done...");
        break;
    }

    // waiting the process done, with timeouts
    const long long t_sent = now_ms();
    const long long stall_ms = stall_timeout_ms(myCalib);
    bool timed_out = false;

    while(!is_exit){
        usleep(200000);

        const long long idle = now_ms() - last_activity_ms.load();

        if(!is_calibration_runing && now_ms() - t_sent > START_TIMEOUT_MS && idle > START_TIMEOUT_MS){
            SDK_LOG("No answer from the gimbal after %lld s. Aborting.", START_TIMEOUT_MS / 1000);
            timed_out = true;
            break;
        }
        if(is_calibration_runing && idle > stall_ms){
            SDK_LOG("%s STALLED at %d%%: no activity for %lld s. Aborting.",
                    mode_name(myCalib), g_last_progress.load(), stall_ms / 1000);
            timed_out = true;
            break;
        }
    }

    // report + verify
    int ret = 1;
    if(is_success){
        SDK_LOG("%s finished successfully.", mode_name(myCalib));
        ret = 0;

        if(myCalib == AUTO_TUNE){
            SDK_LOG("Waiting for the gimbal rebooted...20s");
            usleep(20000000);

            SDK_LOG("Load the Stiffness/Holdstrength values...");
            read_params(tune_ids, "after auto tune");
            std::map<std::string, double> after = snapshot_params();

            SDK_LOG("---- Stiffness / Hold strength (before -> after) ----");
            for(const std::string& id : tune_ids){
                const bool b = before.count(id) > 0;
                const bool a = after.count(id) > 0;
                if(b && a){
                    SDK_LOG("%-10s: %.2f -> %.2f", id.c_str(), before[id], after[id]);
                }
                else if(a){
                    SDK_LOG("%-10s: (no old value) -> %.2f", id.c_str(), after[id]);
                }
                else if(b){
                    SDK_LOG("%-10s: %.2f -> (not received)", id.c_str(), before[id]);
                }
                else{
                    SDK_LOG("%-10s: (not received)", id.c_str());
                }
            }
        }
    }
    else if(timed_out){
        SDK_LOG("%s did NOT complete (timeout).", mode_name(myCalib));
    }
    else{
        SDK_LOG("%s did NOT complete: gimbal answered %s (%d).",
                mode_name(myCalib), result_str(g_last_result.load()), g_last_result.load());
    }

    usleep(1000000);
    SDK_LOG("Exit.");
    my_payload->sdkQuit();
    delete my_payload;
    my_payload = nullptr;

    return ret;
}

void quit_handler(int sig){
    printf("\n");
    printf("TERMINATING AT USER REQUEST \n");
    printf("\n");

    try {
        if(my_payload) my_payload->sdkQuit();
    }
    catch (int error){}

    exit(0);
}

// ---------------------------------------------------------------------------
// callbacks
// ---------------------------------------------------------------------------
void onPayloadStatusChanged(int event, double* param){
    switch(event){
    case PAYLOAD_ACK:{
        // param[0]: command
        // param[1]: result
        // param[2]: progress

        if(param[0] != g_cmd_id) break;

        SDK_LOG("Got PAYLOAD_ACK for command %.0f with status %.0f (%s), progress: %.0f",
                param[0], param[1], result_str((int)param[1]), param[2]);

        if(param[1] == MAV_RESULT_ACCEPTED){
            if(param[2] != 0){
                if(!is_calibration_runing){
                    is_calibration_runing = true;
                    last_activity_ms = now_ms();
                    SDK_LOG("The %s is starting!", mode_name(myCalib));
                }
            }
            else{
                last_activity_ms = now_ms();
                SDK_LOG("The %s done!", mode_name(myCalib));
                is_calibration_runing = false;
                is_success = true;
                is_exit = true;
            }
        }
        else if(param[1] == MAV_RESULT_IN_PROGRESS){
            is_calibration_runing = true;

            const int p = (int)param[2];
            if(p != g_last_progress.load()){
                g_last_progress = p;
                last_activity_ms = now_ms();
            }
            SDK_LOG("The %s is processing...at %.0f%%", mode_name(myCalib), param[2]);
        }
        else{
            // TEMPORARILY_REJECTED, DENIED, UNSUPPORTED, FAILED, CANCELLED...
            SDK_LOG("The %s was NOT accepted / failed: %s (%.0f)",
                    mode_name(myCalib), result_str((int)param[1]), param[1]);
            g_last_result = (int)param[1];
            is_calibration_runing = false;
            is_success = false;
            is_exit = true;
        }
        break;
    }
    default: break;
    }
}

void onPayloadParamChanged(int event, char* param_char, double* param){
    switch(event){
    case PAYLOAD_CAM_PARAMS:{
        // param[0]: param_index
        // param[1]: value
        SDK_LOG(" --> Payload_param: %s, value: %.2f", param_char, param[1]);
        break;
    }
    case PAYLOAD_GB_PARAMS:{
        // param[0]: param_index
        // param[1]: value
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            g_params[param_char] = param[1];
        }
        SDK_LOG("--> Gimbal_param: index: %.f, id: %s, value: %.2f", param[0], param_char, param[1]);
        break;
    }
    default: break;
    }
}