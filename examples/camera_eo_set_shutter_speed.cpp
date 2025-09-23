#include "stdio.h"
#include <pthread.h>
#include <cstdlib>

#include "payloadSdkInterface.h"

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

void onPayloadParamChanged(int event, char* param_char, double* param_double);
void quit_handler(int sig);

int main(int argc, char *argv[]){
    printf("Starting SetShutterSpeed example...\n");
    signal(SIGINT, quit_handler);

    // create payloadsdk object
    my_payload = new PayloadSdkInterface(s_conn);

    // init payload
    my_payload->sdkInitConnection();
    printf("Waiting for payload signal! \n");

    // register callback function
    my_payload->regPayloadParamChanged(onPayloadParamChanged);

    // check connection
    my_payload->checkPayloadConnection();

    // Only handle for VIO/ZIO where shutter param is defined
    #if defined VIO || defined ZIO
        printf("[EO] Forcing view source to EO...\n");
	    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
        
        // Read current shutter speed only (by ID)
        my_payload->getPayloadCameraSettingByID((char*)PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED);
        usleep(1000000);

        // Ensure exposure mode allows shutter control
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE, PAYLOAD_CAMERA_VIDEO_EXPOSURE_SHUTTER, PARAM_TYPE_UINT32);
        usleep(1000000);
    #endif /* VIO || ZIO */

    while(true)
    {
        // Only handle for VIO/ZIO where shutter param is defined
        #if defined VIO || defined ZIO
            // Set shutter speed to 1/1000 (change constant as needed)
            my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED, PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_10, PARAM_TYPE_UINT32);
            usleep(1000000);

            // Read back shutter speed only (by ID)
            my_payload->getPayloadCameraSettingByID((char*)PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED);
            usleep(1000000);

            // Set shutter speed to 1/1000 (change constant as needed)
            my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED, PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1000, PARAM_TYPE_UINT32);
            usleep(1000000);

            // Read back shutter speed only (by ID)
            my_payload->getPayloadCameraSettingByID((char*)PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED);
            usleep(1000000);
        #else
            exit(-1);
        #endif /* VIO || ZIO */
    }

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error) {}

    return 0;
}

void quit_handler(int sig){
    printf("\n");
    printf("TERMINATING AT USER REQUEST \n");
    printf("\n");

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error) {}

    // end program here
    exit(0);
}

static const char* shutter_value_to_label(int value){
    switch(value){
        case PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_10: return "1/10";
        case PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_20: return "1/20";
        case PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_50: return "1/50";
        case PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_100: return "1/100";
        case PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_125: return "1/125";
        case PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_500: return "1/500";
        case PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_725: return "1/725";
        case PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1000: return "1/1000";
        case PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1500: return "1/1500";
        case PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_2000: return "1/2000";
        default: return "unknown";
    }
}

void onPayloadParamChanged(int event, char* param_char, double* param){
    switch(event){
    case PAYLOAD_CAM_PARAMS:{
        // Only log shutter speed updates
        if(strcmp(param_char, PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED) == 0){
            int code = static_cast<int>(param[1]);
            printf("Shutter (C_V_SP): %s (code %d)\n", shutter_value_to_label(code), code);
        }
        break;
    }
    default: break;
    }
}