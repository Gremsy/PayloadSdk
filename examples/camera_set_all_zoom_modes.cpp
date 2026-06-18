#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <string>
#include <limits>

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

double eo_zoom = 0.0;
double ir_zoom = 0.0;
std::string current_menu = "";

void quit_handler(int sig);
void onPayloadStatusChanged(int event, double* param);

void printCommandZoomMode();
void printCommandZoomType();
void printCommandCameraType();

void zoomStepProcess();
void zoomContinuousProcess();
void zoomRangeProcess();

void clearAndPrint(){
    printf("\033[2J\033[H");
    printf("[Zoom] EO: %.2f    IR: %.2f\n", eo_zoom, ir_zoom);
    printf("─────────────────────────────────\n");
    printf("%s", current_menu.c_str());
    fflush(stdout);
}

void updateZoomLine(){
    printf("\033[s");        
    printf("\033[1;1H");     
    printf("\033[2K");       
    printf("[Zoom] EO: %.2f    IR: %.2f", eo_zoom, ir_zoom);
    printf("\033[u");       
    fflush(stdout);
}

int main(int argc, char *argv[]){
    SDK_LOG("Starting Set zoom example...");
    signal(SIGINT, quit_handler);

    my_payload = new PayloadSdkInterface(s_conn);

    my_payload->sdkInitConnection();
    SDK_LOG("Waiting for payload signal!");

    my_payload->regPayloadStatusChanged(onPayloadStatusChanged);
    my_payload->checkPayloadConnection();

    my_payload->setParamRate(PARAM_EO_ZOOM_LEVEL, 1000);
    my_payload->setParamRate(PARAM_IR_ZOOM_LEVEL, 1000);

    while (!time_to_exit){

        // Select Camera
        printCommandCameraType();
        char cam_input;
        std::cin >> cam_input;

        if (cam_input == 'q') break;
        if (cam_input != '0' && cam_input != '1') {
            current_menu = "Invalid input! Please try again.\n\n"
                           "Press '0': EO Camera\n"
                           "Press '1': IR Camera\n";
            clearAndPrint();
            continue;
        }

        if (cam_input == '0') {
            my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
        } else if (cam_input == '1') {
            my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IR, PARAM_TYPE_UINT32);
        } 
        
        // Select zoom mode (EO only)
        if (cam_input == '0'){
            printCommandZoomMode();

            char mode_input;
            std::cin >> mode_input;

            if (mode_input == '0') {
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_COMBINE, PARAM_TYPE_UINT32);
            } else if (mode_input == '1') {
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION, PARAM_TYPE_UINT32);
            } else {
                current_menu = "Invalid input! Please try again.\n\n"
                               "Press '0': Zoom Combine Mode\n"
                               "Press '1': Zoom Super Resolution Mode\n";
                clearAndPrint();
                continue;
            }
        }

        // Select zoom type
        printCommandZoomType();
        char zoom_type_input;
        std::cin >> zoom_type_input;

        if (zoom_type_input == '0') {
            zoomStepProcess();
        } else if (zoom_type_input == '1') {
            zoomContinuousProcess();
        } else if (zoom_type_input == '2') {
            zoomRangeProcess();
        } else {
            current_menu = "Invalid input! Please try again.\n\n"
                           "Press '0': Zoom Step\n"
                           "Press '1': Zoom Continuous\n"
                           "Press '2': Zoom Range\n";
            clearAndPrint();
        }
    }

    return 0;
}

void quit_handler(int sig){
    SDK_LOG("");
    SDK_LOG("TERMINATING AT USER REQUEST");
    SDK_LOG("");

    time_to_exit = true;

    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    exit(0);
}

void onPayloadStatusChanged(int event, double* param){
    switch(event){
    case PAYLOAD_PARAMS:{
        if(param[0] == PARAM_EO_ZOOM_LEVEL){
            eo_zoom = param[1];
        }
        else if(param[0] == PARAM_IR_ZOOM_LEVEL){
            ir_zoom = param[1];
        }
        updateZoomLine();
        break;
    }
    default: break;
    }
}

void printCommandCameraType(){
    current_menu =
        "Press '0': EO Camera\n"
        "Press '1': IR Camera\n"
        "Press 'q': Quit\n";
    clearAndPrint();
}

void printCommandZoomMode(){
    current_menu =
        "Press '0': Zoom Combine Mode\n"
        "Press '1': Zoom Super Resolution Mode\n";
    clearAndPrint();
}

void printCommandZoomType(){
    current_menu =
        "Press '0': Zoom Step\n"
        "Press '1': Zoom Continuous\n"
        "Press '2': Zoom Range\n";
    clearAndPrint();
}

void printZoomStepCommand(){
    current_menu =
        "Press '+': Zoom In (step)\n"
        "Press '-': Zoom Out (step)\n"
        "Press 'q': Back to main menu\n";
    clearAndPrint();
}

void zoomStepProcess(){
    printZoomStepCommand();
    while (!time_to_exit) {
        char input;
        std::cin >> input;

        if (input == '+') {
            my_payload->setCameraZoom(ZOOM_TYPE_STEP, ZOOM_IN);
        } else if (input == '-') {
            my_payload->setCameraZoom(ZOOM_TYPE_STEP, ZOOM_OUT);
        } else if (input == 'q') {
            break;
        } else {
            current_menu = "Invalid input!\n\n"
                           "Press '+': Zoom In (step)\n"
                           "Press '-': Zoom Out (step)\n"
                           "Press 'q': Back to main menu\n";
        }
        clearAndPrint();
    }
}

void printZoomContinuousCommand(){
    current_menu =
        "Press '+': Start Zoom In (continuous)\n"
        "Press '-': Start Zoom Out (continuous)\n"
        "Press 's': Stop Zoom\n"
        "Press 'q': Back to main menu\n";
    clearAndPrint();
}

void zoomContinuousProcess(){
    printZoomContinuousCommand();
    while (!time_to_exit) {
        char input;
        std::cin >> input;

        if (input == '+') {
            my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_IN);
        } else if (input == '-') {
            my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_OUT);
        } else if (input == 's') {
            my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_STOP);
        } else if (input == 'q') {
            my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_STOP);
            break;
        } else {
            current_menu = "Invalid input!\n\n"
                           "Press '+': Start Zoom In (continuous)\n"
                           "Press '-': Start Zoom Out (continuous)\n"
                           "Press 's': Stop Zoom\n"
                           "Press 'q': Back to main menu\n";
        }
        clearAndPrint();
    }
}

void printZoomRangeCommand(){
    current_menu =
        "Enter zoom percentage (0.0 - 100.0)\n"
        "Enter '-1' to go back to main menu\n";
    clearAndPrint();
}

void zoomRangeProcess(){
    printZoomRangeCommand();
    while (!time_to_exit) {

        float zoom_percent;
        if (!(std::cin >> zoom_percent)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            current_menu = "Invalid input! Please enter a number.\n\n"
                        "Enter zoom percentage (0.0 - 100.0)\n"
                        "Enter '-1' to go back to main menu\n";
            clearAndPrint();
            continue;
        }
        if (zoom_percent == -1.0f) {
            break;
        }
        if (zoom_percent < 0.0 || zoom_percent > 100.0) {
            current_menu = "Invalid value! Please enter a value between 0.0 and 100.0\n\n"
                           "Enter zoom percentage (0.0 - 100.0)\n"
                           "Enter '-1' to go back to main menu\n";
        } else {
            my_payload->setCameraZoom(ZOOM_TYPE_RANGE, zoom_percent);
            current_menu =
                "Enter zoom percentage (0.0 - 100.0)\n"
                "Enter '-1' to go back to main menu\n";
        }
        clearAndPrint();
    }
}
