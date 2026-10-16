#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <algorithm>

#include "gamepad_reader.h"
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
void onPayloadStatusChanged(int event, double* param);

static std::atomic<bool> g_quit{false};

static void onSignal(int) { 
    g_quit = true; 
}

#include <algorithm>

// Map axis value -32767..32767 to outMin..outMax
inline float mapAxis(int value, float outMin = -180.0f, float outMax = 180.0f) {
    value = std::clamp(value, -32767, 32767);  // guard against -32768
    return outMin + (value + 32767) * (outMax - outMin) / 65534.0f;
}

// control values buffer
float _gb_pitch = 0, _gb_roll = 0,_gb_yaw = 0;
double eo_zoom = 0.0;
double ir_zoom = 0.0;
int _cam_zoom_dir = 0;
uint16_t _flags = 0;
bool _gb_rq_home = false;

void gimbalSetHome(){
    if(my_payload != nullptr){
        printf("Set gimbal return HOME \n");
        _flags |= GIMBAL_DEVICE_FLAGS_NEUTRAL;
        _flags &= ~GIMBAL_DEVICE_FLAGS_RETRACT; // xoa co
        _flags &= ~(1 << 14);                   // xoa co mapping
        my_payload->setGimbalMode(_flags);
        usleep(100000);
        // reset the control bit
        _flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL; // xoa co
        my_payload->setGimbalMode(_flags);
        usleep(3000000);
        } 
}

// Callback nhan su kien tu thread cua GamepadReader.
// Luu y: ham nay chay tren thread khac voi main -> giu ngan gon,
// khong block lau (vd: khong goi lenh PSDK cham trong nay).
static void onGamepadEvent(const GamepadEvent& ev) {
    switch (ev.type) {
    case GamepadEventType::Connected:
        std::cout << "Connected: " << ev.deviceName << " at " << ev.devicePath << std::endl;
        break;

    case GamepadEventType::Disconnected:
        std::cout << "Gamepad disconnected" << std::endl;
        break;

    case GamepadEventType::Button:
        std::cout << "Button " << GamepadReader::buttonName(ev.number)
                  << (ev.value ? " pressed" : " released") << std::endl;

        if (ev.number == (int)GamepadButton::A && ev.value == 1) {
            if(ev.value)
                _gb_rq_home = true;
        }
        break;

    case GamepadEventType::Axis:
        std::cout << "Axis " << GamepadReader::axisName(ev.number)
                  << " = " << ev.value << std::endl;

        // Gimbal control
        if (ev.number == (int)GamepadAxis::LY) {
            _gb_pitch = mapAxis(ev.value, 20, -20);
        }
        if (ev.number == (int)GamepadAxis::LX) {
            _gb_yaw = mapAxis(ev.value, 90, -90);
        }

        // Camera control
        if (ev.number == (int)GamepadAxis::RY) {
            if(ev.value < -100)
                _cam_zoom_dir = 1;
            else if(ev.value > 100)
                _cam_zoom_dir = -1;
            else _cam_zoom_dir = 0;
        }
        break;
    }
}

int main() {
    std::signal(SIGINT, onSignal);   // Ctrl+C
    std::signal(SIGTERM, onSignal);

    printf("Starting Gamepad control app...\n");

    // create payloadsdk object
    my_payload = new PayloadSdkInterface(s_conn);

    // init payload
    my_payload->sdkInitConnection();
    printf("Waiting for payload signal! \n");

    // register callback function
    my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

    // check connection
    my_payload->checkPayloadConnection();
    usleep(100000);

    printf("Set gimbal RC mode \n");
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, PAYLOAD_CAMERA_RC_MODE_STANDARD, PARAM_TYPE_UINT32);
    usleep(100000);
    printf("Set view mode to EO \n");
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
    usleep(100000);

    GamepadReader reader;
    reader.setCallback(onGamepadEvent);
    reader.start();

    std::cout << "Waiting for gamepad... (Ctrl+C de thoat)" << std::endl;

    // Main thread lam viec khac o day
    while (!g_quit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if(my_payload != nullptr){
            // printf("Moving gimbal by: %.2f %.2f %.2f \n", _gb_pitch, _gb_roll, _gb_yaw);
            // send control signal to gimbal
            
            my_payload->setGimbalSpeed(_gb_pitch, _gb_roll , _gb_yaw, INPUT_SPEED);

            // printf("Apply camera zoom continuous control \n");
            if (_cam_zoom_dir == 1) {
                my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_IN);
            } else if (_cam_zoom_dir == -1) {
                my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_OUT);
            } else if (_cam_zoom_dir == 0) {
                my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_STOP);
            }

            if(_gb_rq_home){
                _gb_rq_home = !_gb_rq_home;
                gimbalSetHome();
            }
        }
    }

    reader.stop();
    std::cout << "Exit" << std::endl;
    return 0;
}

void onPayloadStatusChanged(int event, double* param){
    
    switch(event){
    case PAYLOAD_GB_ATTITUDE:{
        // param[0]: pitch
        // param[1]: roll
        // param[2]: yaw

        // printf("Pich: %.2f - Roll: %.2f - Yaw: %.2f\n", param[0], param[1], param[2]);
        break;
    }
    case PAYLOAD_PARAMS:{
        if(param[0] == PARAM_EO_ZOOM_LEVEL){
            eo_zoom = param[1];
            printf("eo_zoom: %.2f \n", eo_zoom);
        }
        else if(param[0] == PARAM_IR_ZOOM_LEVEL){
            ir_zoom = param[1];
        }
        break;
    }
    default: break;
    }
}