#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "gamepad_controller.h"
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


const int JOYSTICK_DEADZONE = 2500;

const float GIMBAL_PITCH_SPEED = 20.0f;
const float GIMBAL_YAW_SPEED   = 90.0f;
const float GIMBAL_ROLL_SPEED  = 20.0f;

const int VIEW_SRC_VALUES[]   = { 0, 1, 2, 3 };
const int VIEW_SRC_COUNT      = sizeof(VIEW_SRC_VALUES)   / sizeof(VIEW_SRC_VALUES[0]);

const int RECORD_SRC_VALUES[] = { 0, 1, 2, 5 };
const int RECORD_SRC_COUNT    = sizeof(RECORD_SRC_VALUES) / sizeof(RECORD_SRC_VALUES[0]);

const int TRACK_FRAME_W = 1920;
const int TRACK_FRAME_H = 1080;
const int TRACK_BOX     = 128;

enum tracking_mode_t {
    TRACK_STOP      = 0,
    TRACK_ACTIVE    = 1,
    TRACK_EAGLEEYES = 2
};

enum class GimbalMode { FOLLOW, LOCK, MAPPING, HOME, COUNT };


PayloadSdkInterface* my_payload = nullptr;
GamepadController*   my_gamepad = nullptr;

uint16_t _flags        = 0;
bool     _isRecording  = false;
bool     _isTracking   = false;
int      _gimbalMode   = (int)GimbalMode::FOLLOW;
int      _viewSrcIdx   = 0;
int      _recordSrcIdx = 0;

double eo_zoom = 0.0;
double ir_zoom = 0.0;


void onSignal(int) {
    if (my_gamepad) my_gamepad->requestQuit();
}

int applyDeadzone(int value) {
    return (std::abs(value) < JOYSTICK_DEADZONE) ? 0 : value;
}

float mapAxis(int value, float outMin, float outMax) {
    value = std::clamp(value, -32767, 32767);
    return outMin + (value + 32767) * (outMax - outMin) / 65534.0f;
}

void gimbalSetFollow(){
    _flags &= ~GIMBAL_DEVICE_FLAGS_YAW_LOCK;
	_flags &= ~(1 << 14);					// xoa co
    _flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL;	// xoa co
    _flags &= ~GIMBAL_DEVICE_FLAGS_RETRACT; // xoa co
    my_payload->setGimbalMode(_flags);
}

void gimbalSetLock(){
    _flags |= GIMBAL_DEVICE_FLAGS_YAW_LOCK;
    _flags &= ~(1 << 14);					// xoa co
    _flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL;	// xoa co
    _flags &= ~GIMBAL_DEVICE_FLAGS_RETRACT; // xoa co
    my_payload->setGimbalMode(_flags);
}

void gimbalSetMapping(){
    _flags |= (1 << 14);
    _flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL;	// xoa co
    _flags &= ~GIMBAL_DEVICE_FLAGS_RETRACT; // xoa co
    my_payload->setGimbalMode(_flags);
}

static void gimbalSetHome() {
    printf("Set gimbal return HOME\n");
    _flags |=  GIMBAL_DEVICE_FLAGS_NEUTRAL;
    _flags |= GIMBAL_DEVICE_FLAGS_NEUTRAL;
    _flags &= ~GIMBAL_DEVICE_FLAGS_RETRACT; // xoa co
    _flags &= ~(1 << 14);                   // xoa co mapping
    my_payload->setGimbalMode(_flags);
    usleep(100000);
    // reset the control bit
    _flags &= ~GIMBAL_DEVICE_FLAGS_NEUTRAL; // xoa co
    my_payload->setGimbalMode(_flags);
}

static void changeGimbalMode() {
    _gimbalMode = (_gimbalMode + 1) % (int)GimbalMode::COUNT;

    switch ((GimbalMode)_gimbalMode) {
    case GimbalMode::FOLLOW:
        printf("Change gimbal mode to FOLLOW\n");
        gimbalSetFollow();
        break;
    case GimbalMode::LOCK:
        printf("Change gimbal mode to LOCK\n");
        gimbalSetLock();
        break;
    case GimbalMode::MAPPING:
        printf("Change gimbal mode to MAPPING\n");
        gimbalSetMapping();
        break;
    case GimbalMode::HOME:
        printf("Change gimbal mode to HOME\n");
        gimbalSetHome();
        break;
    default:
        break;
    }
}

static void moveGimbal() {
    int p = applyDeadzone(my_gamepad->getAnalog(Action::GIMBAL_PITCH));
    int y = applyDeadzone(my_gamepad->getAnalog(Action::GIMBAL_YAW));
    int r = applyDeadzone(my_gamepad->getAnalog(Action::GIMBAL_ROLL));

    float pitch = mapAxis(p, -GIMBAL_PITCH_SPEED, GIMBAL_PITCH_SPEED);
    float yaw   = mapAxis(y, -GIMBAL_YAW_SPEED,   GIMBAL_YAW_SPEED);
    float roll  = mapAxis(r, -GIMBAL_ROLL_SPEED,  GIMBAL_ROLL_SPEED);

    my_payload->setGimbalSpeed(-pitch, roll, yaw, INPUT_SPEED);
}

static void zoomCamera() {
    int z = applyDeadzone(my_gamepad->getAnalog(Action::ZOOM));
    if      (z < 0) my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_IN);
    else if (z > 0) my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_OUT);
    else            my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_STOP);
}

static void zoomWide(){
    my_payload->setCameraZoom(ZOOM_TYPE_RANGE, 0);
}

static void zoomTele(){
    my_payload->setCameraZoom(ZOOM_TYPE_RANGE, 100.0);
}

static void toggleRecord() {
    _isRecording = !_isRecording;
    printf("%s record\n", _isRecording ? "Start" : "Stop");
    if (_isRecording) 
        my_payload->setPayloadCameraRecordVideoStart();
    else              
        my_payload->setPayloadCameraRecordVideoStop();
}

static void toggleTracking() {
    _isTracking = !_isTracking;
    printf("%s tracking\n", _isTracking ? "Start" : "Stop");
    if (_isTracking) {
        my_payload->setPayloadObjectTrackingMode(TRACK_ACTIVE);
        my_payload->setPayloadObjectTrackingPosition(TRACK_FRAME_W / 2, TRACK_FRAME_H / 2, TRACK_BOX, TRACK_BOX);
    } else {
        my_payload->setPayloadObjectTrackingMode(TRACK_STOP);
    }
}

void changeViewSource() {
    _viewSrcIdx = (_viewSrcIdx + 1) % VIEW_SRC_COUNT;
    int value = VIEW_SRC_VALUES[_viewSrcIdx];
    printf("Set view source to %d\n", value);
    my_payload->setPayloadCameraParam("C_SOURCE", value, PARAM_TYPE_UINT32);
}

void changeRecordSource() {
    _recordSrcIdx = (_recordSrcIdx + 1) % RECORD_SRC_COUNT;
    int value = RECORD_SRC_VALUES[_recordSrcIdx];
    printf("Set record source to %d\n", value);
    my_payload->setPayloadCameraParam("C_V_REC", value, PARAM_TYPE_UINT32);
}

void executeAction(Action a) {
    if (!GamepadController::isAnalog(a))
        printf("%s -> %s\n", GamepadController::actionName(a), my_gamepad->inputNameOf(a));

    switch (a) {
    case Action::GIMBAL_PITCH:
    case Action::GIMBAL_YAW:
    case Action::GIMBAL_ROLL:   
        moveGimbal();          
        break;

    case Action::ZOOM:          
        zoomCamera();          
        break;

    case Action::GIMBAL_HOME:   
        gimbalSetHome();       
        break;

    case Action::GIMBAL_MODE:   
        changeGimbalMode();    
        break;

    case Action::IMAGE_CAPTURE: 
        my_payload->setPayloadCameraCaptureImage(); 
        break;

    case Action::VIDEO_RECORD:  
        toggleRecord();        
        break;

    case Action::CHANGE_VIEW:   
        changeViewSource();    
        break;

    case Action::CHANGE_RECORD:     
        changeRecordSource();  
        break;

    case Action::TRACKING:      
        toggleTracking();      
        break;

    default: break;
    }
}

void onPayloadStatusChanged(int event, double* param) {
    switch (event) {
    case PAYLOAD_GB_ATTITUDE:
        // param[0]: pitch 
        // param[1]: roll 
        // param[2]: yaw

        // printf("Pich: %.2f - Roll: %.2f - Yaw: %.2f\n", param[0], param[1], param[2]);
        break;

    case PAYLOAD_PARAMS:
        if (param[0] == PARAM_EO_ZOOM_LEVEL) {
            eo_zoom = param[1];
            printf("eo_zoom: %.2f\n", eo_zoom);
        } else if (param[0] == PARAM_IR_ZOOM_LEVEL) {
            ir_zoom = param[1];
            printf("ir_zoom: %.2f\n", ir_zoom);
        }
        break;

    default:
        break;
    }
}



int main() {
    printf("Starting Gamepad control app...\n");

    my_gamepad = new GamepadController();
    std::signal(SIGINT,  onSignal);   // Ctrl+C
    std::signal(SIGTERM, onSignal);

    my_payload = new PayloadSdkInterface(s_conn);
    my_payload->sdkInitConnection();
    printf("Waiting for payload signal!\n");
    my_payload->regPayloadStatusChanged(onPayloadStatusChanged);
    my_payload->checkPayloadConnection();
    usleep(100000);

    my_gamepad->setActionCallback(executeAction);
    my_gamepad->start();

    while (!my_gamepad->quitRequested()){
        my_gamepad->update(20);
    }

    my_gamepad->stop();
    my_payload->setGimbalSpeed(0, 0, 0, INPUT_SPEED);
    my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_STOP);
    my_payload->sdkQuit();

    delete my_gamepad;
    printf("Exit\n");
    return 0;
}
