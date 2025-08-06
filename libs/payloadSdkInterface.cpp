#include "payloadSdkInterface.h"

#define SDK_VERSION "3.0.0_build.04022025"

void*
start_thrd_received_msg(void *args)
{
    // takes an smart track object argument
    PayloadSdkInterface *my_payload = (PayloadSdkInterface *)args;

    // run the object's read thread
    my_payload->payload_recv_handle();

    // done!
    return NULL;
}

PayloadSdkInterface::PayloadSdkInterface(){
    SDK_LOG("Starting Gremsy PayloadSdk %s", SDK_VERSION);
}

PayloadSdkInterface::PayloadSdkInterface(T_ConnInfo data){
    SDK_LOG("Starting Gremsy PayloadSdk %s", SDK_VERSION);
    payload_ctrl_type = data.type;
    if(payload_ctrl_type == CONTROL_UART){
        payload_uart_port = (char*)data.device.uart.name;
        payload_uart_baud = data.device.uart.baudrate;
    }else if(payload_ctrl_type == CONTROL_UDP){
        udp_ip_target = (char*)data.device.udp.ip;
        udp_port_target = data.device.udp.port;
    }
}

PayloadSdkInterface::~PayloadSdkInterface(){
}

void
PayloadSdkInterface::
regPayloadStatusChanged(payload_status_callback_t func){
    __notifyPayloadStatusChanged = func;
}

void
PayloadSdkInterface::
regPayloadParamChanged(payload_param_callback_t func){
    __notifyPayloadParamChanged = func;
}

void
PayloadSdkInterface::
regPayloadStreamChanged(payload_streamInfo_callback_t func){
    __notifyPayloadStreamChanged = func;
}

bool 
PayloadSdkInterface::
sdkInitConnection(){
    /* Port for connect with payload */
    if(payload_ctrl_type == CONTROL_UART){
        port = new Serial_Port(payload_uart_port, payload_uart_baud);
    }else if(payload_ctrl_type == CONTROL_UDP)
        port = new UDP_Port(udp_ip_target, udp_port_target);
    else{
        SDK_LOG("Please define your control method first. See payloadsdk.h");
        return false;
    }
    /* Instantiate an gimbal interface object */
    payload_interface = new Autopilot_Interface(port, SYS_ID, COMP_ID, 2, MAVLINK_COMM_1);

    // quit port will close at terminator event
    port_quit        = port;

    /* Start the port and payload_interface */
    try{
        port->start();
        payload_interface->start();
    }catch(...){
        SDK_LOG("Open Serial Port Error\r");
        return false;
    }

    // init thread to check receive message from payload
    all_threads_init();
    
    usleep(1000000);
    
    return true;
}

void
PayloadSdkInterface::
sdkQuit(){
    if(port_quit != nullptr){
        port_quit->stop();
    }
    payload_interface->handle_quit(0);
    time_to_exit = true;
}

bool 
PayloadSdkInterface::
all_threads_init(){
    int rc = pthread_create(&thrd_recv, NULL, &start_thrd_received_msg, this);
    if (rc){
        std::cout << "\nError: Can not create thread!" << rc << std::endl;
        return false;
    }
    std::cout << "Thread created\n" << std::endl;

    return true;
}

void
PayloadSdkInterface::
checkPayloadConnection(){
    bool result = false;

    while(!time_to_exit){
        mavlink_message_t msg;
        uint8_t msg_cnt = getNewMewssage(msg);

        result = false;

        if(msg_cnt){ // got the message

            if(msg.compid >= MAV_COMP_ID_CAMERA
                && msg.compid <= MAV_COMP_ID_CAMERA6){ // got the message from the camera
                // update the camera id
                CAMERA_SYSTEM_ID = msg.sysid;
                CAMERA_COMPONENT_ID = msg.compid;

                result = true;
            }
            if(msg.compid == MAV_COMP_ID_GIMBAL
                || msg.compid == MAV_COMP_ID_GIMBAL2
                || msg.compid == MAV_COMP_ID_GIMBAL3
                || msg.compid == MAV_COMP_ID_GIMBAL4
                || msg.compid == MAV_COMP_ID_GIMBAL5
                || msg.compid == MAV_COMP_ID_GIMBAL6){ // got the message from the camera
                
                // update the gimbal id
                GIMBAL_SYSTEM_ID = msg.sysid;
                GIMBAL_COMPONENT_ID = msg.compid;

                result = true;
            }
        } 

        if(result){
            SDK_LOG("Payload connected! ");
            break;
        }
        
    }
}

uint8_t 
PayloadSdkInterface::
getNewMewssage(mavlink_message_t& new_msg){
    if(payload_interface != nullptr){
        return payload_interface->get_nxt_message(new_msg);
    }
    return 0;
}

void
PayloadSdkInterface::
setPayloadCameraParam(char param_id[], uint32_t param_value, uint8_t param_type){
    mavlink_param_ext_set_t msg={0};

    current_gimbal_mode = param_value;

    strcpy((char *)msg.param_id, param_id);

    cam_param_union_t u;
    u.param_uint32 = param_value;
    std::string str(reinterpret_cast<char const *>(u.bytes), CAM_PARAM_VALUE_LEN);
    strcpy(msg.param_value, str.c_str());

    msg.param_type = param_type;
    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_param_ext_set_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}


void
PayloadSdkInterface::
getPayloadCameraSettingList(){
    mavlink_param_ext_request_list_t msg= {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    // msg.trimmed = 0;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_param_ext_request_list_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
getPayloadCameraSettingByID(char* ID){
    mavlink_param_ext_request_read_t msg= {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.param_index = -1;
    strncpy(msg.param_id, ID, 16);

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_param_ext_request_read_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
getPayloadCameraSettingByIndex(uint8_t idx){
    mavlink_param_ext_request_read_t msg= {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    strncpy(msg.param_id, "", 16);
    msg.param_index = idx;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_param_ext_request_read_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
getPayloadStorage(){
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_REQUEST_STORAGE_INFORMATION;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
getPayloadCaptureStatus(){
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
getPayloadCameraMode(){
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_REQUEST_CAMERA_SETTINGS;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
getPayloadCameraInformation(){
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_REQUEST_CAMERA_INFORMATION;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    message.sysid = SYS_ID;
    message.compid = COMP_ID;

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
getPayloadCameraStreamingInformation(){
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
setPayloadGimbalParamByID(char* param_id, float param_value){
    mavlink_param_set_t msg={0};

    strcpy((char *)msg.param_id, param_id);
    msg.param_value = param_value;

    // msg.param_type = param_type;
    msg.target_system = GIMBAL_SYSTEM_ID;
    msg.target_component = GIMBAL_COMPONENT_ID;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_param_set_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
sendPayloadGimbalCalibGyro(){
    mavlink_command_long_t msg = {0};

    msg.target_system = GIMBAL_SYSTEM_ID;
    msg.target_component = GIMBAL_COMPONENT_ID;
    msg.command = MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
    msg.confirmation = 1;

    msg.param7 = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    message.sysid = SYS_ID;
    message.compid = COMP_ID;

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
sendPayloadGimbalCalibAccel(){
    mavlink_command_long_t msg = {0};

    msg.target_system = GIMBAL_SYSTEM_ID;
    msg.target_component = GIMBAL_COMPONENT_ID;
    msg.command = MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
    msg.confirmation = 1;

    msg.param7 = 2;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    message.sysid = SYS_ID;
    message.compid = COMP_ID;

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
sendPayloadGimbalCalibMotor(){
    mavlink_command_long_t msg = {0};

    msg.target_system = GIMBAL_SYSTEM_ID;
    msg.target_component = GIMBAL_COMPONENT_ID;
    msg.command = MAV_CMD_DO_SET_HOME;
    msg.confirmation = 1;

    msg.param7 = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    message.sysid = SYS_ID;
    message.compid = COMP_ID;

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
sendPayloadGimbalSearchHome(){
    mavlink_command_long_t msg = {0};

    msg.target_system = GIMBAL_SYSTEM_ID;
    msg.target_component = GIMBAL_COMPONENT_ID;
    msg.command = MAV_CMD_DO_SET_HOME;
    msg.confirmation = 1;

    msg.param7 = 2;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    message.sysid = SYS_ID;
    message.compid = COMP_ID;

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
sendPayloadGimbalAutoTune(bool status){
    mavlink_command_long_t msg = {0};

    msg.target_system = GIMBAL_SYSTEM_ID;
    msg.target_component = GIMBAL_COMPONENT_ID;
    msg.command = MAV_CMD_USER_3;
    msg.confirmation = 1;

    msg.param7 = status ? 1 : 0;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    message.sysid = SYS_ID;
    message.compid = COMP_ID;

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
getPayloadGimbalSettingList(){
    mavlink_param_request_list_t msg= {0};

    msg.target_system = GIMBAL_SYSTEM_ID;
    msg.target_component = GIMBAL_COMPONENT_ID;
    // msg.trimmed = 0;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_param_request_list_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
getPayloadGimbalSettingByID(char* ID){
    mavlink_param_request_read_t msg= {0};

    msg.target_system = GIMBAL_SYSTEM_ID;
    msg.target_component = GIMBAL_COMPONENT_ID;
    msg.param_index = -1;
    strncpy(msg.param_id, ID, 16);

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_param_request_read_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
getPayloadGimbalSettingByIndex(uint8_t idx){
    mavlink_param_request_read_t msg= {0};

    msg.target_system = GIMBAL_SYSTEM_ID;
    msg.target_component = GIMBAL_COMPONENT_ID;
    strncpy(msg.param_id, "", 16);
    msg.param_index = idx;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_param_request_read_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
setPayloadCameraMode(CAMERA_MODE mode){
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_SET_CAMERA_MODE;
    msg.param2 = (uint32_t)mode;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
setPayloadCameraCaptureImage(float interval_s){
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_IMAGE_START_CAPTURE;
    msg.param2 = (float)interval_s;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
setPayloadCameraStopImage(){
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_IMAGE_STOP_CAPTURE;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
setPayloadCameraRecordVideoStart(){
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_VIDEO_START_CAPTURE;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
setPayloadCameraRecordVideoStop(){
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_VIDEO_STOP_CAPTURE;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

/**
 * Set IR FFC mode
 * Mode: Manual: 0, Auto: 1
 **/
void 
PayloadSdkInterface::
setPayloadCameraFFCMode(uint8_t mode){
    if(mode < 0 || mode >= FFC_MODE_END) return;

    mavlink_command_long_t msg = {0};

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
    msg.command = MAV_CMD_USER_4;
    msg.param1 = 2;
    msg.param2 = 6;
    msg.param3 = mode;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

/**
 * Get IR FFC mode
 **/
void 
PayloadSdkInterface::
getPayloadCameraFFCMode(uint8_t& mode){

}

/**
 * Set IR FFC trigger
 **/
void 
PayloadSdkInterface::
setPayloadCameraFFCTrigg(){
    mavlink_command_long_t msg = {0};

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
    msg.command = MAV_CMD_USER_4;
    msg.param1 = 2;
    msg.param2 = 7;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
setPayloadStreamBitrate(uint32_t bitrate){
    mavlink_command_long_t msg = {0};

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
    msg.command = MAV_CMD_USER_4;
    msg.param1 = 4;
    msg.param2 = 2;
    msg.param3 = bitrate;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

uint32_t 
PayloadSdkInterface::
getPayloadStreamBitrate(){

}

void
PayloadSdkInterface::
requestParamValue(uint8_t pIndex){
    // SDK_LOG("%s ", __func__);

    mavlink_param_request_read_t request = {0};

    request.target_system = PAYLOAD_SYSTEM_ID;
    request.target_component = PAYLOAD_COMPONENT_ID;
    request.param_index = pIndex;
    strncpy(request.param_id, payloadParams[pIndex].id, 16);

    // printf("%s, for index: %d, id: %s\n", __func__, pIndex, payloadParams[pIndex].id);

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_param_request_read_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &request);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
setParamRate(uint8_t pIndex, uint16_t time_ms){
    payloadParams[pIndex].msg_rate = time_ms;
    sendPayloadRequestStreamRate(pIndex, time_ms);
}

void 
PayloadSdkInterface::
requestMessageStreamInterval(){
    for(uint8_t i =0; i < PARAM_COUNT; i++){
        if(payloadParams[i].msg_rate >= 0){
            // SDK_LOG("msd_id %d, interval %ld, send to %d, %d", i, payloadParams[i].msg_rate, PAYLOAD_SYSTEM_ID, PAYLOAD_COMPONENT_ID);

            sendPayloadRequestStreamRate(i, payloadParams[i].msg_rate);
        }
    }
}

void
PayloadSdkInterface::
setGimbalSpeed(float spd_pitch, float spd_roll, float spd_yaw, input_mode_t mode){

    /* Pack message */
    mavlink_gimbal_device_set_attitude_t attitude = { 0 };
    attitude.target_system    = GIMBAL_SYSTEM_ID;
    attitude.target_component = GIMBAL_COMPONENT_ID;


    switch (current_gimbal_mode){
        case(PAYLOAD_CAMERA_GIMBAL_MODE_OFF):
            attitude.flags = current_attitude_flags | GIMBAL_DEVICE_FLAGS_RETRACT;
            break;
        case(PAYLOAD_CAMERA_GIMBAL_MODE_RESET):
            attitude.flags = current_attitude_flags | GIMBAL_DEVICE_FLAGS_NEUTRAL;
            break;
        case(PAYLOAD_CAMERA_GIMBAL_MODE_LOCK):
            attitude.flags = current_attitude_flags | GIMBAL_DEVICE_FLAGS_YAW_LOCK;
            break;
        case(PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW):
            attitude.flags = current_attitude_flags & (~GIMBAL_DEVICE_FLAGS_YAW_LOCK);
            break;
        case(PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING):
            #define MESSAGE_FLAG_MAPPING 0x4000
            attitude.flags = current_attitude_flags | MESSAGE_FLAG_MAPPING;
            break;
        default: break;
    }    

    if (mode == INPUT_ANGLE) {
        /* Convert target to quaternion */
        if (spd_yaw > 180.f || spd_yaw < -180.f)
        {
            SDK_LOG("ERROR: Gimbal Protocol V2 only supports yaw axis from -180 degrees to 180 degrees!");
            return;
        }
        if (spd_roll > 180.f || spd_roll < -180.f)
        {
            SDK_LOG("ERROR: Gimbal Protocol V2 only supports roll axis from -180 degrees to 180 degrees!");
            return;
        }
        if(spd_pitch > 90.f || spd_pitch < -90.f){
            SDK_LOG("ERROR: Gimbal Protocol V2 only supports roll axis from -90 degrees to 90 degrees!");
            return;
        }
        mavlink_euler_to_quaternion(to_rad(spd_roll), to_rad(spd_pitch), to_rad(spd_yaw), attitude.q);
        attitude.angular_velocity_x = NAN;
        attitude.angular_velocity_y = NAN;
        attitude.angular_velocity_z = NAN;

    } else {
        attitude.angular_velocity_x = to_rad(spd_roll);
        attitude.angular_velocity_y = to_rad(spd_pitch);
        attitude.angular_velocity_z = to_rad(spd_yaw);
        attitude.q[0] = NAN;
        attitude.q[1] = NAN;
        attitude.q[2] = NAN;
        attitude.q[3] = NAN;
    }

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message = { 0 };
    mavlink_msg_gimbal_device_set_attitude_encode(SYS_ID, COMP_ID, &message, &attitude);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
setCameraZoom(float zoomType,float zoomValue)
{
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_SET_CAMERA_ZOOM;
    msg.param1 = (float)zoomType;
    msg.param2 = (float)zoomValue;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
setCameraFocus(float focusType, float focusValue)
{
    mavlink_command_long_t msg = {0};

    msg.target_system = CAMERA_SYSTEM_ID;
    msg.target_component = CAMERA_COMPONENT_ID;
    msg.command = MAV_CMD_SET_CAMERA_FOCUS;
    msg.param1 = (float)focusType;
    msg.param2 = (float)focusValue;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
setPayloadObjectTrackingMode(float mode){
    mavlink_command_long_t msg = {0};

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
    msg.command = MAV_CMD_USER_4;
    msg.param1 = 4;
    msg.param2 = 0;
    msg.param3 = 0;
    msg.param4 = mode;
    msg.param5 = 0;
    msg.param6 = 0;
    msg.param7 = 0;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);

    SDK_LOG("%s %.f ", __func__, mode);
}

void 
PayloadSdkInterface::
setPayloadObjectTrackingPosition(float pos_x, float pos_y, float width, float height){
    mavlink_command_long_t msg = {0};

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
    msg.command = MAV_CMD_USER_4;
    msg.param1 = 4;
    msg.param2 = 0;
    msg.param3 = 1;
    msg.param4 = pos_x;
    msg.param5 = pos_y;
    msg.param6 = width;
    msg.param7 = height;
    msg.confirmation = 1;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);

    SDK_LOG("%s, %.2f %.2f %.2f %.2f ", __func__, pos_x, pos_y, width, height);
}

void 
PayloadSdkInterface::
sendPayloadGPSPosition(mavlink_global_position_int_t gps){
    
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_global_position_int_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &gps);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
sendPayloadGPSRawInt(mavlink_gps_raw_int_t gps_raw){
    
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_gps_raw_int_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &gps_raw);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
sendPayloadSystemTime(mavlink_system_time_t sys_time){
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_system_time_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &sys_time);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
sendPayloadRequestStreamRate(int index, uint16_t time_ms){
    mavlink_command_long_t cmd{0};

    cmd.target_system = PAYLOAD_SYSTEM_ID;
    cmd.target_component = PAYLOAD_COMPONENT_ID;

    cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd.param1 = index;
    cmd.param2 = time_ms * 1000; // interval
    cmd.param7 = 1; // Response to requestor

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;
    mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &cmd);

    // do the write
    payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
payload_recv_handle()
{
    // check payload messages
    while(!time_to_exit){
        mavlink_message_t msg;
        uint8_t msg_cnt = getNewMewssage(msg);
        if(msg_cnt){
            // SDK_LOG("Got %d message in queue ", msg_cnt);
            // SDK_LOG("   --> message %d from system_id: %d with component_id: %d ", msg.msgid, msg.sysid, msg.compid);
            if(msg.compid == PAYLOAD_COMPONENT_ID){
                PAYLOAD_SYSTEM_ID = msg.sysid;

                if(!is_send_stream_request){
                    // only need to send 1 time, after get the sys_id of the payload
                    requestMessageStreamInterval();
                    is_send_stream_request = true;
                }
            }

            // update gimbal id
            if(msg.compid == MAV_COMP_ID_GIMBAL
                || msg.compid == MAV_COMP_ID_GIMBAL2
                || msg.compid == MAV_COMP_ID_GIMBAL3
                || msg.compid == MAV_COMP_ID_GIMBAL4
                || msg.compid == MAV_COMP_ID_GIMBAL5
                || msg.compid == MAV_COMP_ID_GIMBAL6
                )
            {
                GIMBAL_SYSTEM_ID = msg.sysid;
                GIMBAL_COMPONENT_ID = msg.compid;
            }

            // update payload camera id
            if(msg.compid >= MAV_COMP_ID_CAMERA
                && msg.compid <= MAV_COMP_ID_CAMERA6)
            {
                CAMERA_SYSTEM_ID = msg.sysid;
                CAMERA_COMPONENT_ID = msg.compid;
            }

            switch(msg.msgid){
            case MAVLINK_MSG_ID_HEARTBEAT:{
                // SDK_LOG("Got hearbeat, from %d, seq %d", msg.compid, msg.seq);

                break;
            }
            case MAVLINK_MSG_ID_PARAM_EXT_VALUE:{
                _handle_msg_param_ext_value(&msg);
                break;
            }
            case MAVLINK_MSG_ID_COMMAND_ACK:{
                _handle_msg_command_ack(&msg);
                break;
            }
            case MAVLINK_MSG_ID_CAMERA_INFORMATION:{
                _handle_msg_camera_information(&msg);
                break;
            }
            case MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION:{
                _handle_msg_camera_stream_information(&msg);
                break;
            }
            case MAVLINK_MSG_ID_STORAGE_INFORMATION:{
                _handle_msg_storage_information(&msg);
                break;
            }
            case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:{
                _handle_msg_camera_capture_status(&msg);
                break;
            }
            case MAVLINK_MSG_ID_CAMERA_SETTINGS:{
                _handle_msg_camera_settings(&msg);
                break;
            }

            case MAVLINK_MSG_ID_MOUNT_ORIENTATION:{
                _handle_msg_mount_orientation(&msg);
                break;
            }
            case MAVLINK_MSG_ID_PARAM_VALUE:{
                _handle_msg_param_value(&msg);
                break;
            }
            case MAVLINK_MSG_ID_DEBUG:{
                // for update params from the payload
                _handle_msg_debug(&msg);
                break;
            }
            case MAVLINK_MSG_ID_PARAM_EXT_ACK:{
                _handle_msg_command_ext_ack(&msg);
                break;
            }
            case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:{
                _handle_msg_device_attitude(&msg);
                break;  
            }            
            default: break;
            }
        }else{
        }
        usleep(100);
    }
}

void
PayloadSdkInterface::
_handle_msg_param_ext_value(mavlink_message_t* msg){
    // SDK_LOG("%s msg_id %d ", __func__, msg->msgid);
    mavlink_param_ext_value_t param_ext_value = {0};
    mavlink_msg_param_ext_value_decode(msg, &param_ext_value);

    uint32_t param_uint32;
    memcpy(&param_uint32, param_ext_value.param_value, sizeof(param_uint32));

    if(__notifyPayloadParamChanged != NULL){
        double params[2] = {param_ext_value.param_index, param_uint32};
        __notifyPayloadParamChanged(PAYLOAD_CAM_PARAMS, param_ext_value.param_id, params);
    }
}

void 
PayloadSdkInterface::
_handle_msg_command_ext_ack(mavlink_message_t* msg){
    mavlink_param_ext_ack_t ext_ack = {0};
    mavlink_msg_param_ext_ack_decode(msg, &ext_ack);

    // SDK_LOG("Got ext_ack for param_id:%s with result:%d", ext_ack.param_id, ext_ack.param_result);
    if(__notifyPayloadStatusChanged != NULL){
        double params[1] = {ext_ack.param_result};
        __notifyPayloadStatusChanged(PAYLOAD_PARAM_EXT_ACK, params);
    }
}

void
PayloadSdkInterface::
_handle_msg_command_ack(mavlink_message_t* msg){
    mavlink_command_ack_t cmd_ack = {0};
    mavlink_msg_command_ack_decode(msg, &cmd_ack);

    // SDK_LOG("Got ACK for command %d with status %d", cmd_ack.command, cmd_ack.result);
    if(__notifyPayloadStatusChanged != NULL){
        double params[3] = {cmd_ack.command, cmd_ack.result, cmd_ack.progress};
        __notifyPayloadStatusChanged(PAYLOAD_ACK, params);
    }
}

void
PayloadSdkInterface::
_handle_msg_camera_information(mavlink_message_t* msg){
    mavlink_camera_information_t camera_info = {0};
    mavlink_msg_camera_information_decode(msg, &camera_info);

    if(__notifyPayloadStatusChanged != NULL){
        double params[1] = {camera_info.flags};
        __notifyPayloadStatusChanged(PAYLOAD_CAM_INFO, params);
    }
}

void
PayloadSdkInterface::
_handle_msg_camera_stream_information(mavlink_message_t* msg){
    mavlink_video_stream_information_t stream_info = {0};
    mavlink_msg_video_stream_information_decode(msg, &stream_info);

    if(__notifyPayloadStreamChanged != NULL){
        double params[3] = {stream_info.type, stream_info.resolution_v, stream_info.resolution_h};

        __notifyPayloadStreamChanged(PAYLOAD_CAM_STREAMINFO, stream_info.uri, params);
    }
}

void
PayloadSdkInterface::
_handle_msg_storage_information(mavlink_message_t* msg){
    mavlink_storage_information_t storage_info = {0};
    mavlink_msg_storage_information_decode(msg, &storage_info);

    if(__notifyPayloadStatusChanged != NULL){
        double params[4] = {storage_info.total_capacity, storage_info.used_capacity, storage_info.available_capacity, storage_info.status};
        __notifyPayloadStatusChanged(PAYLOAD_CAM_STORAGE_INFO, params);
    }
}

void
PayloadSdkInterface::
_handle_msg_camera_capture_status(mavlink_message_t* msg){
    mavlink_camera_capture_status_t capture_status = {0};
    mavlink_msg_camera_capture_status_decode(msg, &capture_status);

    if(__notifyPayloadStatusChanged != NULL){
        double params[4] = {capture_status.image_status, capture_status.video_status, capture_status.image_count, capture_status.recording_time_ms};
        __notifyPayloadStatusChanged(PAYLOAD_CAM_CAPTURE_STATUS, params);
    }
}

void
PayloadSdkInterface::
_handle_msg_camera_settings(mavlink_message_t* msg){
    mavlink_camera_settings_t camera_setting = {0};
    mavlink_msg_camera_settings_decode(msg, &camera_setting);

    if(__notifyPayloadStatusChanged != NULL){
        double params[3] = {camera_setting.mode_id, camera_setting.zoomLevel, camera_setting.focusLevel};
        __notifyPayloadStatusChanged(PAYLOAD_CAM_SETTINGS, params);
    }
}

void
PayloadSdkInterface::
_handle_msg_mount_orientation(mavlink_message_t* msg){
    mavlink_mount_orientation_t packet;
    mavlink_msg_mount_orientation_decode(msg, &packet);

    if(__notifyPayloadStatusChanged != NULL){
        double pitch_ = packet.pitch;
        double roll_ = packet.roll;
        double yaw_ = (current_gimbal_mode == PAYLOAD_CAMERA_GIMBAL_MODE_LOCK) ? packet.yaw_absolute : packet.yaw;

        double params[3] = {pitch_, roll_, yaw_};
        __notifyPayloadStatusChanged(PAYLOAD_GB_ATTITUDE, params);
    }
}

void
PayloadSdkInterface::
_handle_msg_param_value(mavlink_message_t* msg){
    mavlink_param_value_t value = {0};
    mavlink_msg_param_value_decode(msg, &value);
    if(msg->compid == PAYLOAD_COMPONENT_ID){
        // params value from the payload
        if(__notifyPayloadStatusChanged != NULL){
            double params[2] = {value.param_index, value.param_value};
            __notifyPayloadStatusChanged(PAYLOAD_PARAMS, params);
        }
    }
    else if(msg->compid == GIMBAL_COMPONENT_ID){
        // params value from the gimbal
        if(__notifyPayloadParamChanged != NULL){
            double params[2] = {value.param_index, value.param_value};
            __notifyPayloadParamChanged(PAYLOAD_GB_PARAMS, value.param_id, params);
        }
    }
}

void 
PayloadSdkInterface::
_handle_msg_debug(mavlink_message_t* msg){
    mavlink_debug_t value = {0};
    mavlink_msg_debug_decode(msg, &value);

    if(msg->compid == PAYLOAD_COMPONENT_ID){
        // params value from the payload
        if(__notifyPayloadStatusChanged != NULL){
            double params[2] = {value.ind, value.value};
            __notifyPayloadStatusChanged(PAYLOAD_PARAMS, params);
        }
    }
    else if(msg->compid == GIMBAL_COMPONENT_ID){
        // params value from the gimbal
        if(__notifyPayloadParamChanged != NULL){
            double params[2] = {value.ind, value.value};
            __notifyPayloadParamChanged(PAYLOAD_GB_PARAMS, "", params);
        }
    }
}

void
PayloadSdkInterface::
_handle_msg_device_attitude(mavlink_message_t* msg)
{
    mavlink_gimbal_device_attitude_status_t attitude = {0};
    mavlink_msg_gimbal_device_attitude_status_decode(msg, &attitude);

    if(__notifyPayloadParamChanged != NULL)
    {
        double param[6];
        char param_mode[16];
        float roll, pitch, yaw;
        mavlink_quaternion_to_euler(attitude.q, &roll, &pitch, &yaw);

        param[0] = to_deg(pitch);
        param[1] = to_deg(roll);
        param[2] = to_deg(yaw);
        param[3] = attitude.angular_velocity_x;
        param[4] = attitude.angular_velocity_y;
        param[5] = attitude.angular_velocity_z;

        // Save the current attitude flag 
        current_attitude_flags = attitude.flags;

        if(current_attitude_flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK)
        {
            strcpy(param_mode, "LOCK_MODE");
        }
        else if(current_attitude_flags & GIMBAL_DEVICE_FLAGS_RETRACT)
        {
            strcpy(param_mode, "OFF_MODE");
        }
        else if(current_attitude_flags & GIMBAL_DEVICE_FLAGS_NEUTRAL)
        {
            strcpy(param_mode, "RESET_MODE");
        }
        else if(current_attitude_flags & 0x4000)
        {
            strcpy(param_mode, "MAPPING_MODE");
        }
        else 
        {   
            strcpy(param_mode, "FOLLOW_MODE");
        }            
        __notifyPayloadParamChanged(PAYLOAD_GB_ATTITUDE, param_mode, (double *)param);
    }
}