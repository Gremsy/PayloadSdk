#include "payloadSdkInterface.h"

#define SDK_VERSION "2.0.0_build.16082024"

Autopilot_Interface* payload_interface = nullptr;

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


void*
start_thrd_request_msg(void *args)
{
    // takes an smart track object argument
    PayloadSdkInterface *my_payload = (PayloadSdkInterface *)args;

    // run the object's read thread
    my_payload->payload_request_handle();

    // done!
    return NULL;
}

PayloadSdkInterface::PayloadSdkInterface(){
    printf("Starting Gremsy PayloadSdk %s\n", SDK_VERSION);

    // init paramRate
    for(int i = 0; i < PARAM_COUNT; i++){
        paramRate[i] = 0; // 500ms as default
    }
}

PayloadSdkInterface::PayloadSdkInterface(T_ConnInfo data){
    printf("Starting Gremsy PayloadSdk %s\n", SDK_VERSION);
    payload_ctrl_type = data.type;
    if(payload_ctrl_type == CONTROL_UART){
        payload_uart_port = (char*)data.device.uart.name;
        payload_uart_baud = data.device.uart.baudrate;
    }else if(payload_ctrl_type == CONTROL_UDP){
        udp_ip_target = (char*)data.device.udp.ip;
        udp_port_target = data.device.udp.port;
    }

    // init paramRate
    for(int i = 0; i < PARAM_COUNT; i++){
        paramRate[i] = 0; // 500ms as default
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
        printf("Please define your control method first. See payloadsdk.h\n");
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
        printf("Open Serial Port Error\r\n");
        return false;
    }

    // init thread to check receive message from payload
    all_threads_init();

    return true;
}

void
PayloadSdkInterface::
sdkQuit(){
    if(port_quit != nullptr){
        port_quit->stop();
    }
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

    rc = pthread_create(&thrd_request_params, NULL, &start_thrd_request_msg, this);
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
    while(!time_to_exit){
        mavlink_message_t msg;
        uint8_t msg_cnt = getNewMewssage(msg);

        if(msg_cnt && msg.sysid == PAYLOAD_SYSTEM_ID && msg.compid == PAYLOAD_COMPONENT_ID){
            printf("Payload connected! \n");
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

    strcpy((char *)msg.param_id, param_id);

    cam_param_union_t u;
    u.param_uint32 = param_value;
    std::string str(reinterpret_cast<char const *>(u.bytes), CAM_PARAM_VALUE_LEN);
    strcpy(msg.param_value, str.c_str());

    msg.param_type = param_type;
    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;

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

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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
getPayloadStorage(){
    mavlink_command_long_t msg = {0};

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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
setPayloadCameraMode(CAMERA_MODE mode){
    mavlink_command_long_t msg = {0};

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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
setPayloadCameraCaptureImage(int interval_s){
    mavlink_command_long_t msg = {0};

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
    msg.command = MAV_CMD_IMAGE_START_CAPTURE;
    msg.param2 = (int)interval_s;
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

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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

void
PayloadSdkInterface::
requestParamValue(uint8_t pIndex){
    // printf("%s \n", __func__);

    mavlink_param_request_read_t request = {0};

    request.target_system = SYS_ID_USER2;
    request.target_component = MAV_COMP_ID_USER2;
    request.param_index = pIndex;

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
    paramRate[pIndex] = time_ms;
}

void
PayloadSdkInterface::
setGimbalSpeed(float spd_pitch, float spd_roll, float spd_yaw, input_mode_t mode){

    /* Pack message */
    mavlink_gimbal_device_set_attitude_t attitude = { 0 };
    attitude.target_system    = GIMBAL_SYSTEM_ID;
    attitude.target_component = GIMBAL_COMPONENT_ID;
    if (mode == INPUT_ANGLE) {
        /* Convert target to quaternion */
        if (spd_yaw > 180.f || spd_yaw < -180.f)
        {
            printf("ERROR: Gimbal Protocol V2 only supports yaw axis from -180 degrees to 180 degrees!\n");
            return;
        }
        if (spd_roll > 180.f || spd_roll < -180.f)
        {
            printf("ERROR: Gimbal Protocol V2 only supports roll axis from -180 degrees to 180 degrees!\n");
            return;
        }
        if(spd_pitch > 90.f || spd_pitch < -90.f){
            printf("ERROR: Gimbal Protocol V2 only supports roll axis from -90 degrees to 90 degrees!\n");
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
sendDroneAttitude(mavlink_attitude_t att){

    // This function will send the autopilot attitude mesage to the payload
    // Before sending thí mesage, please make sure that your system does not have any MAV_COMP_ID_AUTOPILOT1 to avoid any conflict
    // The attitude message need to be send from MAV_COMP_ID_AUTOPILOT1, so we will create a dummy deivce to send this message.
    
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message = {0};
    mavlink_msg_attitude_encode(SYS_ID, MAV_COMP_ID_AUTOPILOT1, &message, &att);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    payload_interface->push_message_to_queue(message);

    printf("%s, from %d %d \n", __func__, SYS_ID, COMP_ID);
}

void
PayloadSdkInterface::
setCameraZoom(float zoomType,float zoomValue)
{
    mavlink_command_long_t msg = {0};

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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

    msg.target_system = PAYLOAD_SYSTEM_ID;
    msg.target_component = PAYLOAD_COMPONENT_ID;
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
setPayloadObjectTrackingParams(float cmd, float pos_x, float pos_y){
    mavlink_command_long_t msg = {0};

    msg.target_system = SYS_ID_USER2;
    msg.target_component = MAV_COMP_ID_USER2;
    msg.command = MAV_CMD_USER_4;
    msg.param1 = 1;
    msg.param2 = 1;
    msg.param3 = 0;
    msg.param4 = cmd;
    msg.param6 = pos_x;
    msg.param7 = pos_y;
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

    printf("%s %.2f %.2f %.2f \n", __func__, cmd, pos_x, pos_y);
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
payload_recv_handle()
{
    // check payload messages
    while(!time_to_exit){
        mavlink_message_t msg;
        uint8_t msg_cnt = getNewMewssage(msg);
        if(msg_cnt){
            // printf("Got %d message in queue \n", msg_cnt);
            // printf("   --> message %d from system_id: %d with component_id: %d \n", msg.msgid, msg.sysid, msg.compid);
            if(msg.compid == MAV_COMP_ID_USER2)
                SYS_ID_USER2 = msg.sysid;

            switch(msg.msgid){
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
            default: break;
            }
        }else{
        }
        usleep(100);
    }
}

void
PayloadSdkInterface::
payload_request_handle(){
    uint8_t param_id = 0;
    uint64_t tick_cnt = 0;

    while(!time_to_exit){
        tick_cnt++;
        for(int i = 0; i < PARAM_COUNT; i++){
            if(paramRate[i] > 0){
                if(!(tick_cnt % paramRate[i]))
                    requestParamValue(i);
            }
        }
        usleep(1000); // 1ms
    }
}

void
PayloadSdkInterface::
_handle_msg_param_ext_value(mavlink_message_t* msg){
    // printf("%s msg_id %d \n", __func__, msg->msgid);
    mavlink_param_ext_value_t param_ext_value = {0};
    mavlink_msg_param_ext_value_decode(msg, &param_ext_value);

    uint32_t param_uint32;
    memcpy(&param_uint32, param_ext_value.param_value, sizeof(param_uint32));

    if(__notifyPayloadParamChanged != NULL){
        double params[2] = {param_ext_value.param_index, param_uint32};
        __notifyPayloadParamChanged(PAYLOAD_CAM_PARAM_VALUE, param_ext_value.param_id, params);
    }
}

void
PayloadSdkInterface::
_handle_msg_command_ack(mavlink_message_t* msg){
    mavlink_command_ack_t cmd_ack = {0};
    mavlink_msg_command_ack_decode(msg, &cmd_ack);

    // printf("Got ACK for command %d with status %d\n", cmd_ack.command, cmd_ack.result);
    if(__notifyPayloadStatusChanged != NULL){
        double params[2] = {cmd_ack.command, cmd_ack.result};
        __notifyPayloadStatusChanged(PAYLOAD_GB_ACK, params);
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
        double params[3] = {packet.pitch, packet.roll, packet.yaw};
        __notifyPayloadStatusChanged(PAYLOAD_GB_ATTITUDE, params);
    }
}

void
PayloadSdkInterface::
_handle_msg_param_value(mavlink_message_t* msg){
    mavlink_param_value_t value = {0};
    mavlink_msg_param_value_decode(msg, &value);

    if(__notifyPayloadStatusChanged != NULL){
        double params[2] = {value.param_index, value.param_value};
        __notifyPayloadStatusChanged(PAYLOAD_PARAMS, params);
    }
}