#include "payloadSdkInterface.h"

PayloadSdkInterface::PayloadSdkInterface(){
	printf("Starting Gremsy PayloadSdk %s\n", SDK_VERSION);
}

PayloadSdkInterface::~PayloadSdkInterface(){

}

bool 
PayloadSdkInterface::
sdkInitConnection(){
	/* Port for connect with payload */
#if(CONTROL_METHOD == CONTROL_UART)
    port = new Serial_Port(payload_uart_port, payload_uart_baud);
#elif(CONTROL_METHOD == CONTROL_UDP)
    port = new UDP_Port(udp_ip_target, udp_port);
#else
    printf("Please define your control method first. See payloadsdk.h\n");
    exit(0);
#endif
    
    /* Instantiate an gimbal interface object */
    payload_interface = new Autopilot_Interface(port, SYS_ID, COMP_ID, 2, MAVLINK_COMM_0);


    // quit port will close at terminator event
    port_quit        = port;


    /* Start the port and payload_interface */
    port->start();
    payload_interface->start();
}

void 
PayloadSdkInterface::
sdkQuit(){
	if(port_quit != nullptr){
		port_quit->stop();
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
moveGimbal(float pitch_spd, float yaw_spd){
	// send Do_MOUNT_CONTROL message

	mavlink_command_long_t msg ={0};


	msg.command = MAV_CMD_DO_MOUNT_CONTROL;
	msg.param1 = pitch_spd;
	msg.param2 = 0;
	msg.param3 = yaw_spd;
	msg.target_system = 1;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.confirmation = 0;

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
	msg.trimmed = 0;

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
setPayloadCameraCaptureImage(){
	mavlink_command_long_t msg = {0};

	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.command = MAV_CMD_IMAGE_START_CAPTURE;
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