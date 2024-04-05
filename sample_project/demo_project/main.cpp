#include "main.h"

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

T_psdk_process_state s_proc;

/*!< Private prototype */
static void usage(){
	std::cout << "Usage : If Onboard computer connect with Payload by Serial Port :" << std::endl;
	std::cout <<" ./demo_project --device | -d <serial_port> --baud | -b <baudrate> " << std::endl;
	std::cout << "If Onboard computer connect with Payload by UDP :" << std::endl;
	std::cout << "./demo_project --udp | -u <payload_ip> --port | -p <connect_port> " << std::endl;
	std::cout << "E.g: ./demo_project --device /dev/ttyUSB0 --baud 115200" << std::endl;
	std::cout << "Thank you!! \r\n";
}
/*!<@brief: 
 * 
 * */
static int parse_argument(int argc, char** argv){
	/*! check argument numbers*/
	if(argc < 2){
		/**/
		return 1;
	}
	/*! read each arg*/
	for(int i = 1; i < argc; i++){
		//HELP command
		if(!strcmp(argv[i],"-h")|| !strcmp(argv[i],"--help")){
			usage();
			return 0;
		}	

		//UART DEVICE
		if(!strcmp(argv[i],"-d") || !strcmp(argv[i],"--device")){
			s_conn.type = CONTROL_UART;
			if (argc > i + 1){
				s_conn.device.uart.name = argv[i + 1];
			}
		}
		if(!strcmp(argv[i],"-b") || !strcmp(argv[i],"--baud")){
			if (argc > i + 1){
				s_conn.device.uart.baudrate = atoi(argv[i + 1]);
			}
		}

		//UDP CONNECTION
		if(!strcmp(argv[i],"-u") || !strcmp(argv[i],"--udp")){
			s_conn.type = CONTROL_UDP;
			if (argc > i + 1){
				s_conn.device.udp.ip = argv[i + 1];
			}
		}
		if(!strcmp(argv[i],"-p") || !strcmp(argv[i],"--port")){
			if (argc > i + 1){
				s_conn.device.udp.port = atoi(argv[i + 1]);
			}
		}

	}
}

void quit_handler(int sig){
    printf("\n");
    printf("TERMINATING AT USER REQUEST \n");
    printf("\n");

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    exit(0);
}


static void _handle_msg_param_ext_value(mavlink_message_t* msg){
	// printf("%s msg_id %d \n", __func__, msg->msgid);

	mavlink_param_ext_value_t param_ext_value = {0};
	mavlink_msg_param_ext_value_decode(msg, &param_ext_value);

    uint32_t param_uint32;
    memcpy(&param_uint32, param_ext_value.param_value, sizeof(param_uint32));
    

	printf(" --> Param_id: %s, value: %d\n", param_ext_value.param_id, param_uint32);
}
/*!<@brief:
 * @para1:
 * @retval:
 * */
int8_t psdk_run_sample(){
	mavlink_message_t msg;
	if(my_payload == nullptr) {
		PRINT_ERR("%s | %d | PayloadSdkInterface is nullptr");
		return -1;
	}
	uint8_t msg_cnt = my_payload->getNewMewssage(msg);
	if(!msg_cnt){
		return 1;
	}

	switch(s_proc._state){
	case STATE_IDLE:
		{
			s_proc._state = STATE_WAIT_TO_CONNECT_PAYLOAD;
		}
		break;
	case STATE_WAIT_TO_CONNECT_PAYLOAD:
		{
			if(msg.sysid == PAYLOAD_SYSTEM_ID && msg.compid == PAYLOAD_COMPONENT_ID){	// found message from payload
				
				PRINT_INFO("%s | %s",__func__,state_name[s_proc._state]);
				PRINT_INFO("Connnected to payload!!!");

				usleep(5000000);
				s_proc._state = STATE_SET_PAYLOAD_PARAM;
			}
		}
		break;
	case STATE_SET_PAYLOAD_PARAM:
		{
		#if 1
			PRINT_INFO("%s | %s | Upload default parameter for payload!!!!",__func__,state_name[s_proc._state]);
			/*! Enable tracking mode*/
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE,PAYLOAD_CAMERA_TRACKING_ENABLE,PARAM_TYPE_UINT32);
			/*! Set OSD Mode to STATUS */
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_OSD_MODE,PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS,PARAM_TYPE_UINT32);
			/*! Set Dual RC Mode*/
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE,PAYLOAD_CAMERA_RC_MODE_STANDARD,PARAM_TYPE_UINT32);
			/*! Turn Off Flip mode*/
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_FLIP,PAYLOAD_CAMERA_VIDEO_FLIP_OFF,PARAM_TYPE_UINT32);
			/*! Set shutter speed to 1/1000*/
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED,PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED_1_1000,PARAM_TYPE_UINT32);
			/*! Set Apertture value to 10*/
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_APERTURE_VALUE,10,PARAM_TYPE_UINT32);
			/*! Set Bright value value to 20*/
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_BRIGHT_VALUE,20,PARAM_TYPE_UINT32);
			/*! Set Auto White Balance mode*/
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE,PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE_AUTO,PARAM_TYPE_UINT32);
			/*! Set Superresolution for Zoom mode */
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE,PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION,PARAM_TYPE_UINT32);
			/*!< Set Gimbal Mode*/
			my_payload->setGimbalMode(Gimbal_Protocol::GIMBAL_FOLLOW_MODE);
		#endif	
			usleep(2000000);
			my_payload->getPayloadCameraSettingList();
			s_proc._time_usec = _get_time_usec();
			s_proc._state = STATE_LOAD_PAYLOAD_PARAM;

			// set palette if has
			usleep(100000);
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_3, PARAM_TYPE_UINT32);

			usleep(100000);
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EOIR, PARAM_TYPE_UINT32);
		}	
		break;
	case STATE_LOAD_PAYLOAD_PARAM:
		{
			if(msg.msgid == MAVLINK_MSG_ID_PARAM_EXT_VALUE){
				_handle_msg_param_ext_value(&msg);
				s_proc._time_usec = _get_time_usec();
				break;
			}

			uint64_t curr_time = _get_time_usec();
			if((curr_time - s_proc._time_usec) > 500000){	/* Wait for 100ms from last ext value message*/
				PRINT_INFO("%s | %s | Load Parameter of payload successfully!!!!",__func__,state_name[s_proc._state]);
				s_proc._state = STATE_START_MOVEMENT;
				s_proc._time_usec = _get_time_usec();
			}
		}
		break;
	case STATE_START_MOVEMENT:
		{
			// PRINT_INFO("%s | %s | Start sequence of movement for payload",__func__,state_name[s_proc._state]);
			// my_payload->setGimbalPowerOff();
			// usleep(2000000);
			// my_payload->setGimbalPowerOn();
			// usleep(2000000);
			s_proc._state = STATE_MOVEMENT_0;
		}
		break;
	case STATE_MOVEMENT_0:
		{
			PRINT_INFO("%s | %s | Recenter gimbal postion",__func__,state_name[s_proc._state]);
			my_payload->setGimbalResetMode(Gimbal_Protocol::GIMBAL_RESET_MODE_PITCH_AND_YAW);
			usleep(3000000);
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_1X, PARAM_TYPE_UINT32);

			usleep(10000000); // wait for 10 secs

			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_3, PARAM_TYPE_UINT32);
			usleep(10000000);
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IR, PARAM_TYPE_UINT32);
			usleep(10000000);
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_1, PARAM_TYPE_UINT32);
			usleep(10000000);

			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EOIR, PARAM_TYPE_UINT32);
			usleep(5000000);
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, PAYLOAD_CAMERA_IR_PALETTE_3, PARAM_TYPE_UINT32);
			usleep(10000000);

			my_payload->setGimbalSpeed(0, 0, 0, Gimbal_Protocol::INPUT_SPEED); //fix bug for angle mode
			s_proc._state = STATE_MOVEMENT_1;
		}
		break;
	case STATE_MOVEMENT_1:
		{
			PRINT_INFO("%s | %s | Zoom in to 30x 5 seconds and Zoom out to 1x 5 seconds",__func__,state_name[s_proc._state]);
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_30X, PARAM_TYPE_UINT32);	
			usleep(10000000);
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_1X, PARAM_TYPE_UINT32);	
			usleep(10000000);
			s_proc._state = STATE_MOVEMENT_2;	
		}
		break;
	case STATE_MOVEMENT_2:
		{	
			PRINT_INFO("%s | %s | Set yaw to 60 degree and Zoom in to 20x",__func__,state_name[s_proc._state]);
			my_payload->setGimbalSpeed(0, 0, 60, Gimbal_Protocol::INPUT_ANGLE);
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_20X, PARAM_TYPE_UINT32);	
			usleep(10000000);
			s_proc._state = STATE_MOVEMENT_3;
			s_proc._time_usec = _get_time_usec();
		}	
		break;	
	case STATE_MOVEMENT_3:
		{			
			PRINT_INFO("%s | %s | Rotate yaw axis with speed of 100 degree/s counter-clockwise direction with 20x resolution for 8 seconds",__func__,state_name[s_proc._state]);
			/**/
			uint64_t curr_time = _get_time_usec();
			if((curr_time - s_proc._time_usec) > 8000000){
				s_proc._time_usec = _get_time_usec();
				s_proc._state = STATE_MOVEMENT_4;
			}else{
				my_payload->setGimbalSpeed(0.0, 0.0 , -100.0f, Gimbal_Protocol::INPUT_SPEED);				
			}
		}
		break;
	case STATE_MOVEMENT_4:
		{
			PRINT_INFO("%s | %s | Rotate yaw axis with speed 100 degree/s clockwise direction with 20x resolution for 8 seconds ",__func__,state_name[s_proc._state]);
			uint64_t curr_time = _get_time_usec();
			if((curr_time - s_proc._time_usec) > 8000000){
				s_proc._time_usec = _get_time_usec();
				s_proc._state = STATE_MOVEMENT_5;
			}else{
				my_payload->setGimbalSpeed(0.0f, 0.0f , 100.0f, Gimbal_Protocol::INPUT_SPEED);				
			}
		}
		break;
	case STATE_MOVEMENT_5:
		{
			PRINT_INFO("%s | %s | Stop gimbal, Zoom in to 1x",__func__,state_name[s_proc._state]);
			/**/
			my_payload->setGimbalSpeed(0.0f,0.0f,0.0f,Gimbal_Protocol::INPUT_SPEED);
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_1X, PARAM_TYPE_UINT32);	
			usleep(10000000);
			s_proc._state = STATE_MOVEMENT_6;
		}
		break;
	case STATE_MOVEMENT_6:
		{
			PRINT_INFO("%s | %s | Set pitch angle to 20 degree, yaw angle to 60 degree and Zoom in to 2x for 5 seconds",__func__,state_name[s_proc._state]);
			/**/
			my_payload->setGimbalSpeed(20.0f,0.0f,-60.0f,Gimbal_Protocol::INPUT_ANGLE);
			usleep(5000000);
			uint8_t _zoom_level = ZOOM_SUPER_RESOLUTION_4X;
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, _zoom_level, PARAM_TYPE_UINT32);	
			usleep(10000000);
			s_proc._state = STATE_MOVEMENT_7;
			s_proc._time_usec = _get_time_usec();
		}
		break;
	case STATE_MOVEMENT_7:
		{
			PRINT_INFO("%s | %s | Rotate yaw axis with speed 200 degree/s clockwise direction with 2X resolution for 8 seconds",__func__,state_name[s_proc._state]);
			uint64_t curr_time = _get_time_usec();
			if((curr_time - s_proc._time_usec) > 8000000){
				s_proc._time_usec = _get_time_usec();
				s_proc._state = STATE_MOVEMENT_8;
			}else{
				my_payload->setGimbalSpeed(0.0f,0.0f ,200.0f, Gimbal_Protocol::INPUT_SPEED);				
			}
		}
		break;
	case STATE_MOVEMENT_8:
		{
			PRINT_INFO("%s | %s | Rotate yaw axis with speed 100 degree/s counter-clockwise direction  with 2X resolution for 8 seconds",__func__,state_name[s_proc._state]);
			uint64_t curr_time = _get_time_usec();
			if((curr_time - s_proc._time_usec) > 8000000){
				s_proc._time_usec = _get_time_usec();
				s_proc._state = STATE_DONE;
			}else{
				my_payload->setGimbalSpeed(0.0f,0.0f ,-200.0f, Gimbal_Protocol::INPUT_SPEED);				
			}
		}
		break;
	case STATE_DONE:
		{
			PRINT_INFO("%s | %s | Stop gimbal, Zoom in to 1x",__func__,state_name[s_proc._state]);
			my_payload->setGimbalSpeed(0.0f, 0.0f , 0.0f, Gimbal_Protocol::INPUT_SPEED);	
			my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_1X, PARAM_TYPE_UINT32);	
			usleep(5000000);
			s_proc._state = STATE_START_MOVEMENT;
			/**/
		}
		break;
	default:
		break;
	}
	return 1;
}


int main(int argc,char** argv){
	/*parse argument from command line*/
	if(parse_argument(argc,argv) == 0){
		exit(0);
	}
	signal(SIGINT,quit_handler);
	time_to_exit = false;
	/*!Init payload interface class pointer*/
	my_payload = new PayloadSdkInterface(s_conn);

	if(my_payload->sdkInitConnection() == false){
		PRINT_ERR("Payload Interface Init failed,close the program!!!!");
		exit(1);
	}
	
	while(!time_to_exit){

		if(psdk_run_sample() < 0){
			break;
		}
		usleep(1000);	//sleep 1ms
	}
	PRINT_INFO("END PROGRAM");
	exit(0);
}



