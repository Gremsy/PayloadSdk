#include "payloadSdkInterface.h"

PayloadSdkInterface::PayloadSdkInterface(){

}

PayloadSdkInterface::~PayloadSdkInterface(){

}

bool 
PayloadSdkInterface::
sdkInitConnection(){
	/* Port for connect with payload */
    Generic_Port *port;
    port = new Serial_Port(payload_uart_port, payload_uart_baud);
    
    /* Instantiate an gimbal interface object */
    int _system_id = 1;
    payload_interface = new Autopilot_Interface(port, _system_id, MAV_COMP_ID_ONBOARD_COMPUTER, 2, MAVLINK_COMM_0);


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