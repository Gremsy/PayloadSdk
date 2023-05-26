#include "main.h"

// ------------------------------------------------------------------------------
//   VARIABLES
// ------------------------------------------------------------------------------
static T_ConnInfo s_conn = {
	CONTROL_UART,
	"/dev/ttyUSB0",
	115200
};

/**/

static void usage(){
	std::cout << "THIS IS MESSAGE SHOW YOU HOW TO USE PAYLOAD SDK" << std::endl;
}
/*!<@brief: 
 * 
 * */
static int parse_argument(int argc, char** argv){
	/*! check argument numbers*/
	printf("argc : [%d]\r\n",argc);
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
		if(!strcmp(argv[i],"-s") || !strcmp(argv[i],"--serial")){
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

}
/*!<@brief:
 * @para1:
 * @retval:
 * */
uint8_t run_sample(){

}

int main(int argc,char** argv){
	/*parse argument from command line*/
	if(parse_argument(argc,argv) == 0){
		exit(0);
	}
	signal(SIGINT,quit_handler);

	/*!Init payload interface class pointer*/
	my_payload = new PayloadSdkInterface(s_conn);
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");
	
	exit(0);
}



