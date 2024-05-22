#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <string>
using namespace std;

#include <gst/gst.h>
#include <gst/app/app.h>
#include <gst/base/gstbasetransform.h>
#include <gst/video/gstvideofilter.h>
#include <gst/video/video-format.h>
#include <gst/video/video-info.h>
#include <gst/video/video.h>

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


pthread_t thrd_gstreamer;
GMainLoop *loop;
GstElement* main_pipeline = nullptr;
bool gstreamer_start();
void gstreamer_terminate();
void *start_loop_thread(void *threadid);


void quit_handler(int sig);

void onPayloadStatusChanged(int event, double* param);
void onPayloadStreamChanged(int event, char* param_char, double* param_double);

typedef enum{
	idle = 0,
	check_camera_info,
	check_streaming_uri,
	start_pipeline,
	pipeline_running,
}get_stream_sequence_t;

get_stream_sequence_t my_job = idle;
uint8_t time_to_view = 10;
bool _is_rtsp_stream = false;
string _stream_uri;


int main(int argc, char *argv[]){
	printf("Starting GetStreaming example...\n");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);
	my_payload->regPayloadStreamChanged(onPayloadStreamChanged);

	// check connection
	my_payload->checkPayloadConnection();


	// set record source
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IREO, PARAM_TYPE_UINT32);
	
	usleep(500000);

	my_job = check_camera_info;
	while(!time_to_exit){
		// to caputre image with payload, follow this sequence
		switch(my_job){
		case idle:{
			// do nothing;
			printf("Program exit. \n");

			// close payload interface
			try {
				my_payload->sdkQuit();
			}
			catch (int error){}

			exit(0);
			break;
		}
		case check_camera_info:{
			printf("Send request to read camera's information \n");
			my_payload->getPayloadCameraInformation();
			break;
		}
		case check_streaming_uri:{
			my_payload->getPayloadCameraStreamingInformation();
			break;
		}
		case start_pipeline:{
			gstreamer_start();
			my_job = pipeline_running;
			break;
		}
		case pipeline_running:{
			break;
		}
		default: break;
		}

		usleep(1500000); // sleep 0.5s
	}

    
	return 0;
}

void quit_handler( int sig ){
    printf("\n");
    printf("TERMINATING AT USER REQUEST \n");
    printf("\n");

    time_to_exit = true;

    // close payload interface
    try {
        my_payload->sdkQuit();
        gstreamer_terminate();
    }
    catch (int error){}

    // end program here
    exit(0);
}

void onPayloadStatusChanged(int event, double* param){

	switch(event){
	case PAYLOAD_CAM_INFO:{
		// param[0]: flags

		// if payload have stream video
		if((int16_t)param[0] & (CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM)){
			printf("   ---> Got payload has streaming video, Check streaming URI \n");
			my_job = check_streaming_uri;
		}else{
			printf("   ---> Payload has no streaming video \n");
			printf("It looks like your payload was setting to output video only to HDMI \n");
			printf("Payload setting changed to: both HDMI and Ethernet.\n");
			printf("Please reboot your payload to apply this setting. Then try again.\n");
			my_job = idle;
		}

		break;
	}
	default: break;
	}
}

void onPayloadStreamChanged(int event, char* param_char, double* param_double){
	switch(event){
	case PAYLOAD_CAM_STREAMINFO:{
		// param_char: stream uri
		// param_double[0]: stream type

		printf("   ---> Got streaming information: \n");
		printf("   ---> Streaming type: %.2f \n", param_double[0]);
		printf("   ---> Streaming resolution_v: %.2f \n", param_double[1]);
		printf("   ---> Streaming resolution_h: %.2f \n", param_double[2]);
		printf("   ---> Streaming uri: %s \n", param_char);

		if(my_job == check_streaming_uri){
			my_job = start_pipeline;

			_is_rtsp_stream = (param_double[0] == VIDEO_STREAM_TYPE_RTSP) ? true : false;
			_stream_uri = param_char;
		}

		break;
	}
	default: break;
	}
}

//  pipeline handle to show video
bool gstreamer_start(){

    int rc = pthread_create(&thrd_gstreamer, NULL, &start_loop_thread, (int *)1);
	if (rc){
		std::cout << "\nError: Can not create thread!" << rc << std::endl;
		return false;
	}
	std::cout << "Gstreamer thread created\n" << std::endl;
}

void *start_loop_thread(void *threadid)
{
	GError *error = nullptr;

	// gst_init(&argc, &argv);
    gst_init(NULL, NULL);

    string descr;
    if(!_is_rtsp_stream){
	    descr = "udpsrc port=" + _stream_uri + " ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! nvvidconv ! nvoverlaysink sync=false async=false"
	                    ;
    }else{
    	descr = "rtspsrc location=" + _stream_uri + " latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink sync=false async=false";
    }

    printf("%s %s \n", __func__, descr.c_str());
    main_pipeline = gst_parse_launch(descr.c_str(), &error);

    if(error) {
        g_print("could not construct pipeline: %s\n", error->message);
        g_error_free(error);
        exit(-1);
    }
   
    loop = g_main_loop_new(NULL, FALSE);


    gst_element_set_state(GST_ELEMENT(main_pipeline), GST_STATE_PLAYING);

    printf("gstreamer init done\n");

    g_main_loop_run(loop);

    pthread_exit(NULL);
}

void gstreamer_terminate(){
    printf("Exit gstreamer\n");
    if(loop != nullptr)
    	g_main_loop_quit(loop);

    if(main_pipeline != nullptr){
	    gst_element_set_state(GST_ELEMENT(main_pipeline), GST_STATE_NULL);
	    gst_object_unref(GST_OBJECT(main_pipeline));
	}
    
    if(thrd_gstreamer)
    	pthread_join(thrd_gstreamer ,NULL);
}