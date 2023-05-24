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

PayloadSdkInterface* my_payload = nullptr;
bool time_to_exit = false;

pthread_t thrd_recv;
pthread_t thrd_gstreamer;
GMainLoop *loop;
GstElement* main_pipeline = nullptr;
bool gstreamer_start();
void gstreamer_terminate();
void *start_loop_thread(void *threadid);


bool all_threads_init();
void *payload_recv_handle(void *threadid);
void quit_handler(int sig);

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

void _handle_msg_camera_information(mavlink_message_t* msg);
void _handle_msg_camera_stream_information(mavlink_message_t* msg);
void _handle_msg_command_ack(mavlink_message_t* msg);

int main(int argc, char *argv[]){
	printf("Starting GetStreaming example...\n");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface();

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// check connection
	while(!time_to_exit){
		mavlink_message_t msg;
		uint8_t msg_cnt = my_payload->getNewMewssage(msg);

		if(msg_cnt && msg.sysid == PAYLOAD_SYSTEM_ID && msg.compid == PAYLOAD_COMPONENT_ID){
			printf("Payload connected! \n");
			break;
		}
		usleep(10000);
	}

	// init thread to check receive message from payload
	all_threads_init();
	
	// change setting of VIDEO_OUT to BOTH_HDMI_UDP
	printf("Change VideoOutput to both HDMI and Ethernet \n");
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_OUTPUT, PAYLOAD_CAMERA_VIDEO_OUTPUT_BOTH, PARAM_TYPE_UINT32);
	usleep(500000);

	my_job = check_camera_info;
	while(!time_to_exit){
		// to caputre image with payload, follow this sequence
		switch(my_job){
		case idle:{
			// do nothing;
			printf("Program exit. \n");
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

bool all_threads_init(){

	int rc = pthread_create(&thrd_recv, NULL, &payload_recv_handle, (int *)1);
	if (rc){
		std::cout << "\nError: Can not create thread!" << rc << std::endl;
		return false;
	}
	std::cout << "Thread created\n" << std::endl;
}

void *payload_recv_handle(void *threadid)
{
	// check payload messages
	while(!time_to_exit){
	#if 1
		try{
			if(my_payload != nullptr){
				mavlink_message_t msg;
				uint8_t msg_cnt = my_payload->getNewMewssage(msg);
				// printf("message cnt: %d \n", msg_cnt);

				if(msg_cnt){
					// printf("Got %d message in queue \n", msg_cnt);
					// printf("   --> message %d from system_id: %d with component_id: %d \n", msg.msgid, msg.sysid, msg.compid);
					switch(msg.msgid){
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
					default: break;
					}
				}else{
					// printf("Recieved message queue empty. \n");
				}

				usleep(10000);
			}else{
				printf("My_payload nullptr \n");
			}
		}catch ( ... ) {};
	#else
		printf("thread running \n");
		usleep(1000000);
	#endif
	}
	pthread_exit(NULL);
}

void _handle_msg_command_ack(mavlink_message_t* msg){
	mavlink_command_ack_t cmd_ack = {0};

	mavlink_msg_command_ack_decode(msg, &cmd_ack);

	// printf("Got ACK for command %d with status %d\n", cmd_ack.command, cmd_ack.progress);
}

void _handle_msg_camera_information(mavlink_message_t* msg){
	mavlink_camera_information_t camera_info = {0};

	mavlink_msg_camera_information_decode(msg, &camera_info);

	// if payload have stream video
	if(camera_info.flags & (CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM)){
		printf("   ---> Got payload has streaming video, Check streaming URI \n");
		my_job = check_streaming_uri;
	}else{
		printf("   ---> Payload has no streaming video \n");
		printf("It looks like your payload was setting to output video only to HDMI \n");
		printf("Payload setting changed to: both HDMI and Ethernet.\n");
		printf("Please reboot your payload to apply this setting. Then try again.\n");
		my_job = idle;
	}
}

void _handle_msg_camera_stream_information(mavlink_message_t* msg){
	mavlink_video_stream_information_t stream_info = {0};

	mavlink_msg_video_stream_information_decode(msg, &stream_info);

	printf("   ---> Got streaming information: \n");
	printf("   ---> Streaming type: %d \n", stream_info.type);
	printf("   ---> Streaming resolution_v: %d \n", stream_info.resolution_v);
	printf("   ---> Streaming resolution_h: %d \n", stream_info.resolution_h);
	printf("   ---> Streaming name: %s \n", stream_info.name);
	printf("   ---> Streaming uri: %s \n", stream_info.uri);

	if(my_job == check_streaming_uri){
		my_job = start_pipeline;

		_is_rtsp_stream = (stream_info.type == VIDEO_STREAM_TYPE_RTSP) ? true : false;
		_stream_uri = stream_info.uri;
	}
}

//  pipeline handle
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
    	descr = "rtspsrc location=" + _stream_uri + " latency=0 ! rtph264depay ! h264parse ! nvv4l2decoder ! nvoverlaysink sync=false async=false";
    	printf("Does not support RTSP stream with this version \n");
    	exit(0);
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