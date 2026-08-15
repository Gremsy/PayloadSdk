/**
 * This example will show you how to receive object detection bounding boxes from the Payload
 * and draw them on the video stream.
 * This example will:
 * 1. Change the view mode to EO
 * 2. Change the Tracking Mode to Object Detection
 * 3. Request the video streaming information to get the RTSP uri
 * 4. Request the detection stream over V2_EXTENSION at 10Hz
 * 5. Open the video with OpenCV and draw the received bounding boxes
 *
 * Dependencies: OpenCV only (already required by examples/CMakeLists.txt)
 **/

#include "stdio.h"
#include"payloadSdkInterface.h"

#include <iostream>
#include <string>
#include <mutex>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>

PayloadSdkInterface* my_payload = nullptr;
bool time_to_exit = false;

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

void quit_handler(int sig);
void onDetectionsReceived(const det_packet_t& pkt);
void onPayloadStreamChanged(int event, char* param_char, double* param_double);
void handle_video();
void report_detections();

// How often the terminal dump is allowed to print, in milliseconds.
#define PRINT_PERIOD_MS 1000

// The payload stops sending packets entirely when detection is turned off, so
// silence is the only signal we get. After this long without a packet the boxes
// are treated as stale: they stop being drawn and the terminal says so.
// Must be comfortably longer than the requested stream interval (100ms).
#define DET_TIMEOUT_MS 1500

// Shared state: written by the MAVLink receive thread, read by the video loop.
std::mutex   det_mutex;
det_packet_t det_packet = {};
bool         det_valid = false;
std::chrono::steady_clock::time_point det_last_rx;

std::string stream_uri = "";
bool is_stream_ready = false;

// Reference resolution the payload uses when reporting box coordinates.
// The video may come in at a different size, so boxes are scaled before drawing.
#define DET_REF_W 1920.0
#define DET_REF_H 1080.0

// class_id -> name, for the COCO-80 model the payload runs on the EO camera.
//
// NOTE: the wire format only carries class_id, so this table has to match the
// model loaded on the payload. The IR camera runs a different model with its own
// (much shorter) class list, so ids seen while viewing IR do NOT map through this
// table — they will show up as "class N" instead of a wrong name.
static const char* const COCO_LABELS[] = {
	"person",         "bicycle",       "car",            "motorcycle",
	"airplane",       "bus",           "train",          "truck",
	"boat",           "traffic light", "fire hydrant",   "stop sign",
	"parking meter",  "bench",         "bird",           "cat",
	"dog",            "horse",         "sheep",          "cow",
	"elephant",       "bear",          "zebra",          "giraffe",
	"backpack",       "umbrella",      "handbag",        "tie",
	"suitcase",       "frisbee",       "skis",           "snowboard",
	"sports ball",    "kite",          "baseball bat",   "baseball glove",
	"skateboard",     "surfboard",     "tennis racket",  "bottle",
	"wine glass",     "cup",           "fork",           "knife",
	"spoon",          "bowl",          "banana",         "apple",
	"sandwich",       "orange",        "broccoli",       "carrot",
	"hot dog",        "pizza",         "donut",          "cake",
	"chair",          "couch",         "potted plant",   "bed",
	"dining table",   "toilet",        "tv",             "laptop",
	"mouse",          "remote",        "keyboard",       "cell phone",
	"microwave",      "oven",          "toaster",        "sink",
	"refrigerator",   "book",          "clock",          "vase",
	"scissors",       "teddy bear",    "hair drier",     "toothbrush",
};
#define COCO_LABEL_COUNT (sizeof(COCO_LABELS) / sizeof(COCO_LABELS[0]))

// Resolve a class id to a printable name. Writes into the caller's buffer so it
// is safe to call from both the MAVLink thread and the video loop.
// An id outside the table falls back to the raw number, so an unknown model shows
// something usable instead of a wrong name.
static void det_class_name(uint8_t class_id, char* out, size_t out_size){
	if(class_id < COCO_LABEL_COUNT)
		snprintf(out, out_size, "%s", COCO_LABELS[class_id]);
	else
		snprintf(out, out_size, "class %u", class_id);
}

// One fixed colour per class id, so the same class always looks the same and two
// different classes never share a colour.
//
// The hue is stepped by the golden ratio instead of spread evenly: consecutive
// ids land far apart on the colour wheel, so classes that tend to show up
// together (person/bicycle/car) stay easy to tell apart. Saturation and value
// are pinned high to keep the black label text readable on top.
static cv::Scalar class_color(uint8_t class_id){
	static cv::Scalar cache[256];
	static bool built = false;

	if(!built){
		const double golden = 0.618033988749895;
		for(int i = 0; i < 256; i++){
			// OpenCV stores hue in 0..179, not 0..359
			int hue = (int)(fmod(i * golden, 1.0) * 180.0);
			cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 220, 255));
			cv::Mat bgr;
			cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
			cv::Vec3b c = bgr.at<cv::Vec3b>(0, 0);
			cache[i] = cv::Scalar(c[0], c[1], c[2]);
		}
		built = true;
	}

	return cache[class_id];
}

int main(int argc, char *argv[]){
	printf("Starting Get Bounding Box Detection on Video example...\n");
	signal(SIGINT,quit_handler);

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadDetectionChanged(onDetectionsReceived);
	my_payload->regPayloadStreamChanged(onPayloadStreamChanged);

	// check connection
	my_payload->checkPayloadConnection();

	// Init the environment
	#ifndef ZIO
	// change view mode to EO
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
	#endif
	// change tracking mode to Object detection
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE, PAYLOAD_CAMERA_TRACKING_OBJ_DETECTION, PARAM_TYPE_UINT32);
	usleep(500000);

	// request the streaming information to get the rtsp uri
	my_payload->getPayloadCameraStreamingInformation();

	// request the detection stream over V2_EXTENSION, interval 100ms (10Hz)
	// if you do not want to receive the message anymore, need to set rate to 0
	printf("Request detection stream at 10Hz \n");
	my_payload->sendPayloadRequestStreamRate(MAVLINK_MSG_ID_V2_EXTENSION, 100);

	// Wait for the RTSP uri to arrive, then run the video loop on the main thread
	// (cv::imshow must be called from the main thread).
	while(!time_to_exit){
		if(is_stream_ready){
			handle_video();
			break;
		}
		// Report while waiting too: without a video stream this loop is the only
		// place the detection state gets shown.
		report_detections();
		usleep(100000);
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
        // stop the detection stream
        my_payload->sendPayloadRequestStreamRate(MAVLINK_MSG_ID_V2_EXTENSION, 0);
        usleep(100000);

        my_payload->sdkQuit();
    }
    catch (int error){}

    // end program here
    exit(0);
}

void onPayloadStreamChanged(int event, char* param_char, double* param_double){
	switch(event){
	case PAYLOAD_CAM_STREAMINFO:{
		stream_uri = std::string(param_char);
		printf("%s, stream uri: %s \n", __func__, stream_uri.c_str());
		if(!stream_uri.empty())
			is_stream_ready = true;
		break;
	}
	default: break;
	}
}

// Runs on the MAVLink receive thread: only latch the packet here, never print.
// Reporting happens from the main loop so a slow terminal cannot stall message
// processing.
void onDetectionsReceived(const det_packet_t& pkt){
	std::lock_guard<std::mutex> lock(det_mutex);
	det_packet = pkt;
	det_valid = true;
	det_last_rx = std::chrono::steady_clock::now();
}

// Dump the detection state to the terminal, for running without the video window.
// Call this from whichever loop is running; it throttles itself.
//
// Throttled to PRINT_PERIOD_MS rather than to a packet count, so the output stays
// readable no matter what stream rate was requested: at 20 boxes and 10Hz an
// unthrottled dump would be 200 lines a second.
//
// Three states the operator needs to tell apart:
//   - packets arriving with boxes   -> the table below
//   - packets arriving, none found  -> "no detections", link is alive
//   - no packets at all             -> "stream stopped", detection is off or the
//                                      link is down. The payload sends nothing
//                                      when detection is disabled, so a timeout
//                                      is the only way to notice.
void report_detections(){
	static std::chrono::steady_clock::time_point last_print;
	static bool warned_stopped = false;

	const auto now = std::chrono::steady_clock::now();
	if(now - last_print < std::chrono::milliseconds(PRINT_PERIOD_MS))
		return;
	last_print = now;

	det_packet_t pkt;
	bool valid = false;
	std::chrono::steady_clock::time_point last_rx;
	{
		std::lock_guard<std::mutex> lock(det_mutex);
		pkt = det_packet;
		valid = det_valid;
		last_rx = det_last_rx;
	}

	if(!valid){
		printf("[det] waiting for the first detection packet... \n");
		return;
	}

	const auto silence = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx);
	if(silence > std::chrono::milliseconds(DET_TIMEOUT_MS)){
		// Repeat at a slow cadence instead of once, so an operator joining the
		// terminal later still sees the current state.
		printf("[det] stream stopped (%.1fs without a packet) - detection off or link down \n",
		       silence.count() / 1000.0);
		warned_stopped = true;
		return;
	}

	if(warned_stopped){
		printf("[det] stream resumed \n");
		warned_stopped = false;
	}

	if(pkt.num_boxes == 0){
		// Still print something: it tells the operator the link is alive and the
		// payload simply is not seeing anything.
		printf("[det] frame %u | no detections \n", pkt.frame_id);
		return;
	}

	printf("[det] frame %u | %u box(es) \n", pkt.frame_id, pkt.num_boxes);
	printf("      %-4s  %-16s %5s  %6s %6s %6s %6s \n",
	       "id", "class", "conf", "x", "y", "w", "h");

	for(uint8_t i = 0; i < pkt.num_boxes; i++){
		const det_box_t& b = pkt.boxes[i];

		char class_text[32] = {0};
		det_class_name(b.class_id, class_text, sizeof(class_text));

		printf("      %-4u  %-16s %4.0f%%  %6u %6u %6u %6u \n",
		       b.track_id, class_text, b.confidence * 100.0 / 255.0,
		       b.x, b.y, b.w, b.h);
	}
}

void handle_video(){
	printf("Opening video stream... \n");
	cv::VideoCapture cap(stream_uri);
	if(!cap.isOpened()){
		printf("ERROR: can not open the video stream: %s \n", stream_uri.c_str());
		return;
	}

	cv::Mat frame;
	while(!time_to_exit){
		report_detections();

		if(!cap.read(frame) || frame.empty()){
			usleep(10000);
			continue;
		}

		// scale from the payload coordinate (1920x1080) to the real video resolution
		const double sx = frame.cols / DET_REF_W;
		const double sy = frame.rows / DET_REF_H;

		// take a snapshot of the latest detections
		det_packet_t pkt;
		bool valid = false;
		std::chrono::steady_clock::time_point last_rx;
		{
			std::lock_guard<std::mutex> lock(det_mutex);
			pkt = det_packet;
			valid = det_valid;
			last_rx = det_last_rx;
		}

		// Drop stale boxes instead of leaving them frozen on screen: when
		// detection is switched off the payload just stops sending, and the last
		// packet would otherwise stay drawn forever over a live picture.
		if(valid && (std::chrono::steady_clock::now() - last_rx) >
		            std::chrono::milliseconds(DET_TIMEOUT_MS)){
			valid = false;
		}

		if(valid){
			for(uint8_t i = 0; i < pkt.num_boxes; i++){
				const det_box_t& b = pkt.boxes[i];

				cv::Rect rect((int)(b.x * sx), (int)(b.y * sy),
				              (int)(b.w * sx), (int)(b.h * sy));

				const cv::Scalar color = class_color(b.class_id);

				cv::rectangle(frame, rect, color, 2);

				// Label format: "<track id> - <class name> - <confidence>%"
				char class_text[32] = {0};
				det_class_name(b.class_id, class_text, sizeof(class_text));

				char buf_text[80] = {0};
				snprintf(buf_text, sizeof(buf_text), "%u - %s - %.0f%%",
				         b.track_id, class_text, b.confidence * 100.0 / 255.0);

				int baseline = 0;
				cv::Size tsize = cv::getTextSize(buf_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
				cv::rectangle(frame,
				              cv::Point(rect.x, rect.y - tsize.height - 6),
				              cv::Point(rect.x + tsize.width + 4, rect.y),
				              color, cv::FILLED);
				cv::putText(frame, buf_text, cv::Point(rect.x + 2, rect.y - 4),
				            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, 8);
			}
		}

		cv::imshow("Detection", frame);
		if(cv::waitKey(1) == 27) break; // ESC to exit
	}

	cap.release();
	cv::destroyAllWindows();
}