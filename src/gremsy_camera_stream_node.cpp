#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <signal.h>
#include <pthread.h>
#include <cstdlib>
#include <string>
#include <mutex>
#include <atomic>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/gstvideometa.h>

// Include the payload SDK interface
#include "payloadSdkInterface.h"

using namespace std::chrono_literals;
using namespace std;

typedef enum{
    idle = 0,
    check_camera_info,
    check_streaming_uri,
    stream_info_received,
    processing_frames
} stream_sequence_t;

// Forward declaration of the class
class GremsyCameraStreamNode;

// Global variables for C-style callbacks (must be declared at global scope)
GremsyCameraStreamNode* g_stream_node_instance = nullptr;
std::mutex g_instance_mutex;

// Forward declarations for C-style callbacks
void onPayloadStatusChanged(int event, double* param);
void onPayloadStreamChanged(int event, char* param_char, double* param_double);

class GremsyCameraStreamNode : public rclcpp::Node
{
public:
    GremsyCameraStreamNode() : Node("gremsy_camera_stream_node")
    {
        declare_common_parameters();
        declare_node_parameters();
        load_common_parameters();
        load_node_parameters();
        
        // Initialize payload connection
        initialize_payload_connection();
        
        setup_publishers_subscribers();
        
        // Initialize camera streaming with config values
        initialize_camera_streaming();
        
        RCLCPP_INFO(this->get_logger(), "Gremsy Camera Stream Node initialized");
        RCLCPP_INFO(this->get_logger(), "Stream mode: %s", stream_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "Auto-start streaming: %s", auto_start_stream_ ? "enabled" : "disabled");
    }
    
    ~GremsyCameraStreamNode() {
        // Clear global instance safely
        std::lock_guard<std::mutex> lock(g_instance_mutex);
        if (g_stream_node_instance == this) {
            g_stream_node_instance = nullptr;
        }
        cleanup_resources();
    }

private:
    void declare_common_parameters() {
        this->declare_parameter("network.udp_ip_target", "10.4.1.13");
        this->declare_parameter("network.udp_port_target", 14566);
        this->declare_parameter("uart.port", "/dev/ttyUSB0");
        this->declare_parameter("uart.baudrate", 115200);
        this->declare_parameter("connection.control_method", 1);
        this->declare_parameter("logging.level", "INFO");
        this->declare_parameter("node.publish_rate", 30.0);
        this->declare_parameter("node.timeout", 5.0);
    }
    
    void declare_node_parameters() {
        // Topic parameters
        this->declare_parameter("topics.rgb_camera_stream", "gimbal/camera/rgb/image");
        this->declare_parameter("topics.ir_camera_stream", "gimbal/camera/ir/image");
        this->declare_parameter("topics.rgb_camera_info", "gimbal/camera/rgb/camera_info");
        this->declare_parameter("topics.ir_camera_info", "gimbal/camera/ir/camera_info");
        this->declare_parameter("topics.stream_control", "gimbal/camera/stream/control");
        this->declare_parameter("topics.stream_status", "gimbal/camera/stream_status");
        this->declare_parameter("topics.storage_info", "gimbal/camera/storage_info");
        
        // Camera parameters
        this->declare_parameter("camera.stream_mode", "DUAL");
        this->declare_parameter("camera.rgb_enabled", true);
        this->declare_parameter("camera.ir_enabled", true);
        this->declare_parameter("camera.auto_start_stream", true);
        this->declare_parameter("camera.stream_quality", "HD");
        this->declare_parameter("camera.framerate", 30);
        this->declare_parameter("camera.encoding", "h264");
        
        // Dual stream parameters
        this->declare_parameter("camera.dual_stream.layout", "SIDE_BY_SIDE");
        this->declare_parameter("camera.dual_stream.rgb_position", "LEFT");
        this->declare_parameter("camera.dual_stream.ir_position", "RIGHT");
        this->declare_parameter("camera.dual_stream.pip_size", 0.25);
        this->declare_parameter("camera.dual_stream.pip_position", "TOP_RIGHT");
        
        // Stream URLs (auto-detected by default)
        this->declare_parameter("camera.stream_urls.rgb_url", "");
        this->declare_parameter("camera.stream_urls.ir_url", "");
        this->declare_parameter("camera.stream_urls.dual_url", "");
        
        // Recording parameters
        this->declare_parameter("camera.recording.auto_record", false);
        this->declare_parameter("camera.recording.record_duration", 300);
        this->declare_parameter("camera.recording.record_path", "/tmp/gremsy_recordings");
    }
    
    void load_common_parameters() {
        udp_ip_target_ = this->get_parameter("network.udp_ip_target").as_string();
        udp_port_target_ = this->get_parameter("network.udp_port_target").as_int();
        uart_port_ = this->get_parameter("uart.port").as_string();
        uart_baudrate_ = this->get_parameter("uart.baudrate").as_int();
        control_method_ = this->get_parameter("connection.control_method").as_int();
        log_level_ = this->get_parameter("logging.level").as_string();
        publish_rate_ = this->get_parameter("node.publish_rate").as_double();
        timeout_ = this->get_parameter("node.timeout").as_double();
    }
    
    void load_node_parameters() {
        // Load topic names
        rgb_stream_topic_ = this->get_parameter("topics.rgb_camera_stream").as_string();
        ir_stream_topic_ = this->get_parameter("topics.ir_camera_stream").as_string();
        rgb_camera_info_topic_ = this->get_parameter("topics.rgb_camera_info").as_string();
        ir_camera_info_topic_ = this->get_parameter("topics.ir_camera_info").as_string();
        stream_control_topic_ = this->get_parameter("topics.stream_control").as_string();
        stream_status_topic_ = this->get_parameter("topics.stream_status").as_string();
        storage_info_topic_ = this->get_parameter("topics.storage_info").as_string();
        
        // Load camera settings
        stream_mode_ = this->get_parameter("camera.stream_mode").as_string();
        rgb_enabled_ = this->get_parameter("camera.rgb_enabled").as_bool();
        ir_enabled_ = this->get_parameter("camera.ir_enabled").as_bool();
        auto_start_stream_ = this->get_parameter("camera.auto_start_stream").as_bool();
        stream_quality_ = this->get_parameter("camera.stream_quality").as_string();
        framerate_ = this->get_parameter("camera.framerate").as_int();
        encoding_ = this->get_parameter("camera.encoding").as_string();
        
        // Load dual stream settings
        dual_layout_ = this->get_parameter("camera.dual_stream.layout").as_string();
        rgb_position_ = this->get_parameter("camera.dual_stream.rgb_position").as_string();
        ir_position_ = this->get_parameter("camera.dual_stream.ir_position").as_string();
        pip_size_ = this->get_parameter("camera.dual_stream.pip_size").as_double();
        pip_position_ = this->get_parameter("camera.dual_stream.pip_position").as_string();
        
        // Load stream URLs
        rgb_url_ = this->get_parameter("camera.stream_urls.rgb_url").as_string();
        ir_url_ = this->get_parameter("camera.stream_urls.ir_url").as_string();
        dual_url_ = this->get_parameter("camera.stream_urls.dual_url").as_string();
        
        // Load recording settings
        auto_record_ = this->get_parameter("camera.recording.auto_record").as_bool();
        record_duration_ = this->get_parameter("camera.recording.record_duration").as_int();
        record_path_ = this->get_parameter("camera.recording.record_path").as_string();
        
        // Override individual enables based on stream mode
        if (stream_mode_ == "RGB") {
            rgb_enabled_ = true;
            ir_enabled_ = false;
        } else if (stream_mode_ == "IR") {
            rgb_enabled_ = false;
            ir_enabled_ = true;
        } else if (stream_mode_ == "DUAL") {
            rgb_enabled_ = true;
            ir_enabled_ = true;
        }
    }
    
    void initialize_payload_connection() {
        printf("Starting Gremsy Camera Stream Node...\n");
        
        T_ConnInfo s_conn;
        
        if (control_method_ == CONTROL_UDP) {
            s_conn.type = CONTROL_UDP;
            s_conn.device.udp.ip = const_cast<char*>(udp_ip_target_.c_str());
            s_conn.device.udp.port = udp_port_target_;
            RCLCPP_INFO(this->get_logger(), "Connecting via UDP to %s:%d", 
                       udp_ip_target_.c_str(), udp_port_target_);
        } else {
            s_conn.type = CONTROL_UART;
            s_conn.device.uart.name = const_cast<char*>(uart_port_.c_str());
            s_conn.device.uart.baudrate = uart_baudrate_;
            RCLCPP_INFO(this->get_logger(), "Connecting via UART to %s at %d baud", 
                       uart_port_.c_str(), uart_baudrate_);
        }

        try {
            payload_interface_ = new PayloadSdkInterface(s_conn);
            payload_interface_->sdkInitConnection();
            RCLCPP_INFO(this->get_logger(), "Waiting for payload signal!");
            
            // Set global instance with thread safety
            std::lock_guard<std::mutex> lock(g_instance_mutex);
            g_stream_node_instance = this;
            
            // Register callback functions (C-style callbacks)
            payload_interface_->regPayloadStatusChanged(onPayloadStatusChanged);
            payload_interface_->regPayloadStreamChanged(onPayloadStreamChanged);
            
            payload_interface_->checkPayloadConnection();
            
            RCLCPP_INFO(this->get_logger(), "✅ Payload SDK initialized successfully");
            connection_established_ = true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to initialize payload SDK: %s", e.what());
            connection_established_ = false;
            if (payload_interface_) {
                delete payload_interface_;
                payload_interface_ = nullptr;
            }
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "❌ Unknown error during payload SDK initialization");
            connection_established_ = false;
            if (payload_interface_) {
                delete payload_interface_;
                payload_interface_ = nullptr;
            }
        }
    }
    
    void cleanup_resources() {
        shutdown_requested_.store(true);
        
        // Cleanup GStreamer pipeline
        cleanup_gstreamer();
        
        if (payload_interface_) {
            try {
                RCLCPP_INFO(this->get_logger(), "Shutting down payload SDK...");
                payload_interface_->sdkQuit();
                delete payload_interface_;
                payload_interface_ = nullptr;
            } catch (...) {
                // Ignore errors during cleanup
            }
        }
    }

    void setup_publishers_subscribers() {
        if (rgb_enabled_) {
            rgb_stream_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(rgb_stream_topic_, 10);
            rgb_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(rgb_camera_info_topic_, 10);
        }
        
        if (ir_enabled_) {
            ir_stream_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(ir_stream_topic_, 10);
            ir_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(ir_camera_info_topic_, 10);
        }
        
        storage_info_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(storage_info_topic_, 10);
        stream_status_publisher_ = this->create_publisher<std_msgs::msg::String>(stream_status_topic_, 10);
        
        stream_control_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            stream_control_topic_, 10,
            std::bind(&GremsyCameraStreamNode::stream_control_callback, this, std::placeholders::_1));
        
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
        stream_timer_ = this->create_wall_timer(
            timer_period, std::bind(&GremsyCameraStreamNode::publish_streams, this));
    }

    void initialize_camera_streaming() {
        if (!connection_established_ || !payload_interface_) {
            RCLCPP_WARN(this->get_logger(), "Cannot initialize camera streaming - not connected");
            return;
        }
        
        try {
            // Initialize GStreamer
            if (!gst_is_initialized()) {
                gst_init(nullptr, nullptr);
                RCLCPP_INFO(this->get_logger(), "🎬 GStreamer initialized");
            }
            
            // Set stream mode based on configuration
            if (stream_mode_ == "DUAL" || (rgb_enabled_ && ir_enabled_)) {
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_VIEW_SRC, 
                    PAYLOAD_CAMERA_VIEW_IREO, 
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "🎥 Set view source to DUAL (IR+EO) - Layout: %s", dual_layout_.c_str());
            } else if (stream_mode_ == "IR" || ir_enabled_) {
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_VIEW_SRC, 
                    PAYLOAD_CAMERA_VIEW_IR, 
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "🎥 Set view source to IR only");
            } else {
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_VIEW_SRC, 
                    PAYLOAD_CAMERA_VIEW_EO, 
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "🎥 Set view source to EO (RGB) only");
            }
            
            usleep(500000);
            
            if (auto_start_stream_) {
                streaming_state_ = check_camera_info;
                RCLCPP_INFO(this->get_logger(), "🚀 Auto-starting stream initialization...");
            } else {
                streaming_state_ = idle;
                RCLCPP_INFO(this->get_logger(), "⏸️ Streaming ready but not auto-started (send START command)");
            }
            
            RCLCPP_INFO(this->get_logger(), "Camera streaming initialized");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera streaming: %s", e.what());
        }
    }
    
    bool setup_gstreamer_pipeline() {
        if (stream_uri_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No stream URI available");
            return false;
        }
        
        try {
            std::string pipeline_desc;
            
            if (stream_mode_ == "DUAL") {
                // For dual stream, we'll decode the combined stream and split it
                pipeline_desc = "rtspsrc location=" + stream_uri_ + 
                               " latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! "
                               "videoconvert ! video/x-raw,format=RGB ! "
                               "appsink name=dual_sink emit-signals=true sync=false";
            } else if (stream_mode_ == "RGB") {
                pipeline_desc = "rtspsrc location=" + stream_uri_ + 
                               " latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! "
                               "videoconvert ! video/x-raw,format=RGB ! "
                               "appsink name=rgb_sink emit-signals=true sync=false";
            } else if (stream_mode_ == "IR") {
                pipeline_desc = "rtspsrc location=" + stream_uri_ + 
                               " latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! "
                               "videoconvert ! video/x-raw,format=GRAY8 ! "
                               "appsink name=ir_sink emit-signals=true sync=false";
            }
            
            RCLCPP_INFO(this->get_logger(), "🔧 Creating GStreamer pipeline: %s", pipeline_desc.c_str());
            
            GError* error = nullptr;
            gst_pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &error);
            
            if (error) {
                RCLCPP_ERROR(this->get_logger(), "GStreamer pipeline error: %s", error->message);
                g_error_free(error);
                return false;
            }
            
            if (!gst_pipeline_) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline");
                return false;
            }
            
            // Setup app sink callbacks based on stream mode
            if (stream_mode_ == "DUAL") {
                setup_dual_stream_sink();
            } else if (stream_mode_ == "RGB") {
                setup_rgb_stream_sink();
            } else if (stream_mode_ == "IR") {
                setup_ir_stream_sink();
            }
            
            // Start the pipeline
            GstStateChangeReturn ret = gst_element_set_state(gst_pipeline_, GST_STATE_PLAYING);
            if (ret == GST_STATE_CHANGE_FAILURE) {
                RCLCPP_ERROR(this->get_logger(), "Failed to start GStreamer pipeline");
                gst_object_unref(gst_pipeline_);
                gst_pipeline_ = nullptr;
                return false;
            }
            
            RCLCPP_INFO(this->get_logger(), "✅ GStreamer pipeline started successfully");
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception setting up GStreamer pipeline: %s", e.what());
            return false;
        }
    }
    
    void setup_dual_stream_sink() {
        GstElement* appsink = gst_bin_get_by_name(GST_BIN(gst_pipeline_), "dual_sink");
        if (appsink) {
            g_object_set(appsink, "max-buffers", 1, "drop", TRUE, nullptr);
            g_signal_connect(appsink, "new-sample", G_CALLBACK(on_dual_frame_callback), this);
            gst_object_unref(appsink);
        }
    }
    
    void setup_rgb_stream_sink() {
        GstElement* appsink = gst_bin_get_by_name(GST_BIN(gst_pipeline_), "rgb_sink");
        if (appsink) {
            g_object_set(appsink, "max-buffers", 1, "drop", TRUE, nullptr);
            g_signal_connect(appsink, "new-sample", G_CALLBACK(on_rgb_frame_callback), this);
            gst_object_unref(appsink);
        }
    }
    
    void setup_ir_stream_sink() {
        GstElement* appsink = gst_bin_get_by_name(GST_BIN(gst_pipeline_), "ir_sink");
        if (appsink) {
            g_object_set(appsink, "max-buffers", 1, "drop", TRUE, nullptr);
            g_signal_connect(appsink, "new-sample", G_CALLBACK(on_ir_frame_callback), this);
            gst_object_unref(appsink);
        }
    }
    
    // GStreamer callback functions
    static GstFlowReturn on_dual_frame_callback(GstElement* sink, gpointer user_data) {
        auto* node = static_cast<GremsyCameraStreamNode*>(user_data);
        return node->process_dual_frame(sink);
    }
    
    static GstFlowReturn on_rgb_frame_callback(GstElement* sink, gpointer user_data) {
        auto* node = static_cast<GremsyCameraStreamNode*>(user_data);
        return node->process_rgb_frame(sink);
    }
    
    static GstFlowReturn on_ir_frame_callback(GstElement* sink, gpointer user_data) {
        auto* node = static_cast<GremsyCameraStreamNode*>(user_data);
        return node->process_ir_frame(sink);
    }
    
    GstFlowReturn process_dual_frame(GstElement* sink) {
        GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) {
            return GST_FLOW_ERROR;
        }
        
        try {
            GstBuffer* buffer = gst_sample_get_buffer(sample);
            GstCaps* caps = gst_sample_get_caps(sample);
            
            // Get video info from caps
            GstVideoInfo video_info;
            if (!gst_video_info_from_caps(&video_info, caps)) {
                gst_sample_unref(sample);
                return GST_FLOW_ERROR;
            }
            
            // Map the buffer for reading
            GstMapInfo map_info;
            if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
                gst_sample_unref(sample);
                return GST_FLOW_ERROR;
            }
            
            // Create OpenCV Mat from buffer data
            cv::Mat frame(video_info.height, video_info.width, CV_8UC3, map_info.data);
            
            // Split dual stream based on layout
            cv::Mat rgb_frame, ir_frame;
            split_dual_frame(frame, rgb_frame, ir_frame);
            
            // Publish both frames
            if (rgb_enabled_ && rgb_stream_publisher_) {
                publish_opencv_image(rgb_frame, rgb_stream_publisher_, "rgb8");
            }
            
            if (ir_enabled_ && ir_stream_publisher_) {
                // Convert RGB IR to grayscale for IR topic
                cv::Mat ir_gray;
                cv::cvtColor(ir_frame, ir_gray, cv::COLOR_RGB2GRAY);
                publish_opencv_image(ir_gray, ir_stream_publisher_, "mono8");
            }
            
            // Cleanup
            gst_buffer_unmap(buffer, &map_info);
            gst_sample_unref(sample);
            
            return GST_FLOW_OK;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing dual frame: %s", e.what());
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }
    
    GstFlowReturn process_rgb_frame(GstElement* sink) {
        GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) {
            return GST_FLOW_ERROR;
        }
        
        try {
            auto image_msg = gst_sample_to_ros_image(sample, "rgb8");
            if (rgb_stream_publisher_) {
                rgb_stream_publisher_->publish(*image_msg);
            }
            
            gst_sample_unref(sample);
            return GST_FLOW_OK;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing RGB frame: %s", e.what());
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }
    
    GstFlowReturn process_ir_frame(GstElement* sink) {
        GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) {
            return GST_FLOW_ERROR;
        }
        
        try {
            auto image_msg = gst_sample_to_ros_image(sample, "mono8");
            if (ir_stream_publisher_) {
                ir_stream_publisher_->publish(*image_msg);
            }
            
            gst_sample_unref(sample);
            return GST_FLOW_OK;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing IR frame: %s", e.what());
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }
    
    void split_dual_frame(const cv::Mat& dual_frame, cv::Mat& rgb_frame, cv::Mat& ir_frame) {
        int width = dual_frame.cols;
        int height = dual_frame.rows;
        
        if (dual_layout_ == "SIDE_BY_SIDE") {
            int half_width = width / 2;
            
            if (rgb_position_ == "LEFT") {
                rgb_frame = dual_frame(cv::Rect(0, 0, half_width, height)).clone();
                ir_frame = dual_frame(cv::Rect(half_width, 0, half_width, height)).clone();
            } else {
                ir_frame = dual_frame(cv::Rect(0, 0, half_width, height)).clone();
                rgb_frame = dual_frame(cv::Rect(half_width, 0, half_width, height)).clone();
            }
        } else if (dual_layout_ == "PICTURE_IN_PICTURE") {
            // Main image (larger one)
            rgb_frame = dual_frame.clone();
            
            // Extract PiP region based on position and size
            int pip_width = static_cast<int>(width * pip_size_);
            int pip_height = static_cast<int>(height * pip_size_);
            
            cv::Rect pip_rect;
            if (pip_position_ == "TOP_RIGHT") {
                pip_rect = cv::Rect(width - pip_width, 0, pip_width, pip_height);
            } else if (pip_position_ == "TOP_LEFT") {
                pip_rect = cv::Rect(0, 0, pip_width, pip_height);
            } else if (pip_position_ == "BOTTOM_RIGHT") {
                pip_rect = cv::Rect(width - pip_width, height - pip_height, pip_width, pip_height);
            } else { // BOTTOM_LEFT
                pip_rect = cv::Rect(0, height - pip_height, pip_width, pip_height);
            }
            
            ir_frame = dual_frame(pip_rect).clone();
        } else {
            // Default: just duplicate the frame
            rgb_frame = dual_frame.clone();
            ir_frame = dual_frame.clone();
        }
    }
    
    void publish_opencv_image(const cv::Mat& cv_image, 
                             rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher,
                             const std::string& encoding) {
        try {
            auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, cv_image).toImageMsg();
            image_msg->header.stamp = this->get_clock()->now();
            image_msg->header.frame_id = "gimbal_camera";
            publisher->publish(*image_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error publishing OpenCV image: %s", e.what());
        }
    }
    
    sensor_msgs::msg::Image::SharedPtr gst_sample_to_ros_image(GstSample* sample, const std::string& encoding) {
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstCaps* caps = gst_sample_get_caps(sample);
        
        // Get video info from caps
        GstVideoInfo video_info;
        if (!gst_video_info_from_caps(&video_info, caps)) {
            throw std::runtime_error("Failed to get video info from caps");
        }
        
        // Map the buffer for reading
        GstMapInfo map_info;
        if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
            throw std::runtime_error("Failed to map GStreamer buffer");
        }
        
        auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
        image_msg->header.stamp = this->get_clock()->now();
        image_msg->header.frame_id = "gimbal_camera";
        image_msg->width = video_info.width;
        image_msg->height = video_info.height;
        image_msg->encoding = encoding;
        image_msg->is_bigendian = false;
        image_msg->step = encoding == "mono8" ? video_info.width : video_info.width * 3;
        
        // Copy data
        size_t data_size = image_msg->step * image_msg->height;
        image_msg->data.resize(data_size);
        std::memcpy(image_msg->data.data(), map_info.data, data_size);
        
        gst_buffer_unmap(buffer, &map_info);
        
        return image_msg;
    }
    
    void cleanup_gstreamer() {
        if (gst_pipeline_) {
            gst_element_set_state(gst_pipeline_, GST_STATE_NULL);
            gst_object_unref(gst_pipeline_);
            gst_pipeline_ = nullptr;
            RCLCPP_INFO(this->get_logger(), "🎬 GStreamer pipeline cleaned up");
        }
    }
    
    void publish_streams() {
        if (!connection_established_ || !payload_interface_ || shutdown_requested_.load()) {
            return;
        }
        
        switch(streaming_state_) {
            case idle:
                break;
            case check_camera_info: {
                RCLCPP_INFO(this->get_logger(), "Requesting camera information...");
                payload_interface_->getPayloadCameraInformation();
                streaming_state_ = idle; // Wait for callback
                break;
            }
            case check_streaming_uri: {
                if (!stream_info_requested_) {
                    payload_interface_->getPayloadCameraStreamingInformation();
                    stream_info_requested_ = true;
                }
                break;
            }
            case stream_info_received: {
                publish_stream_status();
                if (!stream_active_notified_) {
                    log_stream_info();
                    
                    // Start GStreamer pipeline for frame processing
                    if (setup_gstreamer_pipeline()) {
                        streaming_state_ = processing_frames;
                        stream_active_notified_ = true;
                        
                        // Start recording if auto-record is enabled
                        if (auto_record_) {
                            start_recording();
                        }
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to setup GStreamer pipeline");
                        streaming_state_ = idle;
                    }
                }
                break;
            }
            case processing_frames: {
                publish_stream_status();
                publish_camera_info();
                // Frame processing happens in GStreamer callbacks
                break;
            }
            default:
                break;
        }
    }
    
    void log_stream_info() {
        RCLCPP_INFO(this->get_logger(), "🎥 Stream is available at: %s", stream_uri_.c_str());
        RCLCPP_INFO(this->get_logger(), "📺 Resolution: %.0fx%.0f", stream_resolution_h_, stream_resolution_v_);
        RCLCPP_INFO(this->get_logger(), "🎬 Stream mode: %s", stream_mode_.c_str());
        
        if (stream_mode_ == "DUAL") {
            RCLCPP_INFO(this->get_logger(), "🔀 Dual stream layout: %s", dual_layout_.c_str());
            RCLCPP_INFO(this->get_logger(), "📍 RGB position: %s, IR position: %s", 
                       rgb_position_.c_str(), ir_position_.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "🎬 Use this command to view the stream:");
        RCLCPP_INFO(this->get_logger(), "   gst-launch-1.0 rtspsrc location=%s ! decodebin ! autovideosink", stream_uri_.c_str());
        RCLCPP_INFO(this->get_logger(), "   OR");
        RCLCPP_INFO(this->get_logger(), "   vlc %s", stream_uri_.c_str());
    }
    
    void start_recording() {
        if (!connection_established_ || !payload_interface_) {
            return;
        }
        
        try {
            RCLCPP_INFO(this->get_logger(), "🔴 Starting auto-recording to: %s", record_path_.c_str());
            // Add recording start logic here based on your SDK
            
            if (record_duration_ > 0) {
                RCLCPP_INFO(this->get_logger(), "⏱️ Auto-stop recording after %d seconds", record_duration_);
                // Set up timer to stop recording
                auto record_timer = this->create_wall_timer(
                    std::chrono::seconds(record_duration_),
                    [this]() {
                        RCLCPP_INFO(this->get_logger(), "⏹️ Auto-stopping recording");
                        // Add recording stop logic here
                    }
                );
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", e.what());
        }
    }
    
    void publish_stream_status() {
        // Publish stream status
        auto status_msg = std_msgs::msg::String();
        if (streaming_state_ == processing_frames) {
            status_msg.data = "✅ Stream active (" + stream_mode_ + "): " + stream_uri_ + 
                             " (" + std::to_string((int)stream_resolution_h_) + "x" + 
                             std::to_string((int)stream_resolution_v_) + ")";
        } else {
            status_msg.data = "🔄 Setting up " + stream_mode_ + " stream...";
        }
        stream_status_publisher_->publish(status_msg);
    }
    
    void publish_camera_info() {
        auto now = this->get_clock()->now();
        
        // Publish RGB camera info
        if (rgb_enabled_ && rgb_camera_info_publisher_) {
            auto rgb_info_msg = create_camera_info_msg(now, "rgb");
            rgb_camera_info_publisher_->publish(rgb_info_msg);
        }
        
        // Publish IR camera info  
        if (ir_enabled_ && ir_camera_info_publisher_) {
            auto ir_info_msg = create_camera_info_msg(now, "ir");
            ir_camera_info_publisher_->publish(ir_info_msg);
        }
    }
    
    sensor_msgs::msg::CameraInfo create_camera_info_msg(const rclcpp::Time& timestamp, const std::string& camera_type) {
        sensor_msgs::msg::CameraInfo info_msg;
        info_msg.header.stamp = timestamp;
        info_msg.header.frame_id = "gimbal_camera_" + camera_type;
        
        // Set camera resolution based on stream mode and camera type
        if (stream_mode_ == "DUAL" && dual_layout_ == "SIDE_BY_SIDE") {
            // For side-by-side dual stream, each individual stream is half width
            info_msg.width = static_cast<uint32_t>(stream_resolution_h_ / 2);
            info_msg.height = static_cast<uint32_t>(stream_resolution_v_);
        } else {
            info_msg.width = static_cast<uint32_t>(stream_resolution_h_);
            info_msg.height = static_cast<uint32_t>(stream_resolution_v_);
        }
        
        // Basic camera parameters (these should be calibrated for your specific camera)
        info_msg.distortion_model = "plumb_bob";
        info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};  // No distortion assumed
        
        // Camera intrinsic matrix K [3x3] - different for RGB vs IR
        double fx, fy, cx, cy;
        
        if (camera_type == "rgb") {
            // RGB camera intrinsics (you should get these from Gremsy SDK)
            fx = info_msg.width * 0.8;   // Approximate focal length for RGB
            fy = info_msg.height * 0.8;  
            cx = info_msg.width / 2.0;   // Principal point x
            cy = info_msg.height / 2.0;  // Principal point y
        } else { // IR camera
            // IR camera intrinsics (usually different from RGB)
            fx = info_msg.width * 0.75;  // IR cameras often have different focal lengths
            fy = info_msg.height * 0.75; 
            cx = info_msg.width / 2.0;   
            cy = info_msg.height / 2.0;  
        }
        
        info_msg.k = {
            fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0
        };
        
        // Rectification matrix R [3x3] (identity for monocular camera)
        info_msg.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
        
        // Projection matrix P [3x4]
        info_msg.p = {
            fx,  0.0, cx,  0.0,
            0.0, fy,  cy,  0.0,
            0.0, 0.0, 1.0, 0.0
        };
        
        return info_msg;
    }
    
    void stream_control_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received stream control command: %s", msg->data.c_str());
        
        if (!connection_established_ || !payload_interface_) {
            RCLCPP_WARN(this->get_logger(), "Cannot control stream - not connected");
            return;
        }
        
        try {
            if (msg->data == "START_RGB") {
                stream_mode_ = "RGB";
                rgb_enabled_ = true;
                ir_enabled_ = false;
                payload_interface_->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
                restart_streaming();
                RCLCPP_INFO(this->get_logger(), "RGB streaming enabled");
            } else if (msg->data == "START_IR") {
                stream_mode_ = "IR";
                rgb_enabled_ = false;
                ir_enabled_ = true;
                payload_interface_->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IR, PARAM_TYPE_UINT32);
                restart_streaming();
                RCLCPP_INFO(this->get_logger(), "IR streaming enabled");
            } else if (msg->data == "START_DUAL") {
                stream_mode_ = "DUAL";
                rgb_enabled_ = true;
                ir_enabled_ = true;
                payload_interface_->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IREO, PARAM_TYPE_UINT32);
                restart_streaming();
                RCLCPP_INFO(this->get_logger(), "Dual stream (RGB+IR) enabled");
            } else if (msg->data == "GET_STREAM_INFO") {
                if (streaming_state_ == processing_frames) {
                    log_stream_info();
                } else {
                    RCLCPP_INFO(this->get_logger(), "⏸️ No active stream");
                }
            } else if (msg->data == "START_RECORDING") {
                start_recording();
            } else if (msg->data == "STOP_RECORDING") {
                RCLCPP_INFO(this->get_logger(), "⏹️ Stopping recording");
                // Add recording stop logic here
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown command: %s", msg->data.c_str());
                RCLCPP_INFO(this->get_logger(), "Available commands: START_RGB, START_IR, START_DUAL, GET_STREAM_INFO, START_RECORDING, STOP_RECORDING");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute stream control: %s", e.what());
        }
    }
    
    void restart_streaming() {
        cleanup_gstreamer();
        streaming_state_ = check_camera_info;
        stream_info_requested_ = false;
        stream_info_received_ = false;
        stream_active_notified_ = false;
    }

public:
    void handlePayloadStatusChanged(int event, double* param) {
        if (shutdown_requested_.load()) return;
        
        std::lock_guard<std::mutex> lock(callback_mutex_);
        
        switch(event) {
            case PAYLOAD_CAM_INFO: {
                if((int16_t)param[0] & (CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM)) {
                    RCLCPP_INFO(this->get_logger(), "✅ Payload has video streaming capability");
                    if (streaming_state_ == idle) {
                        streaming_state_ = check_streaming_uri;
                        stream_info_requested_ = false;
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "❌ Payload has no video streaming");
                    streaming_state_ = idle;
                }
                break;
            }
            case PAYLOAD_CAM_STORAGE_INFO: {
                RCLCPP_INFO(this->get_logger(), "💾 Storage: %.1f MB available (%.1f MB total, %.1f MB used)", 
                           param[2], param[0], param[1]);
                
                // Publish storage info as ROS topic
                auto storage_msg = std_msgs::msg::Float64MultiArray();
                storage_msg.data = {param[0], param[1], param[2], param[3]}; // total, used, available, status
                storage_info_publisher_->publish(storage_msg);
                break;
            }
            default:
                break;
        }
    }
    
    void handlePayloadStreamChanged(int event, char* param_char, double* param_double) {
        if (shutdown_requested_.load()) return;
        
        std::lock_guard<std::mutex> lock(callback_mutex_);
        
        switch(event) {
            case PAYLOAD_CAM_STREAMINFO: {
                if (streaming_state_ == check_streaming_uri && !stream_info_received_) {
                    stream_type_ = param_double[0];
                    stream_resolution_v_ = param_double[1];
                    stream_resolution_h_ = param_double[2];
                    stream_uri_ = string(param_char);
                    
                    RCLCPP_INFO(this->get_logger(), "📺 Stream Info Received:");
                    RCLCPP_INFO(this->get_logger(), "  Type: %.0f (RTSP=%s)", 
                               stream_type_, (stream_type_ == VIDEO_STREAM_TYPE_RTSP) ? "Yes" : "No");
                    RCLCPP_INFO(this->get_logger(), "  Resolution: %.0fx%.0f", 
                               stream_resolution_h_, stream_resolution_v_);
                    RCLCPP_INFO(this->get_logger(), "  URI: %s", stream_uri_.c_str());
                    
                    is_rtsp_stream_ = (stream_type_ == VIDEO_STREAM_TYPE_RTSP);
                    streaming_state_ = stream_info_received;
                    stream_info_received_ = true;
                }
                break;
            }
            default:
                break;
        }
    }

private:
    // Common parameters
    std::string udp_ip_target_, uart_port_, log_level_;
    int udp_port_target_, uart_baudrate_, control_method_;
    double timeout_;
    
    // Topic names
    std::string rgb_stream_topic_, ir_stream_topic_, rgb_camera_info_topic_, ir_camera_info_topic_;
    std::string stream_control_topic_, stream_status_topic_, storage_info_topic_;
    
    // Camera configuration from config file
    std::string stream_mode_;
    bool rgb_enabled_, ir_enabled_, auto_start_stream_;
    std::string stream_quality_, encoding_;
    int framerate_;
    double publish_rate_;
    
    // Dual stream configuration
    std::string dual_layout_, rgb_position_, ir_position_, pip_position_;
    double pip_size_;
    
    // Stream URLs (can be overridden in config)
    std::string rgb_url_, ir_url_, dual_url_;
    
    // Recording configuration
    bool auto_record_;
    int record_duration_;
    std::string record_path_;
    
    // State
    std::atomic<bool> shutdown_requested_{false};
    bool connection_established_ = false;
    stream_sequence_t streaming_state_ = idle;
    bool stream_info_requested_ = false;
    bool stream_info_received_ = false;
    bool stream_active_notified_ = false;
    
    // Thread safety
    std::mutex callback_mutex_;
    
    // Stream info
    bool is_rtsp_stream_ = false;
    string stream_uri_;
    double stream_type_ = 0, stream_resolution_v_ = 0, stream_resolution_h_ = 0;

    // SDK
    PayloadSdkInterface* payload_interface_ = nullptr;
    
    // GStreamer
    GstElement* gst_pipeline_ = nullptr;

    // ROS2 publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_stream_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_stream_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_camera_info_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ir_camera_info_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr storage_info_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stream_status_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stream_control_subscriber_;
    rclcpp::TimerBase::SharedPtr stream_timer_;
};

void onPayloadStatusChanged(int event, double* param) {
    std::lock_guard<std::mutex> lock(g_instance_mutex);
    if (g_stream_node_instance) {
        g_stream_node_instance->handlePayloadStatusChanged(event, param);
    }
}

void onPayloadStreamChanged(int event, char* param_char, double* param_double) {
    std::lock_guard<std::mutex> lock(g_instance_mutex);
    if (g_stream_node_instance) {
        g_stream_node_instance->handlePayloadStreamChanged(event, param_char, param_double);
    }
}

void quit_handler(int sig) {
    printf("\nTERMINATING CAMERA STREAM NODE AT USER REQUEST\n");
    std::lock_guard<std::mutex> lock(g_instance_mutex);
    if (g_stream_node_instance) {
        rclcpp::shutdown();
    }
    exit(0);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    signal(SIGINT, quit_handler);
    
    try {
        auto node = std::make_shared<GremsyCameraStreamNode>();
        
        RCLCPP_INFO(node->get_logger(), "🎥 Gremsy Camera Stream Node is running...");
        RCLCPP_INFO(node->get_logger(), "📡 Stream commands: START_RGB, START_IR, START_DUAL, GET_STREAM_INFO, START_RECORDING, STOP_RECORDING");
        RCLCPP_INFO(node->get_logger(), "📺 Stream info published to: /gimbal/camera/stream_status");
        RCLCPP_INFO(node->get_logger(), "💾 Storage info published to: /gimbal/camera/storage_info");
        RCLCPP_INFO(node->get_logger(), "🎬 RGB images published to: /gimbal/camera/rgb/image");
        RCLCPP_INFO(node->get_logger(), "🌡️ IR images published to: /gimbal/camera/ir/image");
        RCLCPP_INFO(node->get_logger(), "📷 RGB camera info published to: /gimbal/camera/rgb/camera_info");
        RCLCPP_INFO(node->get_logger(), "📷 IR camera info published to: /gimbal/camera/ir/camera_info");
        RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit");
        
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("gremsy_camera_stream_node"), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}