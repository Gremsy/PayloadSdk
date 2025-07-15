#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <signal.h>
#include <pthread.h>
#include <cstdlib>
#include <string>

// Include the payload SDK interface
#include "payloadSdkInterface.h"

using namespace std::chrono_literals;
using namespace std;

class GremsyCameraMoveNode : public rclcpp::Node
{
public:
    GremsyCameraMoveNode() : Node("gremsy_camera_move_node")
    {
        declare_common_parameters();
        load_common_parameters();
        
        // Initialize payload connection
        initialize_payload_connection();
        
        setup_publishers_subscribers();
        
        RCLCPP_INFO(this->get_logger(), "Gremsy Camera Move Node initialized");
    }
    
    ~GremsyCameraMoveNode() {
        cleanup_payload_connection();
    }

private:
    void declare_common_parameters() {
        // Network Configuration
        this->declare_parameter("network.udp_ip_target", "10.4.1.13");
        this->declare_parameter("network.udp_port_target", 14566);
        
        // UART Configuration
        this->declare_parameter("uart.port", "/dev/ttyUSB0");
        this->declare_parameter("uart.baudrate", 115200);
        
        // Connection Method
        this->declare_parameter("connection.control_method", 1);
        
        // Movement settings
        this->declare_parameter("movement.max_speed", 90.0);
        this->declare_parameter("movement.smoothing", true);
        
        // Node configuration
        this->declare_parameter("node.publish_rate", 100.0);
    }
    
    void load_common_parameters() {
        // Load network parameters
        udp_ip_target_ = this->get_parameter("network.udp_ip_target").as_string();
        udp_port_target_ = this->get_parameter("network.udp_port_target").as_int();
        
        // Load UART parameters
        uart_port_ = this->get_parameter("uart.port").as_string();
        uart_baudrate_ = this->get_parameter("uart.baudrate").as_int();
        
        // Load connection method
        control_method_ = this->get_parameter("connection.control_method").as_int();
        
        // Load movement settings
        max_speed_ = this->get_parameter("movement.max_speed").as_double();
        smoothing_ = this->get_parameter("movement.smoothing").as_bool();
        publish_rate_ = this->get_parameter("node.publish_rate").as_double();
    }
    
    void initialize_payload_connection() {
        printf("Starting Gremsy Camera Move Node...\n");
        
        // Setup connection info based on control method
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
            // Create payload SDK object
            payload_interface_ = new PayloadSdkInterface(s_conn);
            
            // Initialize payload connection
            payload_interface_->sdkInitConnection();
            RCLCPP_INFO(this->get_logger(), "Waiting for payload signal!");
            
            // Check payload connection
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
    
    void cleanup_payload_connection() {
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
        // Publishers
        movement_status_publisher_ = this->create_publisher<std_msgs::msg::String>("gimbal/movement/status", 10);
        
        // Subscribers
        move_angle_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gimbal/camera/move", 10,
            std::bind(&GremsyCameraMoveNode::move_angle_callback, this, std::placeholders::_1));
            
        move_speed_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gimbal/camera/move_speed", 10,
            std::bind(&GremsyCameraMoveNode::move_speed_callback, this, std::placeholders::_1));
            
        zoom_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "gimbal/camera/zoom", 10,
            std::bind(&GremsyCameraMoveNode::zoom_callback, this, std::placeholders::_1));
            
        focus_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "gimbal/camera/focus", 10,
            std::bind(&GremsyCameraMoveNode::focus_callback, this, std::placeholders::_1));
        
        // Timer for publishing movement status
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
        status_timer_ = this->create_wall_timer(
            timer_period, std::bind(&GremsyCameraMoveNode::publish_movement_status, this));
    }
    
    void move_angle_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Move Gimbal Angle: roll=%f, pitch=%f, yaw=%f", 
                   msg->x, msg->y, msg->z);
        
        if (!connection_established_ || !payload_interface_) {
            RCLCPP_WARN(this->get_logger(), "Cannot move gimbal - not connected");
            return;
        }
        
        try {
            // Clamp values to safe ranges
            double roll = std::clamp(msg->x, -180.0, 180.0);
            double pitch = std::clamp(msg->y, -90.0, 90.0);
            double yaw = std::clamp(msg->z, -180.0, 180.0);
            
            // Send angle command to gimbal
            payload_interface_->setGimbalSpeed(roll, pitch, yaw, INPUT_ANGLE);
            
            // Update last values
            last_roll_ = roll;
            last_pitch_ = pitch;
            last_yaw_ = yaw;
            last_command_time_ = this->get_clock()->now();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move gimbal: %s", e.what());
        }
    }
    
    void move_speed_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Move Gimbal Speed: roll=%f, pitch=%f, yaw=%f", 
                   msg->x, msg->y, msg->z);
        
        if (!connection_established_ || !payload_interface_) {
            RCLCPP_WARN(this->get_logger(), "Cannot move gimbal - not connected");
            return;
        }
        
        try {
            // Clamp speeds to max_speed
            double roll_speed = std::clamp(msg->x, -max_speed_, max_speed_);
            double pitch_speed = std::clamp(msg->y, -max_speed_, max_speed_);
            double yaw_speed = std::clamp(msg->z, -max_speed_, max_speed_);
            
            // Send speed command to gimbal
            payload_interface_->setGimbalSpeed(roll_speed, pitch_speed, yaw_speed, INPUT_SPEED);
            
            last_command_time_ = this->get_clock()->now();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set gimbal speed: %s", e.what());
        }
    }
    
    void zoom_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Camera Zoom: %f", msg->data);
        
        if (!connection_established_ || !payload_interface_) {
            RCLCPP_WARN(this->get_logger(), "Cannot zoom camera - not connected");
            return;
        }
        
        try {
            float zoom_value = msg->data;
            
            if (zoom_value >= 0.0f && zoom_value <= 100.0f) {
                // Use ZOOM_TYPE_RANGE for percentage-based zoom (0-100%)
                payload_interface_->setCameraZoom(ZOOM_TYPE_RANGE, zoom_value);
                RCLCPP_INFO(this->get_logger(), "✅ Camera zoom set to: %.1f%%", zoom_value);
            } else if (zoom_value > 100.0f) {
                // Use ZOOM_TYPE_STEP for step-based zoom
                int zoom_steps = static_cast<int>(zoom_value - 100.0f);
                for (int i = 0; i < zoom_steps && i < 10; i++) {
                    payload_interface_->setCameraZoom(ZOOM_TYPE_STEP, ZOOM_IN);
                    usleep(200000); // 0.2 second delay between steps
                }
                RCLCPP_INFO(this->get_logger(), "✅ Camera zoomed in %d steps", zoom_steps);
            } else {
                // Negative values zoom out
                int zoom_steps = static_cast<int>(-zoom_value);
                for (int i = 0; i < zoom_steps && i < 10; i++) {
                    payload_interface_->setCameraZoom(ZOOM_TYPE_STEP, ZOOM_OUT);
                    usleep(200000); // 0.2 second delay between steps
                }
                RCLCPP_INFO(this->get_logger(), "✅ Camera zoomed out %d steps", zoom_steps);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to set camera zoom: %s", e.what());
        }
    }
    
    void focus_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Camera Focus: %f", msg->data);
        
        if (!connection_established_ || !payload_interface_) {
            RCLCPP_WARN(this->get_logger(), "Cannot focus camera - not connected");
            return;
        }
        
        try {
            float focus_value = msg->data;
            
            if (focus_value == 0.0f) {
                // Auto focus
                payload_interface_->setCameraFocus(FOCUS_TYPE_AUTO);
                RCLCPP_INFO(this->get_logger(), "✅ Camera set to auto focus");
            } else if (focus_value > 0.0f) {
                // Focus in (positive values)
                int focus_steps = static_cast<int>(focus_value);
                for (int i = 0; i < focus_steps && i < 10; i++) {
                    payload_interface_->setCameraFocus(FOCUS_TYPE_CONTINUOUS, FOCUS_IN);
                    usleep(200000); // 0.2 second delay between steps
                }
                payload_interface_->setCameraFocus(FOCUS_TYPE_CONTINUOUS, FOCUS_STOP);
                RCLCPP_INFO(this->get_logger(), "✅ Camera focused in %d steps", focus_steps);
            } else {
                // Focus out (negative values)
                int focus_steps = static_cast<int>(-focus_value);
                for (int i = 0; i < focus_steps && i < 10; i++) {
                    payload_interface_->setCameraFocus(FOCUS_TYPE_CONTINUOUS, FOCUS_OUT);
                    usleep(200000); // 0.2 second delay between steps
                }
                payload_interface_->setCameraFocus(FOCUS_TYPE_CONTINUOUS, FOCUS_STOP);
                RCLCPP_INFO(this->get_logger(), "✅ Camera focused out %d steps", focus_steps);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to set camera focus: %s", e.what());
        }
    }
    
    void publish_movement_status() {
        auto status_msg = std_msgs::msg::String();
        
        // Check if recent commands were received
        auto now = this->get_clock()->now();
        auto time_since_last_cmd = (now - last_command_time_).seconds();
        
        if (time_since_last_cmd < 1.0) {
            status_msg.data = "Moving - Last position: roll=" + std::to_string(last_roll_) + 
                             ", pitch=" + std::to_string(last_pitch_) + 
                             ", yaw=" + std::to_string(last_yaw_);
        } else {
            status_msg.data = "Idle - No recent movement commands";
        }
        
        movement_status_publisher_->publish(status_msg);
    }

    // Parameters
    std::string udp_ip_target_;
    int udp_port_target_;
    std::string uart_port_;
    int uart_baudrate_;
    int control_method_;
    double max_speed_;
    bool smoothing_;
    double publish_rate_;
    
    // Connection state
    bool connection_established_ = false;

    // State tracking
    double last_roll_ = 0.0;
    double last_pitch_ = 0.0;
    double last_yaw_ = 0.0;
    rclcpp::Time last_command_time_;

    // Payload interface
    PayloadSdkInterface* payload_interface_ = nullptr;

    // ROS2 objects
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr movement_status_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr move_angle_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr move_speed_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr zoom_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr focus_subscriber_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

// Signal handler for clean shutdown
GremsyCameraMoveNode* g_move_node_instance = nullptr;

void quit_handler(int sig) {
    printf("\n");
    printf("TERMINATING CAMERA MOVE NODE AT USER REQUEST \n");
    printf("\n");

    if (g_move_node_instance) {
        rclcpp::shutdown();
    }
    
    exit(0);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // Set up signal handler
    signal(SIGINT, quit_handler);
    
    try {
        auto node = std::make_shared<GremsyCameraMoveNode>();
        g_move_node_instance = node.get();
        
        RCLCPP_INFO(node->get_logger(), "🎮 Gremsy Camera Move Node is running...");
        RCLCPP_INFO(node->get_logger(), "Movement topics: /gimbal/camera/move, /gimbal/camera/move_speed");
        RCLCPP_INFO(node->get_logger(), "Zoom topic: /gimbal/camera/zoom (0-100%% range, >100 step in, <0 step out)");
        RCLCPP_INFO(node->get_logger(), "Focus topic: /gimbal/camera/focus (0=auto, >0 focus in, <0 focus out)");
        RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit");
        
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("gremsy_camera_move_node"), "Exception in main: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("gremsy_camera_move_node"), "Unknown exception in main");
    }
    
    rclcpp::shutdown();
    return 0;
}
