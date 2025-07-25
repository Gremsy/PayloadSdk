#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <signal.h>
#include <pthread.h>
#include <cstdlib>
#include <string>

// Include the payload SDK interface
#include "payloadSdkInterface.h"

using namespace std::chrono_literals;
using namespace std;

class GremsyGimbalNode : public rclcpp::Node
{
public:
    GremsyGimbalNode() : Node("gremsy_gimbal_node")
    {
        declare_common_parameters();
        declare_node_parameters();
        load_common_parameters();
        load_node_parameters();
        
        // Initialize payload connection
        initialize_payload_connection();
        
        setup_publishers_subscribers();
        
        // Set initial gimbal mode
        set_initial_gimbal_mode();
        
        RCLCPP_INFO(this->get_logger(), "Gremsy Gimbal Node initialized");
    }
    
    ~GremsyGimbalNode() {
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
        
        // MAVLink Configuration
        this->declare_parameter("mavlink.payload_system_id", 1);
        this->declare_parameter("mavlink.payload_component_id", 100);
        this->declare_parameter("mavlink.gimbal_system_id", 1);
        this->declare_parameter("mavlink.gimbal_component_id", 154);
        
        // Node Configuration
        this->declare_parameter("node.timeout", 5.0);
        
        // Logging Configuration
        this->declare_parameter("logging.level", "INFO");

        // Add default position parameters
        this->declare_parameter("gimbal.default_position.enable", true);
        this->declare_parameter("gimbal.default_position.pitch", 0.0);
        this->declare_parameter("gimbal.default_position.roll", -45.0);
        this->declare_parameter("gimbal.default_position.yaw", 0.0);
        this->declare_parameter("gimbal.default_position.delay_ms", 2000);
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
        
        // Load MAVLink parameters
        payload_system_id_ = this->get_parameter("mavlink.payload_system_id").as_int();
        payload_component_id_ = this->get_parameter("mavlink.payload_component_id").as_int();
        gimbal_system_id_ = this->get_parameter("mavlink.gimbal_system_id").as_int();
        gimbal_component_id_ = this->get_parameter("mavlink.gimbal_component_id").as_int();
        
        // Load other parameters
        timeout_ = this->get_parameter("node.timeout").as_double();
        log_level_ = this->get_parameter("logging.level").as_string();
    }
    
    void initialize_payload_connection() {
        printf("Starting Gremsy Gimbal Node...\n");
        
        // Setup connection info based on control method - FIXED: Use reference code structure
        T_ConnInfo s_conn;
        
        if (control_method_ == CONTROL_UDP) {
            s_conn = {
                CONTROL_UDP,
                const_cast<char*>(udp_ip_target_.c_str()),
                udp_port_target_
            };
            
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
            // Create payload SDK object - FIXED: Use reference code pattern
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
    
    void declare_node_parameters() {
        // Topics
        this->declare_parameter("topics.gimbal_status", "gimbal/status");
        this->declare_parameter("topics.gimbal_attitude", "gimbal/attitude");
        this->declare_parameter("topics.gimbal_encoder", "gimbal/encoder");
        this->declare_parameter("topics.gimbal_imu", "gimbal/imu");
        this->declare_parameter("topics.gimbal_motor", "gimbal/motor");
        this->declare_parameter("topics.gimbal_mode_cmd", "gimbal/mode/cmd");
        this->declare_parameter("topics.move_gimbal_angle", "move_gimbal_angle");
        
        // Gimbal settings
        this->declare_parameter("gimbal.lock_mode", 0);
        this->declare_parameter("gimbal.input_mode", 1);
        this->declare_parameter("gimbal.default_mode", "FOLLOW");
        
        // Node configuration
        this->declare_parameter("node.publish_rate", 50.0);
    }
    
    void load_node_parameters() {
        // Load topic names
        gimbal_status_topic_ = this->get_parameter("topics.gimbal_status").as_string();
        gimbal_attitude_topic_ = this->get_parameter("topics.gimbal_attitude").as_string();
        gimbal_encoder_topic_ = this->get_parameter("topics.gimbal_encoder").as_string();
        gimbal_imu_topic_ = this->get_parameter("topics.gimbal_imu").as_string();
        gimbal_motor_topic_ = this->get_parameter("topics.gimbal_motor").as_string();
        gimbal_mode_cmd_topic_ = this->get_parameter("topics.gimbal_mode_cmd").as_string();
        move_gimbal_angle_topic_ = this->get_parameter("topics.move_gimbal_angle").as_string();
        
        // Load gimbal settings
        lock_mode_ = this->get_parameter("gimbal.lock_mode").as_int();
        input_mode_ = this->get_parameter("gimbal.input_mode").as_int();
        default_mode_ = this->get_parameter("gimbal.default_mode").as_string();
        
        // Load node configuration
        publish_rate_ = this->get_parameter("node.publish_rate").as_double();

        // Load default position parameters
        default_position_enabled_ = this->get_parameter("gimbal.default_position.enable").as_bool();
        default_pitch_ = this->get_parameter("gimbal.default_position.pitch").as_double();
        default_roll_ = this->get_parameter("gimbal.default_position.roll").as_double();
        default_yaw_ = this->get_parameter("gimbal.default_position.yaw").as_double();
        default_position_delay_ = this->get_parameter("gimbal.default_position.delay_ms").as_int();
    }
    
    void setup_publishers_subscribers() {
        // Publishers
        status_publisher_ = this->create_publisher<std_msgs::msg::String>(gimbal_status_topic_, 10);
        attitude_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(gimbal_attitude_topic_, 10);
        encoder_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(gimbal_encoder_topic_, 10);
        imu_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(gimbal_imu_topic_, 10);
        motor_publisher_ = this->create_publisher<std_msgs::msg::String>(gimbal_motor_topic_, 10);
        
        // Subscribers
        mode_cmd_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            gimbal_mode_cmd_topic_, 10,
            std::bind(&GremsyGimbalNode::mode_cmd_callback, this, std::placeholders::_1));
            
        // FIXED: Add gimbal movement subscription like reference code
        move_gimbal_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            move_gimbal_angle_topic_, 10,
            std::bind(&GremsyGimbalNode::move_gimbal_callback, this, std::placeholders::_1));
        
        // Timer for publishing status
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
        status_timer_ = this->create_wall_timer(
            timer_period, std::bind(&GremsyGimbalNode::publish_status, this));
    }
    
    void set_initial_gimbal_mode() {
        if (!connection_established_ || !payload_interface_) {
            RCLCPP_WARN(this->get_logger(), "Cannot set gimbal mode - not connected");
            return;
        }
        
        try {
            // FIRST: Set RC mode (like in the working example)
            RCLCPP_INFO(this->get_logger(), "Setting gimbal RC mode to STANDARD");
            payload_interface_->setPayloadCameraParam(
                PAYLOAD_CAMERA_RC_MODE, 
                PAYLOAD_CAMERA_RC_MODE_STANDARD, 
                PARAM_TYPE_UINT32);
            usleep(100000); // 100ms delay like in example
            
            RCLCPP_INFO(this->get_logger(), "Setting gimbal to %s mode", default_mode_.c_str());
            
            // THEN: Set gimbal mode
            if (default_mode_ == "FOLLOW") {
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_GIMBAL_MODE,
                    PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW,
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "✅ Gimbal set to FOLLOW mode - will move with drone body");
            } else if (default_mode_ == "LOCK") {
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_GIMBAL_MODE,
                    PAYLOAD_CAMERA_GIMBAL_MODE_LOCK,
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "✅ Gimbal set to LOCK mode");
            } else if (default_mode_ == "MAPPING") {
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_GIMBAL_MODE,
                    PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING,
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "✅ Gimbal set to MAPPING mode");
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown mode %s, using FOLLOW", default_mode_.c_str());
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_GIMBAL_MODE,
                    PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW,
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "✅ Gimbal set to FOLLOW mode (default)");
            }
            
            usleep(100000); // 100ms delay like in example
            
            // FINALLY: Set default position if enabled
            if (default_position_enabled_) {
                RCLCPP_INFO(this->get_logger(), "Setting default position after mode setup");
                set_default_position_delayed();
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to set gimbal mode: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "❌ Unknown error setting gimbal mode");
        }
    }

    void publish_status() {
        if (!connection_established_ || !payload_interface_) {
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "❌ Gimbal disconnected - Check network connection";
            status_publisher_->publish(status_msg);
            return;
        }
        
        // Publish general status
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "✅ Gimbal in " + default_mode_ + " mode | Connection: " + 
                         (control_method_ == CONTROL_UDP ? 
                         ("UDP " + udp_ip_target_ + ":" + std::to_string(udp_port_target_)) :
                         ("UART " + uart_port_));
        status_publisher_->publish(status_msg);
        
        if (log_level_ == "DEBUG") {
            RCLCPP_DEBUG(this->get_logger(), "Published gimbal status");
        }
    }
    
    // FIXED: Add gimbal movement callback like reference code
    void move_gimbal_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        if (!connection_established_ || !payload_interface_) {
            RCLCPP_WARN(this->get_logger(), "Cannot move gimbal - not connected");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Move Gimbal Request: roll=%f pitch=%f yaw=%f", msg->x, msg->y, msg->z);
        
        try {
            // Use exact same pattern as working example
            payload_interface_->setGimbalSpeed(msg->x, msg->y, msg->z, INPUT_ANGLE);
            RCLCPP_INFO(this->get_logger(), "✅ Gimbal movement command sent");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to move gimbal: %s", e.what());
        }
    }
    
    void mode_cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received mode command: %s", msg->data.c_str());
        
        if (!connection_established_ || !payload_interface_) {
            RCLCPP_WARN(this->get_logger(), "Cannot change mode - not connected");
            return;
        }
        
        try {
            if (msg->data == "FOLLOW") {
                // 🎯 SET GIMBAL TO FOLLOW MODE - this makes it move with drone body
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_GIMBAL_MODE,           // Parameter name "GB_MODE"
                    PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW,    // Value = 2 (follow vehicle)
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "✅ Gimbal set to FOLLOW mode - will move with drone body");
                
                // ADD: Set default position after setting to FOLLOW mode
                if (default_position_enabled_) {
                    RCLCPP_INFO(this->get_logger(), "Mode changed to FOLLOW - setting default position");
                    set_default_position_delayed();
                }
                
            } else if (msg->data == "LOCK") {
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_GIMBAL_MODE,
                    PAYLOAD_CAMERA_GIMBAL_MODE_LOCK,
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "✅ Gimbal set to LOCK mode");
            } else if (msg->data == "MAPPING") {
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_GIMBAL_MODE,
                    PAYLOAD_CAMERA_GIMBAL_MODE_MAPPING,
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "✅ Gimbal set to MAPPING mode");
            } else if (msg->data == "OFF") {
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_GIMBAL_MODE,
                    PAYLOAD_CAMERA_GIMBAL_MODE_OFF,
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "✅ Gimbal turned OFF");
            } else if (msg->data == "RESET") {
                RCLCPP_WARN(this->get_logger(), "Resetting gimbal - this may take several seconds");
                payload_interface_->setPayloadCameraParam(
                    PAYLOAD_CAMERA_GIMBAL_MODE,
                    PAYLOAD_CAMERA_GIMBAL_MODE_RESET,
                    PARAM_TYPE_UINT32);
                RCLCPP_INFO(this->get_logger(), "✅ Gimbal RESET command sent");
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown gimbal mode: %s", msg->data.c_str());
                RCLCPP_INFO(this->get_logger(), "Available modes: FOLLOW, LOCK, MAPPING, OFF, RESET");
                return;
            }
            
            default_mode_ = msg->data;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to change gimbal mode: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "❌ Unknown error changing gimbal mode");
        }
    }

    void set_default_position_delayed() {
        // Create a one-shot timer to set position after delay
        auto timer = this->create_wall_timer(
            std::chrono::milliseconds(default_position_delay_),
            [this]() {
                set_default_position();
            });
            
        // Store timer to prevent it from being destroyed
        position_timer_ = timer;
    }
    
    void set_default_position() {
        if (!connection_established_ || !payload_interface_) {
            RCLCPP_WARN(this->get_logger(), "Cannot set default position - not connected");
            return;
        }
        
        try {
            RCLCPP_INFO(this->get_logger(), "Setting gimbal to default position: roll=%.1f°, pitch=%.1f°, yaw=%.1f°", 
                    default_roll_, default_pitch_, default_yaw_);
            
            // Use exact same pattern as working example
            payload_interface_->setGimbalSpeed(default_roll_, default_pitch_, default_yaw_, INPUT_ANGLE);
            
            RCLCPP_INFO(this->get_logger(), "✅ Gimbal positioned: roll=%.1f°, pitch=%.1f°, yaw=%.1f°", 
                    default_roll_, default_pitch_, default_yaw_);
            
            // Cancel the timer since we only want this to run once
            if (position_timer_) {
                position_timer_->cancel();
                position_timer_.reset();
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to set default position: %s", e.what());
        }
    }

    // Add these member variables:
    bool default_position_enabled_ = true;
    double default_pitch_ = -45.0;
    double default_roll_ = 0.0;
    double default_yaw_ = 0.0;
    int default_position_delay_ = 2000;  // 2 seconds delay
    rclcpp::TimerBase::SharedPtr position_timer_;

    // Common parameters (previously in base class)
    std::string udp_ip_target_;
    int udp_port_target_;
    std::string uart_port_;
    int uart_baudrate_;
    int control_method_;
    double timeout_;
    std::string log_level_;
    
    // MAVLink parameters
    int payload_system_id_;
    int payload_component_id_;
    int gimbal_system_id_;
    int gimbal_component_id_;

    // Node-specific parameters
    std::string gimbal_status_topic_;
    std::string gimbal_attitude_topic_;
    std::string gimbal_encoder_topic_;
    std::string gimbal_imu_topic_;
    std::string gimbal_motor_topic_;
    std::string gimbal_mode_cmd_topic_;
    std::string move_gimbal_angle_topic_;
    int lock_mode_;
    int input_mode_;
    std::string default_mode_;
    double publish_rate_;
    
    // Connection state
    bool connection_established_ = false;

    // Payload interface
    PayloadSdkInterface* payload_interface_ = nullptr;

    // ROS2 objects
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr encoder_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_cmd_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr move_gimbal_subscriber_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

// Signal handler for clean shutdown
GremsyGimbalNode* g_node_instance = nullptr;

void quit_handler(int sig) {
    printf("\n");
    printf("TERMINATING AT USER REQUEST \n");
    printf("\n");

    if (g_node_instance) {
        // The destructor will handle cleanup
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
        auto node = std::make_shared<GremsyGimbalNode>();
        g_node_instance = node.get();
        
        RCLCPP_INFO(node->get_logger(), "🚀 Gremsy Gimbal Node is running...");
        RCLCPP_INFO(node->get_logger(), "Available modes: FOLLOW, LOCK, MAPPING, OFF, RESET");
        RCLCPP_INFO(node->get_logger(), "📡 Mode control topic: /gimbal/mode/cmd");
        RCLCPP_INFO(node->get_logger(), "🎮 Gimbal movement topic: /move_gimbal_angle");
        RCLCPP_INFO(node->get_logger(), "📊 Status topic: /gimbal/status");
        RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit");
        
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("gremsy_gimbal_node"), "Exception in main: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("gremsy_gimbal_node"), "Unknown exception in main");
    }
    
    rclcpp::shutdown();
    return 0;
}