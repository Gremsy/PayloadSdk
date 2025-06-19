/**
 * @file gremsy_heading_follower.cpp
 * @brief Gremsy Gimbal ROS2 Node with Drone Heading Following
 * @description Controls Gremsy VIO gimbal and automatically follows drone heading
 * @author Your Name
 * @date 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <atomic>
#include <algorithm>
#include <cmath>

// Include your PayloadSDK headers
#include "../libs/payloadsdk.h"
#include "../libs/payloadSdkInterface.h"

// Include the mavlink interface headers
#include "../libs/third-party/mavlink/include/autopilot_interface.h"
#include "../libs/third-party/mavlink/include/serial_port.h"
#include "../libs/third-party/mavlink/include/udp_port.h"

#if defined VIO
#include "../libs/payload-define/vio_sdk.h"
#endif

// Control method constants
#define CONTROL_UART 0
#define CONTROL_UDP 1

// Utility functions for angle conversion
inline double to_rad(double deg) { return deg * M_PI / 180.0; }
inline double to_deg(double rad) { return rad * 180.0 / M_PI; }

class GremsyROS2Node : public rclcpp::Node
{
public:
    GremsyROS2Node() : Node("gremsy_gimbal_node"),
                       gimbal_port_(nullptr),
                       payload_interface_(nullptr),
                       gimbal_connected_(false),
                       last_heartbeat_(std::chrono::steady_clock::now()),
                       drone_heading_(0.0),
                       drone_heading_valid_(false),
                       heading_follow_enabled_(false),
                       last_heading_update_(std::chrono::steady_clock::now())
    {
        // Declare and load parameters
        declareParameters();
        loadParameters();

        // Initialize publishers and subscribers
        initializeTopics();

        // Initialize gimbal connection
        initializeGimbal();

        // Start gimbal communication thread
        gimbal_thread_ = std::thread(&GremsyROS2Node::gimbalLoop, this);

        // Create timer for status publishing
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&GremsyROS2Node::publishStatus, this)
        );

        // Create timer for heading follow mode
        heading_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz for heading updates
            std::bind(&GremsyROS2Node::updateHeadingFollow, this)
        );

        RCLCPP_INFO(this->get_logger(), "Gremsy ROS2 Node with heading integration initialized successfully");
    }

    ~GremsyROS2Node()
    {
        gimbal_running_ = false;
        if (gimbal_thread_.joinable()) {
            gimbal_thread_.join();
        }

        // Cleanup gimbal connection
        if (payload_interface_) {
            delete payload_interface_;
        }
        if (gimbal_port_) {
            gimbal_port_->stop();
            delete gimbal_port_;
        }
    }

private:
    // Configuration parameters
    std::string udp_ip_target_;
    int udp_port_target_;
    std::string uart_port_;
    int uart_baudrate_;
    int control_method_;
    double publish_rate_;
    double timeout_;

    // Environment parameters
    std::string robot_name_;
    bool auto_namespace_;

    // Topic names (base names from config, will be prefixed with robot name)
    std::string move_gimbal_angle_topic_;
    std::string gimbal_status_topic_;
    std::string gimbal_attitude_topic_;
    std::string gimbal_command_topic_;
    std::string gimbal_home_topic_;
    std::string gimbal_reboot_topic_;
    std::string gimbal_imu_topic_;
    std::string drone_heading_topic_base_;
    std::string drone_odom_topic_base_;
    std::string drone_imu_topic_base_;
    std::string heading_follow_topic_;

    // Full topic names (with robot namespace)
    std::string drone_heading_topic_full_;
    std::string drone_odom_topic_full_;
    std::string drone_imu_topic_full_;

    // MAVLink parameters
    int payload_system_id_;
    int payload_component_id_;
    int gimbal_system_id_;
    int gimbal_component_id_;

    // Gimbal parameters
    int input_mode_;
    double max_speed_;
    double acceleration_;
    int smoothing_;

    // Angle limits
    double pan_min_, pan_max_;
    double tilt_min_, tilt_max_;
    double roll_min_, roll_max_;

    // Heading follow parameters
    bool enable_heading_follow_;
    double heading_offset_;
    double heading_deadband_;
    double heading_gain_;

    // ROS2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heading_status_pub_;

    // ROS2 Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr home_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reboot_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drone_heading_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr drone_imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heading_follow_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr heading_timer_;

    // Gimbal SDK objects
    Generic_Port *gimbal_port_;
    PayloadSdkInterface *payload_interface_;

    // Threading
    std::thread gimbal_thread_;
    std::atomic<bool> gimbal_running_{true};
    std::atomic<bool> gimbal_connected_;
    std::chrono::steady_clock::time_point last_heartbeat_;

    // Current gimbal state
    attitude<float> current_attitude_;

    // Drone heading state
    std::atomic<double> drone_heading_;
    std::atomic<bool> drone_heading_valid_;
    std::atomic<bool> heading_follow_enabled_;
    std::chrono::steady_clock::time_point last_heading_update_;

    void declareParameters()
    {
        // Network parameters
        this->declare_parameter("network.udp_ip_target", "10.4.1.33");
        this->declare_parameter("network.udp_port_target", 14566);

        // UART parameters
        this->declare_parameter("uart.port", "/dev/ttyUSB0");
        this->declare_parameter("uart.baudrate", 115200);

        // Connection method
        this->declare_parameter("connection.control_method", 1);

        // Environment parameters
        this->declare_parameter("environment.robot_name", "robot_1");
        this->declare_parameter("environment.auto_namespace", true);

        // Topic names
        this->declare_parameter("topics.move_gimbal_angle", "move_gimbal_angle");
        this->declare_parameter("topics.gimbal_status", "gimbal/status");
        this->declare_parameter("topics.gimbal_attitude", "gimbal/attitude");
        this->declare_parameter("topics.gimbal_command", "gimbal/command");
        this->declare_parameter("topics.gimbal_home", "gimbal/home");
        this->declare_parameter("topics.gimbal_reboot", "gimbal/reboot");
        this->declare_parameter("topics.gimbal_imu", "gimbal/imu");
        this->declare_parameter("topics.heading_follow", "gimbal/heading_follow");

        // Drone topics (base names, will be prefixed with robot name)
        this->declare_parameter("topics.drone_heading", "interface/mavros/global_position/compass_hdg");
        this->declare_parameter("topics.drone_odom", "interface/mavros/local_position/odom");
        this->declare_parameter("topics.drone_imu", "interface/mavros/imu/data");

        // MAVLink parameters
        this->declare_parameter("mavlink.payload_system_id", 1);
        this->declare_parameter("mavlink.payload_component_id", 100);
        this->declare_parameter("mavlink.gimbal_system_id", 1);
        this->declare_parameter("mavlink.gimbal_component_id", 154);

        // Gimbal parameters
        this->declare_parameter("gimbal.input_mode", 1);
        this->declare_parameter("gimbal.control.max_speed", 30.0);
        this->declare_parameter("gimbal.control.acceleration", 10.0);
        this->declare_parameter("gimbal.control.smoothing", 3);

        // Angle limits
        this->declare_parameter("gimbal.limits.pan_min", -180.0);
        this->declare_parameter("gimbal.limits.pan_max", 180.0);
        this->declare_parameter("gimbal.limits.tilt_min", -90.0);
        this->declare_parameter("gimbal.limits.tilt_max", 90.0);
        this->declare_parameter("gimbal.limits.roll_min", -45.0);
        this->declare_parameter("gimbal.limits.roll_max", 45.0);

        // Heading follow parameters
        this->declare_parameter("heading_follow.enable", false);
        this->declare_parameter("heading_follow.offset", 0.0);
        this->declare_parameter("heading_follow.deadband", 2.0);
        this->declare_parameter("heading_follow.gain", 1.0);

        // Node parameters
        this->declare_parameter("node.publish_rate", 50.0);
        this->declare_parameter("node.timeout", 5.0);
    }

    void loadParameters()
    {
        // Load network parameters
        udp_ip_target_ = this->get_parameter("network.udp_ip_target").as_string();
        udp_port_target_ = this->get_parameter("network.udp_port_target").as_int();

        // Load UART parameters
        uart_port_ = this->get_parameter("uart.port").as_string();
        uart_baudrate_ = this->get_parameter("uart.baudrate").as_int();

        // Load connection method
        control_method_ = this->get_parameter("connection.control_method").as_int();

        // Load environment parameters
        robot_name_ = this->get_parameter("environment.robot_name").as_string();
        auto_namespace_ = this->get_parameter("environment.auto_namespace").as_bool();

        // Load topic names
        move_gimbal_angle_topic_ = this->get_parameter("topics.move_gimbal_angle").as_string();
        gimbal_status_topic_ = this->get_parameter("topics.gimbal_status").as_string();
        gimbal_attitude_topic_ = this->get_parameter("topics.gimbal_attitude").as_string();
        gimbal_command_topic_ = this->get_parameter("topics.gimbal_command").as_string();
        gimbal_home_topic_ = this->get_parameter("topics.gimbal_home").as_string();
        gimbal_reboot_topic_ = this->get_parameter("topics.gimbal_reboot").as_string();
        gimbal_imu_topic_ = this->get_parameter("topics.gimbal_imu").as_string();
        heading_follow_topic_ = this->get_parameter("topics.heading_follow").as_string();

        // Load drone topic base names
        drone_heading_topic_base_ = this->get_parameter("topics.drone_heading").as_string();
        drone_odom_topic_base_ = this->get_parameter("topics.drone_odom").as_string();
        drone_imu_topic_base_ = this->get_parameter("topics.drone_imu").as_string();

        // Construct full drone topic names with robot namespace
        if (auto_namespace_ && !robot_name_.empty()) {
            drone_heading_topic_full_ = "/" + robot_name_ + "/" + drone_heading_topic_base_;
            drone_odom_topic_full_ = "/" + robot_name_ + "/" + drone_odom_topic_base_;
            drone_imu_topic_full_ = "/" + robot_name_ + "/" + drone_imu_topic_base_;
        } else {
            drone_heading_topic_full_ = drone_heading_topic_base_;
            drone_odom_topic_full_ = drone_odom_topic_base_;
            drone_imu_topic_full_ = drone_imu_topic_base_;
        }

        // Load MAVLink parameters
        payload_system_id_ = this->get_parameter("mavlink.payload_system_id").as_int();
        payload_component_id_ = this->get_parameter("mavlink.payload_component_id").as_int();
        gimbal_system_id_ = this->get_parameter("mavlink.gimbal_system_id").as_int();
        gimbal_component_id_ = this->get_parameter("mavlink.gimbal_component_id").as_int();

        // Load gimbal parameters
        input_mode_ = this->get_parameter("gimbal.input_mode").as_int();
        max_speed_ = this->get_parameter("gimbal.control.max_speed").as_double();
        acceleration_ = this->get_parameter("gimbal.control.acceleration").as_double();
        smoothing_ = this->get_parameter("gimbal.control.smoothing").as_int();

        // Load angle limits
        pan_min_ = this->get_parameter("gimbal.limits.pan_min").as_double();
        pan_max_ = this->get_parameter("gimbal.limits.pan_max").as_double();
        tilt_min_ = this->get_parameter("gimbal.limits.tilt_min").as_double();
        tilt_max_ = this->get_parameter("gimbal.limits.tilt_max").as_double();
        roll_min_ = this->get_parameter("gimbal.limits.roll_min").as_double();
        roll_max_ = this->get_parameter("gimbal.limits.roll_max").as_double();

        // Load heading follow parameters
        enable_heading_follow_ = this->get_parameter("heading_follow.enable").as_bool();
        heading_offset_ = this->get_parameter("heading_follow.offset").as_double();
        heading_deadband_ = this->get_parameter("heading_follow.deadband").as_double();
        heading_gain_ = this->get_parameter("heading_follow.gain").as_double();

        // Load node parameters
        publish_rate_ = this->get_parameter("node.publish_rate").as_double();
        timeout_ = this->get_parameter("node.timeout").as_double();

        // Log loaded configuration
        RCLCPP_INFO(this->get_logger(), "Configuration loaded:");
        RCLCPP_INFO(this->get_logger(), "  Robot: %s (auto_namespace: %s)",
                   robot_name_.c_str(), auto_namespace_ ? "true" : "false");
        if (control_method_ == CONTROL_UDP) {
            RCLCPP_INFO(this->get_logger(), "  Connection: UDP %s:%d",
                       udp_ip_target_.c_str(), udp_port_target_);
        } else {
            RCLCPP_INFO(this->get_logger(), "  Connection: UART %s @ %d",
                       uart_port_.c_str(), uart_baudrate_);
        }
        RCLCPP_INFO(this->get_logger(), "  Gimbal topics: angle=%s, status=%s",
                   move_gimbal_angle_topic_.c_str(), gimbal_status_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Drone heading topic: %s",
                   drone_heading_topic_full_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Heading follow: %s (offset=%.1f°)",
                   enable_heading_follow_ ? "enabled" : "disabled", heading_offset_);
    }

    void initializeTopics()
    {
        // Publishers
        attitude_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            gimbal_attitude_topic_, 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            gimbal_status_topic_, 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            gimbal_imu_topic_, 10);
        heading_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "gimbal/heading_status", 10);

        // Gimbal control subscribers
        angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            move_gimbal_angle_topic_, 10,
            std::bind(&GremsyROS2Node::angleCallback, this, std::placeholders::_1));

        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            gimbal_command_topic_, 10,
            std::bind(&GremsyROS2Node::commandCallback, this, std::placeholders::_1));

        home_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            gimbal_home_topic_, 10,
            std::bind(&GremsyROS2Node::homeCallback, this, std::placeholders::_1));

        reboot_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            gimbal_reboot_topic_, 10,
            std::bind(&GremsyROS2Node::rebootCallback, this, std::placeholders::_1));

        // Drone data subscribers - Use compass heading as primary source
        drone_heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            drone_heading_topic_full_, 10,
            std::bind(&GremsyROS2Node::droneHeadingCallback, this, std::placeholders::_1));

        // Optional: Subscribe to odometry and IMU as backup heading sources
        drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            drone_odom_topic_full_, 10,
            std::bind(&GremsyROS2Node::droneOdomCallback, this, std::placeholders::_1));

        drone_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            drone_imu_topic_full_, 10,
            std::bind(&GremsyROS2Node::droneImuCallback, this, std::placeholders::_1));

        // Heading follow control
        heading_follow_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            heading_follow_topic_, 10,
            std::bind(&GremsyROS2Node::headingFollowCallback, this, std::placeholders::_1));
    }

    void initializeGimbal()
    {
        try {
            // Create port based on configuration
            if (control_method_ == CONTROL_UDP) {
                gimbal_port_ = new UDP_Port(
                    const_cast<char*>(udp_ip_target_.c_str()),
                    udp_port_target_
                );
                RCLCPP_INFO(this->get_logger(), "Creating UDP connection to %s:%d",
                           udp_ip_target_.c_str(), udp_port_target_);
            } else {
                gimbal_port_ = new Serial_Port(
                    const_cast<char*>(uart_port_.c_str()),
                    uart_baudrate_
                );
                RCLCPP_INFO(this->get_logger(), "Creating UART connection to %s @ %d",
                           uart_port_.c_str(), uart_baudrate_);
            }

            // Create payload interface
            payload_interface_ = new PayloadSdkInterface();
            // Note: The exact method to set the mavlink interface may vary
            // depending on the PayloadSdkInterface implementation

            // Start the port
            gimbal_port_->start();

            RCLCPP_INFO(this->get_logger(), "Gimbal connection initialized");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize gimbal: %s", e.what());
            gimbal_port_ = nullptr;
            payload_interface_ = nullptr;
        }
    }

    void gimbalLoop()
    {
        while (gimbal_running_ && rclcpp::ok()) {
            if (payload_interface_ && gimbal_port_) {
                try {
                    // Check connection status
                    auto now = std::chrono::steady_clock::now();
                    if (gimbal_port_->is_running()) {
                        last_heartbeat_ = now;
                        gimbal_connected_ = true;
                    } else {
                        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            now - last_heartbeat_).count();
                        if (elapsed > timeout_) {
                            gimbal_connected_ = false;
                        }
                    }

                    // Update current attitude from payload interface
                    // Note: This would need to be implemented based on the actual
                    // PayloadSdkInterface methods available

                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "Gimbal communication error: %s", e.what());
                    gimbal_connected_ = false;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void publishStatus()
    {
        // Publish attitude
        if (gimbal_connected_) {
            geometry_msgs::msg::Vector3 attitude_msg;
            attitude_msg.x = current_attitude_.roll;
            attitude_msg.y = current_attitude_.pitch;
            attitude_msg.z = current_attitude_.yaw;
            attitude_pub_->publish(attitude_msg);
        }

        // Publish status
        std_msgs::msg::String status_msg;
        status_msg.data = gimbal_connected_ ? "connected" : "disconnected";
        status_pub_->publish(status_msg);

        // Publish heading follow status
        std_msgs::msg::String heading_status_msg;
        if (heading_follow_enabled_) {
            heading_status_msg.data = drone_heading_valid_ ?
                "following_heading" : "waiting_for_heading";
        } else {
            heading_status_msg.data = "heading_follow_disabled";
        }
        heading_status_pub_->publish(heading_status_msg);
    }

    void updateHeadingFollow()
    {
        if (!heading_follow_enabled_ || !drone_heading_valid_ || !gimbal_connected_) {
            return;
        }

        // Check if heading data is recent (within 1 second)
        auto now = std::chrono::steady_clock::now();
        auto heading_age = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_heading_update_).count();

        if (heading_age > 1000) {
            drone_heading_valid_ = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Drone heading data stale, disabling heading follow");
            return;
        }

        // Calculate desired gimbal yaw based on drone heading
        double target_yaw = drone_heading_ + heading_offset_;

        // Normalize to [-180, 180]
        while (target_yaw > 180.0) target_yaw -= 360.0;
        while (target_yaw < -180.0) target_yaw += 360.0;

        // Calculate error with current gimbal yaw
        double yaw_error = target_yaw - current_attitude_.yaw;

        // Normalize error to [-180, 180]
        while (yaw_error > 180.0) yaw_error -= 360.0;
        while (yaw_error < -180.0) yaw_error += 360.0;

        // Apply deadband
        if (std::abs(yaw_error) < heading_deadband_) {
            return;
        }

        // Apply gain and clamp to limits
        double commanded_yaw = current_attitude_.yaw + (yaw_error * heading_gain_);
        commanded_yaw = std::clamp(commanded_yaw, pan_min_, pan_max_);

        // Send command to gimbal (keep current roll and pitch)
        try {
            payload_interface_->setGimbalSpeed(
                current_attitude_.roll,
                current_attitude_.pitch,
                commanded_yaw,
                INPUT_ANGLE
            );

            RCLCPP_DEBUG(this->get_logger(),
                        "Heading follow: drone=%.1f° target=%.1f° error=%.1f° cmd=%.1f°",
                        drone_heading_.load(), target_yaw, yaw_error, commanded_yaw);

        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Failed to send heading follow command: %s", e.what());
        }
    }

    // Utility function to extract yaw from quaternion
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quat, tf_quat);

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

        return yaw * 180.0 / M_PI;  // Convert to degrees
    }

    // ROS2 callback functions
    void angleCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        // Disable heading follow when manual commands are received
        if (heading_follow_enabled_) {
            RCLCPP_INFO(this->get_logger(), "Manual gimbal command received, disabling heading follow");
            heading_follow_enabled_ = false;
        }

        if (!gimbal_connected_ || !payload_interface_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Gimbal not connected, ignoring angle command");
            return;
        }

        // Clamp angles to limits
        float roll = std::clamp(static_cast<float>(msg->x),
                               static_cast<float>(roll_min_),
                               static_cast<float>(roll_max_));
        float pitch = std::clamp(static_cast<float>(msg->y),
                                static_cast<float>(tilt_min_),
                                static_cast<float>(tilt_max_));
        float yaw = std::clamp(static_cast<float>(msg->z),
                              static_cast<float>(pan_min_),
                              static_cast<float>(pan_max_));

        try {
            payload_interface_->setGimbalSpeed(roll, pitch, yaw, INPUT_ANGLE);

            RCLCPP_DEBUG(this->get_logger(), "Manual gimbal command: R=%.1f P=%.1f Y=%.1f",
                        roll, pitch, yaw);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send gimbal command: %s", e.what());
        }
    }

    void commandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!gimbal_connected_ || !payload_interface_) {
            RCLCPP_WARN(this->get_logger(), "Gimbal not connected, ignoring command");
            return;
        }

        std::string command = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received gimbal command: %s", command.c_str());

        // Note: The PayloadSdkInterface may not have direct gimbal control methods
        // You may need to implement these using the lower-level mavlink commands
        if (command == "enable_heading_follow") {
            heading_follow_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "Heading follow enabled");
        } else if (command == "disable_heading_follow") {
            heading_follow_enabled_ = false;
            RCLCPP_INFO(this->get_logger(), "Heading follow disabled");
        } else {
            RCLCPP_WARN(this->get_logger(), "Command '%s' not implemented for PayloadSDK interface", command.c_str());
        }
    }

    void homeCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && gimbal_connected_ && payload_interface_) {
            // Note: Home functionality would need to be implemented
            // using the PayloadSdkInterface methods
            RCLCPP_INFO(this->get_logger(), "Gimbal home command received (not implemented)");
        }
    }

    void rebootCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && gimbal_connected_ && payload_interface_) {
            // Note: Reboot functionality would need to be implemented
            // using the PayloadSdkInterface methods
            RCLCPP_INFO(this->get_logger(), "Gimbal reboot command received (not implemented)");
        }
    }

    // Drone data callbacks
    void droneHeadingCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Direct compass heading in degrees
        drone_heading_ = msg->data;
        drone_heading_valid_ = true;
        last_heading_update_ = std::chrono::steady_clock::now();

        RCLCPP_DEBUG(this->get_logger(), "Received drone compass heading: %.1f°", msg->data);
    }

    void droneOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract heading from odometry quaternion
        double heading = quaternionToYaw(msg->pose.pose.orientation);
        drone_heading_ = heading;
        drone_heading_valid_ = true;
        last_heading_update_ = std::chrono::steady_clock::now();

        RCLCPP_DEBUG(this->get_logger(), "Received drone odometry heading: %.1f°", heading);
    }

    void droneImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract heading from IMU quaternion
        double heading = quaternionToYaw(msg->orientation);
        drone_heading_ = heading;
        drone_heading_valid_ = true;
        last_heading_update_ = std::chrono::steady_clock::now();

        RCLCPP_DEBUG(this->get_logger(), "Received drone IMU heading: %.1f°", heading);
    }

    void headingFollowCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        heading_follow_enabled_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Heading follow %s",
                   msg->data ? "enabled" : "disabled");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<GremsyROS2Node>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("gremsy_main"), "Node failed: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}