#ifndef GREMSY_BASE_NODE_HPP
#define GREMSY_BASE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "payloadSdkInterface.h"

class GremsyBaseNode : public rclcpp::Node
{
public:
    GremsyBaseNode(const std::string& node_name) : Node(node_name)
    {
        declare_common_parameters();
        load_common_parameters();
        initialize_payload_connection();
    }

    virtual ~GremsyBaseNode() {
        if (payload_interface_) {
            delete payload_interface_;
        }
    }

protected:
    // Common parameters
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

    // Payload interface
    PayloadSdkInterface* payload_interface_ = nullptr;

    // Common helper methods
    bool is_connected() const {
        return payload_interface_ != nullptr;
    }

    void log_connection_info() {
        if (control_method_ == CONTROL_UDP) {
            RCLCPP_INFO(this->get_logger(), "Connected via UDP to %s:%d", 
                       udp_ip_target_.c_str(), udp_port_target_);
        } else {
            RCLCPP_INFO(this->get_logger(), "Connected via UART to %s at %d baud", 
                       uart_port_.c_str(), uart_baudrate_);
        }
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
        // Setup connection info
        T_ConnInfo conn_info = {
            static_cast<uint8_t>(control_method_),
            {}
        };
        
        if (control_method_ == CONTROL_UDP) {
            conn_info.device.udp.ip = const_cast<char*>(udp_ip_target_.c_str());
            conn_info.device.udp.port = udp_port_target_;
        } else {
            conn_info.device.uart.name = const_cast<char*>(uart_port_.c_str());
            conn_info.device.uart.baudrate = uart_baudrate_;
        }

        // Initialize payload SDK
        try {
            payload_interface_ = new PayloadSdkInterface(conn_info);
            payload_interface_->sdkInitConnection();
            payload_interface_->checkPayloadConnection();
            
            log_connection_info();
            RCLCPP_INFO(this->get_logger(), "Payload SDK initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize payload SDK: %s", e.what());
            if (payload_interface_) {
                delete payload_interface_;
                payload_interface_ = nullptr;
            }
        }
    }
};

#endif // GREMSY_BASE_NODE_HPP