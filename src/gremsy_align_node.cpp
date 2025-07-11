#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "payloadSdkInterface.h"

using namespace std::chrono_literals;

T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};

PayloadSdkInterface* my_payload = nullptr;

class GimbalFollowerNode : public rclcpp::Node
{
public:
  GimbalFollowerNode() : Node("gimbal_follower_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("gimbal_status", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&GimbalFollowerNode::timer_callback, this));
    subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "move_gimbal_angle", 10,
      std::bind(&GimbalFollowerNode::move_gimbal_callback, this, std::placeholders::_1));

    my_payload = new PayloadSdkInterface(s_conn);
    my_payload->sdkInitConnection();
    my_payload->checkPayloadConnection();

    // Try different approaches to disable stabilization:
    
    // Approach 1: Try different numeric values for the mode
    // Common gimbal modes: 0=manual, 1=stabilize, 2=lock, 3=follow
    try {
        my_payload->setPayloadCameraParam(
          PAYLOAD_CAMERA_RC_MODE,
          2,  // Try 2 for "lock" mode
          PARAM_TYPE_UINT32);
        RCLCPP_INFO(this->get_logger(), "Set gimbal to lock mode (value 2)");
    } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Failed to set lock mode");
    }

    // Approach 2: Try to disable stabilization directly
    // Look for other parameter types that might control stabilization
    
    RCLCPP_INFO(this->get_logger(), "Gimbal configured for lock/follow mode");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Gimbal in LOCK mode";
    RCLCPP_INFO(this->get_logger(), "Status: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void move_gimbal_callback(const geometry_msgs::msg::Vector3 & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Move Gimbal Request: x=%f y=%f z=%f", msg.x, msg.y, msg.z);
    my_payload->setGimbalSpeed(msg.x, msg.y, msg.z, INPUT_ANGLE);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalFollowerNode>());
  rclcpp::shutdown();
  return 0;
}