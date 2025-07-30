#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "payloadSdkInterface.h"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;


T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};
PayloadSdkInterface* my_payload = nullptr;

class MyPublisher : public rclcpp::Node
{
public:
  MyPublisher() : Node("my_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MyPublisher::timer_callback, this));

    subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "move_gimbal_angle", 10, std::bind(&MyPublisher::move_gimbal_angle_mode_callback, this, std::placeholders::_1));

    my_payload = new PayloadSdkInterface(s_conn);
	my_payload->sdkInitConnection();
    my_payload->checkPayloadConnection();
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, 
      PAYLOAD_CAMERA_RC_MODE_STANDARD, PARAM_TYPE_UINT32);
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world!";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
  void move_gimbal_angle_mode_callback(const geometry_msgs::msg::Vector3 & msg) const
  {
    RCLCPP_INFO(this->get_logger(),"x: %f, y: %f, z: %f", msg.x, msg.y, msg.z);
    my_payload->setGimbalSpeed(msg.x, msg.y, msg.z, INPUT_ANGLE);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisher>());
  rclcpp::shutdown();
  return 0;
}