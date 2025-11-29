#ifndef MASTER_JO_HPP_
#define MASTER_JO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "autorace_interfaces/msg/vision_hyun.hpp"
#include "autorace_interfaces/msg/master_jo.hpp"

class MasterJo : public rclcpp::Node
{
public:
  MasterJo();

private:
  void yolo_callback(const std_msgs::msg::String::SharedPtr msg);
  void vision_callback(const autorace_interfaces::msg::VisionHyun::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr flag_pub_;
  rclcpp::Subscription<autorace_interfaces::msg::VisionHyun>::SharedPtr vision_sub_;
  rclcpp::Publisher<autorace_interfaces::msg::MasterJo>::SharedPtr pixel_diff_pub_;

  int current_mission_flag_ = 0;
  int flag_T = 0;
  int flag_L = 0;
  int flag_R = 0;
  int flag_D = 0;
  int flag_P = 0;
};

#endif