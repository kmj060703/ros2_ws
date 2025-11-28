#ifndef MASTER_JO_HPP_
#define MASTER_JO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MasterJo : public rclcpp::Node
{
public:
  MasterJo();

private:
  void yolo_callback(const std_msgs::msg::String::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
};

#endif