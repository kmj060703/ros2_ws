#ifndef PSD_JO_HPP_
#define PSD_JO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

class PsdJo : public rclcpp::Node
{
public:
  PsdJo();

private:
  void topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) const;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subscription_;
};

#endif
