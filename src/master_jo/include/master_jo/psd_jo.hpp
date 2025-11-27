#ifndef PSD_JO_HPP_
#define PSD_JO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

class PsdJo : public rclcpp::Node
{
public:
    PsdJo();

private:
    void topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr psd_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_front_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_left_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_right_;

    const uint16_t detect_object_f = 800;
    const uint16_t detect_object_l = 520; 
    const uint16_t detect_object_r = 560; 
    int flag_f = 0;
    int flag_l = 0;
    int flag_r = 0;
};

#endif