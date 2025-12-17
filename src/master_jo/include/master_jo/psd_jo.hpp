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

    int front_adc[5] = {0};
    int left_adc[5]  = {0};
    int right_adc[5] = {0};
    double front_lpf=0;
    double right_lpf=0;
    double left_lpf=0;
    double alpha=0.2;

    int idx = 0;
};

#endif
