#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>

class ImuJo : public rclcpp::Node
{
    public:
       ImuJo();
    private:
       void topic_callback(const sensor_msgs::msg::Imu msg);
       rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr Imu2Driving;
};