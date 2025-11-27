#ifndef IMU_JO_HPP_
#define IMU_JO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class ImuJo : public rclcpp::Node
{
public:
    ImuJo();

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angle_pub_;
};

#endif