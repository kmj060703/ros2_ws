
#include <chrono>
#include <functional>
#include <memory>
#include <string>


//#include <geometry_msgs/msg/Twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <iostream>
#include<array>
using namespace std;


#include "rclcpp/rclcpp.hpp"

class DrivingYYNode : public rclcpp::Node
{
    public:
       DrivingYYNode();
    // double turn(double angle, double angular_velocity, double step);
    // double go_straight(double distance, double linear_velocity, double step);
    private:
       void topic_callback(const sensor_msgs::msg::Imu msg);
       rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr Imu2Driving;
};

// Imu2Driving =
//       std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(
//       this,
//       "imu");

// std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> msg_ftr_imu_sub_;