#ifndef DRIVING_YY_HPP_
#define DRIVING_YY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/int32.hpp"

class DrivingYY : public rclcpp::Node
{
public:
    DrivingYY();

private:
    void imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void psd_front_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void psd_left_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void psd_right_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void flag_callback(const std_msgs::msg::Int32::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr psd_front_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr psd_left_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr psd_right_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flag_sub_;

    double current_yaw_ = 0.0;
    int is_front_danger_ = 0;
    int is_left_danger_ = 0;
    int is_right_danger_ = 0;
    int mission_flag_ = 0;
};

#endif