#ifndef DRIVING_YY_HPP_
#define DRIVING_YY_HPP_

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "autorace_interfaces/msg/ui2_driving.hpp"
#include "autorace_interfaces/msg/master_jo.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>

using namespace std::chrono_literals;
using namespace std;

class DrivingYY : public rclcpp::Node
{
public:
    DrivingYY();
    void PD_control();
    int start_flag = 0;
    double kp;
    double kd;
    double x;
    double z;
    double error;
    double last_error;
    double max_x;

private:
    void imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void psd_front_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void psd_left_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void psd_right_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void flag_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void ui_callback(const autorace_interfaces::msg::Ui2Driving::SharedPtr msg);
    void drive_callback();
    void pixel_diff_callback(const autorace_interfaces::msg::MasterJo::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr psd_front_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr psd_left_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr psd_right_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flag_sub_;
    rclcpp::Subscription<autorace_interfaces::msg::Ui2Driving>::SharedPtr ui_sub_;
    rclcpp::Subscription<autorace_interfaces::msg::MasterJo>::SharedPtr pixel_diff_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_drive;

    double current_yaw_ = 0.0;
    int is_front_danger_ = 0;
    int is_left_danger_ = 0;
    int is_right_danger_ = 0;
    int mission_flag_ = 0;
    int current_pixel_diff_ = 0;
    rclcpp::TimerBase::SharedPtr drive_timer;
};

#endif