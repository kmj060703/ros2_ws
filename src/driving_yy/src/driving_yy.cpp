#include "../include/driving_yy/driving_yy.hpp"

using std::placeholders::_1;

DrivingYY::DrivingYY() : Node("driving_yy")
{
    imu_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "imu_angle",
        10,
        std::bind(&DrivingYY::imu_callback, this, _1));

    psd_front_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "psd_result_front", 10, std::bind(&DrivingYY::psd_front_callback, this, _1));

    psd_left_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "psd_result_left", 10, std::bind(&DrivingYY::psd_left_callback, this, _1));

    psd_right_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "psd_result_right", 10, std::bind(&DrivingYY::psd_right_callback, this, _1));

    flag_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "master_jo_flag",
        10,
        std::bind(&DrivingYY::flag_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "DrivingYY Start");
    RCLCPP_INFO(this->get_logger(), "초기 미션 플래그: %d", mission_flag_);
}

void DrivingYY::imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    current_yaw_ = msg->z;
    RCLCPP_INFO(this->get_logger(), "Yaw: %.2f 도", current_yaw_ * 180.0 / M_PI);
}

void DrivingYY::psd_front_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    is_front_danger_ = msg->data;
    //RCLCPP_WARN(this->get_logger(), "정면 장애물 감지 여부: %d", is_front_danger_);
}

void DrivingYY::psd_left_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    is_left_danger_ = msg->data;
    //RCLCPP_INFO(this->get_logger(), "왼쪽 장애물 감지 여부: %d", is_left_danger_);
}

void DrivingYY::psd_right_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    is_right_danger_ = msg->data;
    //RCLCPP_INFO(this->get_logger(), "오른쪽 장애물 감지 여부: %d", is_right_danger_);
}

void DrivingYY::flag_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    mission_flag_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "플래그: %d", mission_flag_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrivingYY>());
    rclcpp::shutdown();
    return 0;
}
