#include "imu_jo.hpp"

ImuJo::ImuJo() : Node("imu_jo")
{
    Imu2Driving = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        10,
        std::bind(&ImuJo::topic_callback, this, std::placeholders::_1));
}

void ImuJo::topic_callback(const sensor_msgs::msg::Imu msg)
{
    RCLCPP_INFO(this->get_logger(), "orientation.w: '%f'", msg.orientation.w);
    RCLCPP_INFO(this->get_logger(), "orientation.x: '%f'", msg.orientation.x);
    RCLCPP_INFO(this->get_logger(), "orientation.y: '%f'", msg.orientation.y);
    RCLCPP_INFO(this->get_logger(), "orientation.z: '%f'", msg.orientation.z);
    RCLCPP_INFO(this->get_logger(), "angular_velocity.x: '%f'", msg.angular_velocity.x);
    RCLCPP_INFO(this->get_logger(), "angular_velocity.y: '%f'", msg.angular_velocity.y);
    RCLCPP_INFO(this->get_logger(), "angular_velocity.z: '%f'", msg.angular_velocity.z);
    RCLCPP_INFO(this->get_logger(), "linear_acceleration.x: '%f'", msg.linear_acceleration.x);
    RCLCPP_INFO(this->get_logger(), "linear_acceleration.y: '%f'", msg.linear_acceleration.y);
    RCLCPP_INFO(this->get_logger(), "linear_acceleration.z: '%f'", msg.linear_acceleration.z);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuJo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}