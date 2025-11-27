#include "../include/master_jo/imu_jo.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ImuJo::ImuJo() : Node("imu_jo")
{
    auto qos_profile = rclcpp::SensorDataQoS();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        qos_profile,
        std::bind(&ImuJo::topic_callback, this, std::placeholders::_1));

    angle_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu_angle", 10);

    RCLCPP_INFO(this->get_logger(), "ImuJo start");
}

void ImuJo::topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "orientation.w: '%f'", msg->orientation.w);
    RCLCPP_INFO(this->get_logger(), "orientation.x: '%f'", msg->orientation.x);
    RCLCPP_INFO(this->get_logger(), "orientation.y: '%f'", msg->orientation.y);
    RCLCPP_INFO(this->get_logger(), "orientation.z: '%f'", msg->orientation.z);
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    auto angle_msg = geometry_msgs::msg::Vector3();
    angle_msg.x = roll;
    angle_msg.y = pitch;
    angle_msg.z = yaw;

    angle_pub_->publish(angle_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuJo>());
    rclcpp::shutdown();
    return 0;
}