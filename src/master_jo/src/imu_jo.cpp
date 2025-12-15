#include "../include/master_jo/imu_jo.hpp" 
#include <cmath>

ImuJo::ImuJo() : Node("imu_jo")
{
    auto qos_profile = rclcpp::SensorDataQoS();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        qos_profile,
        std::bind(&ImuJo::topic_callback, this, std::placeholders::_1));

    angle_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu_angle", qos_profile);

    RCLCPP_INFO(this->get_logger(), "ImuJo start");
}

void ImuJo::topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    double x = msg->orientation.x;
    double y = msg->orientation.y;
    double z = msg->orientation.z;
    double w = msg->orientation.w;

    double roll, pitch, yaw_rad;

    double t0 = 2.0 * (w * x + y * z);
    double t1 = 1.0 - 2.0 * (x * x + y * y);
    roll = atan2(t0, t1);

    double t2 = 2.0 * (w * y - z * x);
    if (t2 > 1.0) t2 = 1.0;
    if (t2 < -1.0) t2 = -1.0;
    pitch = asin(t2);

    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (y * y + z * z);
    yaw_rad = atan2(t3, t4);

    double current_yaw = -1.0 * yaw_rad; 

    if (first_run_) {
        yaw_offset_ = current_yaw;
        first_run_ = false;
        RCLCPP_INFO(this->get_logger(), "Offset: %.2f", yaw_offset_);
    }

    double corrected_yaw = atan2(sin(current_yaw - yaw_offset_), cos(current_yaw - yaw_offset_));
    
    auto angle_msg = geometry_msgs::msg::Vector3();
    angle_msg.x = roll*60;
    angle_msg.y = pitch*60;
    angle_msg.z = corrected_yaw*60;

    angle_pub_->publish(angle_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuJo>());
    rclcpp::shutdown();
    return 0;
}