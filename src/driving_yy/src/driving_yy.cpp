#include "../include/driving_yy/driving_yy.hpp"

using std::placeholders::_1;

DrivingYY::DrivingYY() : Node("driving_yy")
{
    kp = 0.0;
    kd = 0.0;
    x = 0.0;
    z = 0.0;
    error = 0.0;
    last_error = 0.0;
    max_x = 0.0;
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
    ui_sub_ = this->create_subscription<autorace_interfaces::msg::Ui2Driving>(
        "/ui2driving_topic",
        10,
        std::bind(&DrivingYY::ui_callback, this, _1));

    pixel_diff_sub_ = this->create_subscription<autorace_interfaces::msg::MasterJo>(
        "pixel_diff",
        10,
        std::bind(&DrivingYY::pixel_diff_callback, this, std::placeholders::_1));

    vision_traffic_sub_ = this->create_subscription<autorace_interfaces::msg::VisionHyun>(
        "/vision/line_diff_info",
        10,
        std::bind(&DrivingYY::vision_traffic_callback, this, _1));

    publisher_drive = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 30);
    drive_timer = this->create_wall_timer(100ms, std::bind(&DrivingYY::drive_callback, this));

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
    // RCLCPP_WARN(this->get_logger(), "정면 장애물 감지 여부: %d", is_front_danger_);
}

void DrivingYY::psd_left_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    is_left_danger_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "왼쪽 장애물 감지 여부: %d", is_left_danger_);
}

void DrivingYY::psd_right_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    is_right_danger_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "오른쪽 장애물 감지 여부: %d", is_right_danger_);
}

void DrivingYY::flag_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    mission_flag_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "플래그: %d", mission_flag_);
}

void DrivingYY::ui_callback(const autorace_interfaces::msg::Ui2Driving::SharedPtr msg)
{
    // 유아이
    start_flag = msg->l_start_flag;
    kp = msg->kp;
    kd = msg->kd;
    x = msg->l_x;
    z = msg->l_z;
}

void DrivingYY::pixel_diff_callback(const autorace_interfaces::msg::MasterJo::SharedPtr msg)
{
    current_pixel_diff_ = msg->pixel_diff;

    RCLCPP_INFO(this->get_logger(), "Pixel Diff: %d", current_pixel_diff_);
}

void DrivingYY::vision_traffic_callback(const autorace_interfaces::msg::VisionHyun::SharedPtr msg)
{
    traffic_light_status_ = msg->traffic_light;

    if (traffic_light_status_ == 1)
    {
        RCLCPP_INFO(this->get_logger(), "빨간불 감지 %d", traffic_light_status_);
    }
    else if (traffic_light_status_ == 2)
    {
        RCLCPP_INFO(this->get_logger(), "초록불 감지 %d", traffic_light_status_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "감지 안됨 혹은 노란색 %d", traffic_light_status_);
    }
}

void DrivingYY::PD_control()
{
    z = kp * error + kd * (error - last_error);
    last_error = error;
    auto msg = geometry_msgs::msg::Twist();
    x = min(pow(max_x * (1 - abs(error) / 500, 0), 2.2), 0.05); // x는 0.05보다 크면 안된다는 뜻?  //500은 중앙 픽셀 값 같기도하고 잘 모르겠다
    if (z < 0)
    {
        z = -max(z, -2.0);
    }
    else
    {
        z = -min(z, 2.0); // 2.0보다 절댓값이 크면 안된다는 뜻인가? 그리고 계산할때 부호반전된다
    }

    if (start_flag == 1)
    {
        msg.linear.x = x;
        msg.angular.z = z;
    }
    else
    {
        msg.linear.x = 0;
        msg.angular.z = 0;
    }
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    publisher_drive->publish(msg);
}

void DrivingYY::drive_callback()
{
    PD_control();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrivingYY>());
    rclcpp::shutdown();
    return 0;
}
