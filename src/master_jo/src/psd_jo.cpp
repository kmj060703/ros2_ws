#include "../include/master_jo/psd_jo.hpp"

PsdJo::PsdJo() : Node("psd_jo")
{
    auto qos_profile = rclcpp::SensorDataQoS();
    psd_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "robit_psd", qos_profile, std::bind(&PsdJo::topic_callback, this, std::placeholders::_1));

    pub_front_ = this->create_publisher<std_msgs::msg::Int32>("psd_result_front", qos_profile);
    pub_left_ = this->create_publisher<std_msgs::msg::Int32>("psd_result_left", qos_profile);
    pub_right_ = this->create_publisher<std_msgs::msg::Int32>("psd_result_right", qos_profile);

    RCLCPP_INFO(this->get_logger(), "PsdJo publish start");
}

void PsdJo::topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
{
    uint16_t front_adc = msg->data[0];
    uint16_t left_adc = msg->data[1];
    uint16_t right_adc = msg->data[2];

    std_msgs::msg::Int32 msg_f;
    std_msgs::msg::Int32 msg_l;
    std_msgs::msg::Int32 msg_r;

    msg_f.data = front_adc;
    msg_l.data = left_adc;
    msg_r.data = right_adc;

    pub_front_->publish(msg_f);
    pub_left_->publish(msg_l);
    pub_right_->publish(msg_r);

    // RCLCPP_INFO(this->get_logger(),
    //             "F_ADC: %d  L_ADC: %d  R_ADC: %d",
    //             front_adc, left_adc, right_adc);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PsdJo>());
    rclcpp::shutdown();
    return 0;
}
