#include "../include/master_jo/psd_jo.hpp"

PsdJo::PsdJo() : Node("psd_jo")
{
    psd_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "robit_psd", 10, std::bind(&PsdJo::topic_callback, this, std::placeholders::_1));

    pub_front_ = this->create_publisher<std_msgs::msg::Int32>("psd_result_front", 10);
    pub_left_ = this->create_publisher<std_msgs::msg::Int32>("psd_result_left", 10);
    pub_right_ = this->create_publisher<std_msgs::msg::Int32>("psd_result_right", 10);

    RCLCPP_INFO(this->get_logger(), "PsdJo publish start");
}

void PsdJo::topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
{
    uint16_t front_adc = msg->data[0];
    uint16_t left_adc = msg->data[1];
    uint16_t right_adc = msg->data[2];

    auto msg_f = std_msgs::msg::Int32();
    auto msg_l = std_msgs::msg::Int32();
    auto msg_r = std_msgs::msg::Int32();

    if (front_adc > detect_object)
    {
        flag_f++;
        if (flag_f > 5)
        {
            msg_f.data = 1;
        }
    }
    else
    {
        flag_f = 0;
    }
    if (left_adc > detect_object)
    {
        flag_l++;
        if (flag_l > 5)
        {
            msg_l.data = 1;
        }
    }
    else
    {
        flag_l = 0;
    }
    if (right_adc > detect_object)
    {
        flag_r++;
        if (flag_r > 5)
        {
            msg_r.data = 1;
        }
    }
    else
    {
        flag_r = 0;
    }

    pub_front_->publish(msg_f);
    pub_left_->publish(msg_l);
    pub_right_->publish(msg_r);

    RCLCPP_INFO(this->get_logger(), "F:%d L:%d R:%d F_ADC: %d L_ADC: %d R_ADC: %d", msg_f.data, msg_l.data, msg_r.data, front_adc, left_adc, right_adc);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PsdJo>());
    rclcpp::shutdown();
    return 0;
}