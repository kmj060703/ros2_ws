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
    if (msg->data.size() < 3)
        return;

    front_adc[idx] = msg->data[0];
    left_adc[idx]  = msg->data[1];
    right_adc[idx] = msg->data[2];

    idx++;
    if (idx >= 5) idx = 0;

    int front_sum = 0;
    int left_sum  = 0;
    int right_sum = 0;

    for (int i = 0; i < 5; i++)
    {
        front_sum += front_adc[i];
        left_sum  += left_adc[i];
        right_sum += right_adc[i];
    }

    std_msgs::msg::Int32 msg_f;
    std_msgs::msg::Int32 msg_l;
    std_msgs::msg::Int32 msg_r;

    msg_f.data = front_sum / 5;
    msg_l.data = left_sum  / 5;
    msg_r.data = right_sum / 5;

    pub_front_->publish(msg_f);
    pub_left_->publish(msg_l);
    pub_right_->publish(msg_r);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PsdJo>());
    rclcpp::shutdown();
    return 0;
}
