#include "psd_jo.hpp"

using std::placeholders::_1;

PsdJo::PsdJo() : Node("psd_jo")
{
  subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
    "robit_psd",
    10,
    std::bind(&PsdJo::topic_callback, this, _1));
    
  RCLCPP_INFO(this->get_logger(), "psd subscribe start");
}

void PsdJo::topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) const
{
  uint16_t left = msg->data[0];
  uint16_t front = msg->data[1];
  uint16_t right = msg->data[2];

  RCLCPP_INFO(this->get_logger(), 
    "PSD 값 | 정면: %d | 왼쪽: %d | 오른쪽: %d", // 가까우면 값 커짐
    left, front, right);

  /*
  if (front < 200) {
      RCLCPP_WARN(this->get_logger(), "꺅 장애물이다!");
  }
  */
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PsdJo>());
  rclcpp::shutdown();
  return 0;
}