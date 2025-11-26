#include "psd_jo.hpp"

PsdJo::PsdJo()
: Node("psd_jo")
{
  RCLCPP_INFO(this->get_logger(), "PsdJo Node start");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PsdJo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
