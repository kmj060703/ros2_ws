#include "master_jo.hpp"

MasterJo::MasterJo()
: Node("master_jo")
{
  RCLCPP_INFO(this->get_logger(), "MasterJo Node start");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MasterJo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
