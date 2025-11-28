#include "../include/master_jo/master_jo.hpp"
#include <string>
#include <vector>
#include <sstream>

MasterJo::MasterJo()
    : Node("master_jo")
{
  yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
      "MasterJo_YOLO",
      10,
      std::bind(&MasterJo::yolo_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "MasterJo_YOLO start");
}

void MasterJo::yolo_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string data = msg->data;
  std::string object_name;
  std::string conf_str;
  std::stringstream ss(data);

  if (std::getline(ss, object_name, ',') && std::getline(ss, conf_str))
  {
    float confidence = std::stof(conf_str);

    if (confidence >= 0.8)
    {
      RCLCPP_INFO(this->get_logger(), "표지판 감지 %s || 신뢰도: %.2f", object_name.c_str(), confidence);

      if (object_name == "T")
      {
        RCLCPP_WARN(this->get_logger(), "T자 구간 진입");
      }
      else if (object_name == "L")
      {
        RCLCPP_WARN(this->get_logger(), "좌회전");
      }
      else if (object_name == "R")
      {
        RCLCPP_WARN(this->get_logger(), "우회전");
      }
      else if (object_name == "D")
      {
        RCLCPP_WARN(this->get_logger(), "장애물 구간 진입");
      }
      else if (object_name == "P")
      {
        RCLCPP_WARN(this->get_logger(), "주차 구간 진입");
      }
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MasterJo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}