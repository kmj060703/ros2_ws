#include "master_jo/master_jo.hpp"
#include <string>
#include <vector>
#include <sstream>

MasterJo::MasterJo()
    : Node("master_jo")
{
  auto qos_profile = rclcpp::SensorDataQoS();
  yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
      "MasterJo_YOLO",
      qos_profile,
      std::bind(&MasterJo::yolo_callback, this, std::placeholders::_1));

  vision_sub_ = this->create_subscription<autorace_interfaces::msg::VisionHyun>(
      "/vision/line_diff_info",
      qos_profile,
      std::bind(&MasterJo::vision_callback, this, std::placeholders::_1));

  flag_pub_ = this->create_publisher<std_msgs::msg::Int32>("master_jo_flag", qos_profile);
  pixel_diff_pub_ = this->create_publisher<autorace_interfaces::msg::MasterJo>("pixel_diff", qos_profile);

  RCLCPP_INFO(this->get_logger(), "MasterJo_YOLO & MasterJo_VISION Start");
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

    if (confidence >= 0.85)
    {
      int detected_flag = 0;

      if (object_name == "T")
      {
        flag_T++;
        if (flag_T > 6)
        {
          detected_flag = 1;
        }
      }
      else
      {
        flag_T = 0;
      }

      if (object_name == "L")
      {
        flag_L++;
        if (flag_L > 6)
        {
          detected_flag = 2;
        }
      }
      else
      {
        flag_L = 0;
      }

      if (object_name == "R")
      {
        flag_R++;
        if (flag_R > 6)
        {
          detected_flag = 3;
        }
      }
      else
      {
        flag_R = 0;
      }

      if (object_name == "D")
      {
        flag_D++;
        if (flag_D > 6)
        {
          detected_flag = 4;
        }
      }
      else
      {
        flag_D = 0;
      }

      if (object_name == "P")
      {
        flag_P++;
        if (flag_P > 6)
        {
          detected_flag = 5;
        }
      }
      else
      {
        flag_P = 0;
      }

      if (detected_flag != 0)
      {
        if (current_mission_flag_ != detected_flag)
        {
          current_mission_flag_ = detected_flag;
          RCLCPP_WARN(this->get_logger(), "구간 변경: %s || 플래그: %d", object_name.c_str(), current_mission_flag_);
        }

        auto flag_msg = std_msgs::msg::Int32();
        flag_msg.data = current_mission_flag_;
        flag_pub_->publish(flag_msg);
      }
    }
  }
}

void MasterJo::vision_callback(const autorace_interfaces::msg::VisionHyun::SharedPtr msg)
{
  int center_x = msg->center_x;
  int yellow_x = msg->yellow_x;
  int white_x = msg->white_x;

  double pixel_diff = center_x - 320;
  double yellow_diff = yellow_x - 320;
  double white_diff = white_x - 320;

  auto diff_msg = autorace_interfaces::msg::MasterJo();

  diff_msg.pixel_diff = pixel_diff;
  diff_msg.yellow_diff = yellow_diff;
  diff_msg.white_diff = white_diff;

  pixel_diff_pub_->publish(diff_msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterJo>());
  rclcpp::shutdown();
  return 0;
}