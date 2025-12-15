/**
 * @file /include/ui_test/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ui_test_QNODE_HPP_
#define ui_test_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "autorace_interfaces/msg/ui2_driving.hpp"
#include "autorace_interfaces/msg/vision_hyun.hpp"
#include "controlitem.hpp"
#endif

#include <thread>
#include <QThread>
#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <QLabel>

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();
  void drive_callback();
  void ui2drive_callback();
  void vision_helper(cv::Mat tmp, int img_id);
  void run() override;
  std::shared_ptr<rclcpp::Node> getNode() const { return node; }

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_drive;
  rclcpp::Publisher<autorace_interfaces::msg::Ui2Driving>::SharedPtr publisher_ui2drive;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr psd_front_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr psd_left_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr psd_right_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flag_sub_;
  rclcpp::Subscription<autorace_interfaces::msg::VisionHyun>::SharedPtr vision_traffic_sub_;
  //feed_YOLO
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr yolo_sub_;

  void imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void psd_front_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void psd_left_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void psd_right_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void flag_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void vision_traffic_callback(const autorace_interfaces::msg::VisionHyun::SharedPtr msg);
  void yolo_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  int sockfd_;
  std::thread udp_thread_;
  bool is_running_;
  void udp_receive_loop();

  size_t count_drive;
  size_t count_state;

  QTimer *new_timer1;
  QTimer *new_timer2;
  std::atomic<double> last_udp_time_sec_;

Q_SIGNALS:
  void rosShutDown();
  void imageReceived(const QPixmap &pixmap, int index);
};

#endif /* ui_test_QNODE_HPP_ */