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
  void run() override;
  std::shared_ptr<rclcpp::Node> getNode() const { return node; }
  int count=0;

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_drive;
  rclcpp::Publisher<autorace_interfaces::msg::Ui2Driving>::SharedPtr publisher_ui2drive;

  int sockfd_;
  std::thread udp_thread_;
  bool is_running_;
  void udp_receive_loop();

  size_t count_drive;
  size_t count_state;

  QTimer *new_timer1;
  QTimer *new_timer2;

  double x_ = 0.0;
  double z_ = 0.0;
  int forw_back_ = 0;
  int left_right_ = 0;
  int start_flag_ = 0;
  int l_start_flag_ = 0;
  int l_state_flag_ = 0;
  double kp_ = 0.0;
  double kd_ = 0.0;
  double l_x_ = 0.0;
  double l_z_ = 0.0;
  double max_vel_ = 0.0;


Q_SIGNALS:
  void rosShutDown();
  void imageReceived(const QPixmap &pixmap, int index);
};

#endif /* ui_test_QNODE_HPP_ */