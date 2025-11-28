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
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "controlitem.hpp"
#endif
#include <QThread>
#include <QTimer> 

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
  void state_callback();

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_drive;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_state;
  size_t count_drive;
  size_t count_state;

  QTimer *new_timer1;
  QTimer *new_timer2;

Q_SIGNALS:
  void rosShutDown();
};

#endif /* ui_test_QNODE_HPP_ */
