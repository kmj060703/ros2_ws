/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date August 2024
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/ui_test/qnode.hpp"

QNode::QNode()
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("ui_test");
  new_timer1 = new QTimer(this);
  new_timer1->setInterval(100);
  new_timer2 = new QTimer(this);
  new_timer2->setInterval(100);
  publisher_drive = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 30);
  publisher_state = node->create_publisher<std_msgs::msg::Int32>("/my_topic", 30);

  const std::string image_topic = "/vision/image_processed";
  image_sub = node->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10,
            std::bind(&QNode::image_callback, this, std::placeholders::_1)
        );

  connect(new_timer1, &QTimer::timeout, this, &QNode::drive_callback);
  connect(new_timer2, &QTimer::timeout, this, &QNode::state_callback);

  new_timer1->start();
  new_timer2->start();

  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::drive_callback(){
  auto msg = geometry_msgs::msg::Twist();
  if(start_flag_==1){
    if(forw_back_==1){
      msg.linear.x=x_;
      msg.angular.z=0;
    }
    else if(forw_back_==-1){
      msg.linear.x=-x_;
      msg.angular.z=0;
    }
    else if(left_right_==1){
      msg.linear.x=x_;
      msg.angular.z=z_;
    }
    else if(left_right_==-1){
      msg.linear.x=x_;
      msg.angular.z=-z_;
    }
    else{
      msg.linear.x=0;
      msg.angular.z=0;
    }
    msg.linear.y=0;
    msg.linear.z=0;
    msg.angular.x=0;
    msg.angular.y=0;
    std::cout<<"linear.x:"<<msg.linear.x<<std::endl;
    std::cout<<"linear.y:"<<msg.linear.y<<std::endl;
    std::cout<<"linear.z:"<<msg.linear.z<<std::endl;
    std::cout<<"angular.x:"<<msg.angular.x<<std::endl;
    std::cout<<"angular.y:"<<msg.angular.y<<std::endl;
    std::cout<<"angular.z:"<<msg.angular.z<<std::endl;
    std::cout<<"-----------------------"<<std::endl;
    publisher_drive->publish(msg);
  }
}

void QNode::state_callback(){
  auto msg = std_msgs::msg::Int32();
  msg.data=state_flag_;
  publisher_state->publish(msg);
}

void QNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("QNode"), "cv_bridge exception: %s", e.what());
    }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}
