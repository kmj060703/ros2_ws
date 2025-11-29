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
  node = rclcpp::Node::make_shared("ui_test_node");
  new_timer1 = new QTimer(this);
  new_timer1->setInterval(100);
  new_timer2 = new QTimer(this);
  new_timer2->setInterval(100);
  publisher_drive = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 30);
  publisher_ui2drive = node->create_publisher<autorace_interfaces::msg::Ui2Driving>("/ui2driving_topic", 30);

  const std::string image_topic = "/vision/image_processed";
  yolo_image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    image_topic,  // 실제 토픽 이름으로 변경
    10,
    std::bind(&QNode::yoloImageCallback, this, std::placeholders::_1)
  );
  
  // Bird's eye view 이미지 토픽 구독 (실제 토픽 이름으로 변경)
  bird_image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    "/bird/image",  // 실제 토픽 이름으로 변경
    10,
    std::bind(&QNode::birdImageCallback, this, std::placeholders::_1)
  );

  connect(new_timer1, &QTimer::timeout, this, &QNode::drive_callback);
  connect(new_timer2, &QTimer::timeout, this, &QNode::ui2drive_callback);

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

void QNode::ui2drive_callback(){
  auto msg = autorace_interfaces::msg::Ui2Driving();
  msg.state_flag=l_state_flag_;
  msg.kp=kp_;
  msg.kd=kd_;
  msg.l_x=l_x_;
  msg.l_z=l_z_;
  msg.l_start_flag=l_start_flag_;
  publisher_ui2drive->publish(msg);
}

void QNode::yoloImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    
    QImage qimage(
      image.data,
      image.cols,
      image.rows,
      image.step,
      QImage::Format_RGB888
    );
    
    QImage rgb_image = qimage.rgbSwapped();
    
    // index 0으로 yolo 이미지 전송
    emit imageReceived(QPixmap::fromImage(rgb_image), 0);
    
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void QNode::birdImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    
    QImage qimage(
      image.data,
      image.cols,
      image.rows,
      image.step,
      QImage::Format_RGB888
    );
    
    QImage rgb_image = qimage.rgbSwapped();
    
    // index 1로 bird 이미지 전송
    emit imageReceived(QPixmap::fromImage(rgb_image), 1);
    
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
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
