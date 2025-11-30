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
    image_topic,
    10,
    std::bind(&QNode::yoloImageCallback, this, std::placeholders::_1)
  );
  
  // 나중에 토픽 이름으로 변경해라~
  bird_image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    "/vision/birdeye_raw",
    10,
    std::bind(&QNode::birdImageCallback, this, std::placeholders::_1)
  );

  bird_image_sub_2_ = node->create_subscription<sensor_msgs::msg::Image>(
    "/vision/birdeye_total",
    10,
    std::bind(&QNode::birdImageCallback_2, this, std::placeholders::_1)
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
    if(l_start_flag_==0) publisher_drive->publish(msg);
}

void QNode::ui2drive_callback(){
  auto msg = autorace_interfaces::msg::Ui2Driving();
  msg.state_flag=l_state_flag_;
  msg.start_flag=start_flag_;
  msg.kp=kp_;
  msg.kd=kd_;
  msg.l_x=l_x_;
  msg.l_z=l_z_;
  msg.l_start_flag=l_start_flag_;
  msg.max_vel=max_vel_;
  publisher_ui2drive->publish(msg);
}

void QNode::yoloImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    // ===== cv_bridge 호출 전 완전한 검증 =====
    
    if (!msg) {
      RCLCPP_WARN(node->get_logger(), "Null yolo message");
      return;
    }
    
    // 정보 출력 (디버깅용)
    RCLCPP_INFO(node->get_logger(), 
      "Yolo RAW: width=%u, height=%u, step=%u, encoding=%s, data_size=%zu",
      msg->width, msg->height, msg->step, msg->encoding.c_str(), msg->data.size());
    
    // 크기 검증
    if (msg->width == 0 || msg->height == 0) {
      RCLCPP_ERROR(node->get_logger(), "Yolo: zero dimensions");
      return;
    }
    
    if (msg->width > 4096 || msg->height > 4096) {
      RCLCPP_ERROR(node->get_logger(), 
        "Yolo: dimensions too large: %ux%u", msg->width, msg->height);
      return;
    }
    
    // Step 검증 (step은 한 행의 바이트 수)
    if (msg->step == 0) {
      RCLCPP_ERROR(node->get_logger(), "Yolo: zero step");
      return;
    }
    
    // Step이 width보다 작으면 문제
    if (msg->step < msg->width) {
      RCLCPP_ERROR(node->get_logger(), 
        "Yolo: step(%u) < width(%u)", msg->step, msg->width);
      return;
    }
    
    // Step이 비정상적으로 크면 문제
    if (msg->step > msg->width * 100) {
      RCLCPP_ERROR(node->get_logger(), 
        "Yolo: step(%u) too large (width=%u)", msg->step, msg->width);
      return;
    }
    
    // 데이터 크기 검증
    if (msg->data.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Yolo: empty data");
      return;
    }
    
    size_t expected_min_size = msg->step * msg->height;
    if (msg->data.size() < expected_min_size) {
      RCLCPP_ERROR(node->get_logger(), 
        "Yolo: data size mismatch. Got %zu, expected at least %zu",
        msg->data.size(), expected_min_size);
      return;
    }
    
    // 최대 크기 제한 (100MB)
    if (msg->data.size() > 100000000) {
      RCLCPP_ERROR(node->get_logger(), 
        "Yolo: data too large: %zu bytes", msg->data.size());
      return;
    }
    
    // Encoding 검증
    if (msg->encoding.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Yolo: empty encoding");
      return;
    }
    
    // 지원하는 encoding만 처리
    if (msg->encoding != "bgr8" && msg->encoding != "rgb8" && 
        msg->encoding != "mono8" && msg->encoding != "8UC3" && 
        msg->encoding != "8UC1") {
      RCLCPP_ERROR(node->get_logger(), 
        "Yolo: unsupported encoding: %s", msg->encoding.c_str());
      return;
    }
    
    RCLCPP_INFO(node->get_logger(), "Yolo: validation passed, converting...");
    
    // ===== 이제 cv_bridge 호출 =====
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    
    if (!cv_ptr) {
      RCLCPP_ERROR(node->get_logger(), "Yolo: cv_ptr is null");
      return;
    }
    
    if (cv_ptr->image.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Yolo: cv_ptr->image is empty");
      return;
    }
    
    cv::Mat image = cv_ptr->image;
    
    RCLCPP_INFO(node->get_logger(), 
      "Yolo: Mat created: %dx%d, channels=%d", 
      image.cols, image.rows, image.channels());
    
    if (!image.isContinuous()) {
      image = image.clone();
    }
    
    QImage qimage(
      image.data,
      image.cols,
      image.rows,
      static_cast<int>(image.step),
      QImage::Format_RGB888
    );
    
    QImage rgb_image = qimage.rgbSwapped().copy();
    
    if (!rgb_image.isNull()) {
      RCLCPP_INFO(node->get_logger(), "Yolo: emitting image");
      emit imageReceived(QPixmap::fromImage(rgb_image), 0);
    }
    
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Yolo cv_bridge: %s", e.what());
  } catch (cv::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Yolo OpenCV: %s", e.what());
  } catch (std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Yolo std: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Yolo: unknown exception");
  }
}
void QNode::birdImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    if (!msg) {
      RCLCPP_WARN(node->get_logger(), "Null bird1 message");
      return;
    }
    
    // 정보 출력
    RCLCPP_INFO(node->get_logger(), 
      "Bird1 RAW: width=%u, height=%u, step=%u, encoding=%s, data_size=%zu",
      msg->width, msg->height, msg->step, msg->encoding.c_str(), msg->data.size());
    
    // 크기 검증
    if (msg->width == 0 || msg->height == 0) {
      RCLCPP_ERROR(node->get_logger(), "Bird1: zero dimensions");
      return;
    }
    
    if (msg->width > 4096 || msg->height > 4096) {
      RCLCPP_ERROR(node->get_logger(), 
        "Bird1: dimensions too large: %ux%u", msg->width, msg->height);
      return;
    }
    
    if (msg->step == 0) {
      RCLCPP_ERROR(node->get_logger(), "Bird1: zero step");
      return;
    }
    
    if (msg->step < msg->width) {
      RCLCPP_ERROR(node->get_logger(), 
        "Bird1: step(%u) < width(%u)", msg->step, msg->width);
      return;
    }
    
    if (msg->step > msg->width * 100) {
      RCLCPP_ERROR(node->get_logger(), 
        "Bird1: step(%u) too large", msg->step);
      return;
    }
    
    if (msg->data.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Bird1: empty data");
      return;
    }
    
    size_t expected_min_size = msg->step * msg->height;
    if (msg->data.size() < expected_min_size) {
      RCLCPP_ERROR(node->get_logger(), 
        "Bird1: data size mismatch. Got %zu, expected at least %zu",
        msg->data.size(), expected_min_size);
      return;
    }
    
    if (msg->data.size() > 100000000) {
      RCLCPP_ERROR(node->get_logger(), 
        "Bird1: data too large: %zu bytes", msg->data.size());
      return;
    }
    
    if (msg->encoding.empty() || 
        (msg->encoding != "bgr8" && msg->encoding != "rgb8" && 
         msg->encoding != "mono8")) {
      RCLCPP_ERROR(node->get_logger(), 
        "Bird1: unsupported encoding: %s", msg->encoding.c_str());
      return;
    }
    
    RCLCPP_INFO(node->get_logger(), "Bird1: validation passed, converting...");
    
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    
    if (!cv_ptr || cv_ptr->image.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Bird1: conversion failed");
      return;
    }
    
    cv::Mat image = cv_ptr->image;
    
    RCLCPP_INFO(node->get_logger(), 
      "Bird1: Mat created: %dx%d, channels=%d", 
      image.cols, image.rows, image.channels());
    
    if (!image.isContinuous()) {
      image = image.clone();
    }
    
    QImage qimage(
      image.data,
      image.cols,
      image.rows,
      static_cast<int>(image.step),
      QImage::Format_RGB888
    );
    
    QImage rgb_image = qimage.rgbSwapped().copy();
    
    if (!rgb_image.isNull()) {
      RCLCPP_INFO(node->get_logger(), "Bird1: emitting image");
      emit imageReceived(QPixmap::fromImage(rgb_image), 1);
    }
    
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Bird1 cv_bridge: %s", e.what());
  } catch (cv::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Bird1 OpenCV: %s", e.what());
  } catch (std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Bird1 std: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Bird1: unknown exception");
  }
}

void QNode::birdImageCallback_2(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    if (!msg) {
      RCLCPP_WARN(node->get_logger(), "Null bird2 message");
      return;
    }
    
    RCLCPP_INFO(node->get_logger(), 
      "Bird2 RAW: width=%u, height=%u, step=%u, encoding=%s, data_size=%zu",
      msg->width, msg->height, msg->step, msg->encoding.c_str(), msg->data.size());
    
    if (msg->width == 0 || msg->height == 0) {
      RCLCPP_ERROR(node->get_logger(), "Bird2: zero dimensions");
      return;
    }
    
    if (msg->width > 4096 || msg->height > 4096) {
      RCLCPP_ERROR(node->get_logger(), 
        "Bird2: dimensions too large: %ux%u", msg->width, msg->height);
      return;
    }
    
    if (msg->step == 0) {
      RCLCPP_ERROR(node->get_logger(), "Bird2: zero step");
      return;
    }
    
    if (msg->data.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Bird2: empty data");
      return;
    }
    
    size_t expected_min_size = msg->step * msg->height;
    if (msg->data.size() < expected_min_size) {
      RCLCPP_ERROR(node->get_logger(), 
        "Bird2: data size mismatch. Got %zu, expected at least %zu",
        msg->data.size(), expected_min_size);
      return;
    }
    
    if (msg->data.size() > 100000000) {
      RCLCPP_ERROR(node->get_logger(), 
        "Bird2: data too large: %zu bytes", msg->data.size());
      return;
    }
    
    if (msg->encoding.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Bird2: empty encoding");
      return;
    }
    
    RCLCPP_INFO(node->get_logger(), "Bird2: validation passed, converting...");
    
    cv_bridge::CvImagePtr cv_ptr;
    
    // encoding에 따라 다르게 처리
    if (msg->encoding == "mono8") {
      // mono8은 그대로 가져온 후 BGR로 변환
      cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
      if (!cv_ptr || cv_ptr->image.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Bird2: mono8 conversion failed");
        return;
      }
      
      // Grayscale을 BGR로 변환
      cv::Mat bgr_image;
      cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_GRAY2BGR);
      cv_ptr->image = bgr_image;
      
    } else if (msg->encoding == "bgr8") {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } else if (msg->encoding == "rgb8") {
      cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    } else {
      RCLCPP_ERROR(node->get_logger(), 
        "Bird2: unsupported encoding: %s", msg->encoding.c_str());
      return;
    }
    
    if (!cv_ptr || cv_ptr->image.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Bird2: conversion failed");
      return;
    }
    
    cv::Mat image = cv_ptr->image;
    
    RCLCPP_INFO(node->get_logger(), 
      "Bird2: Mat created: %dx%d, channels=%d", 
      image.cols, image.rows, image.channels());
    
    if (!image.isContinuous()) {
      image = image.clone();
    }
    
    QImage qimage(
      image.data,
      image.cols,
      image.rows,
      static_cast<int>(image.step),
      QImage::Format_RGB888
    );
    
    // bgr8이면 RGB로 변환, 아니면 그대로
    QImage rgb_image;
    if (msg->encoding == "bgr8" || msg->encoding == "mono8") {
      rgb_image = qimage.rgbSwapped().copy();
    } else {
      rgb_image = qimage.copy();
    }
    
    if (!rgb_image.isNull()) {
      RCLCPP_INFO(node->get_logger(), "Bird2: emitting image");
      emit imageReceived(QPixmap::fromImage(rgb_image), 2);
    }
    
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Bird2 cv_bridge: %s", e.what());
  } catch (cv::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Bird2 OpenCV: %s", e.what());
  } catch (std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Bird2 std: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Bird2: unknown exception");
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
