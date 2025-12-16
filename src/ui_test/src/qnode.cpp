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
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <vector>

#define UDP_PORT 9999
#define PACKET_SIZE 4096
using namespace std;
using namespace std::placeholders;

QNode::QNode()
{
  auto sensor_qos = rclcpp::SensorDataQoS();
  int argc = 0;
  char **argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("ui_test_node");
  new_timer1 = new QTimer(this);
  new_timer1->setInterval(100);
  new_timer2 = new QTimer(this);
  new_timer2->setInterval(100);
    imu_sub_ = node->create_subscription<geometry_msgs::msg::Vector3>(
        "imu_angle",
        sensor_qos,
        std::bind(&QNode::imu_callback, this, _1));

    psd_front_sub_ = node->create_subscription<std_msgs::msg::Int32>(
        "psd_result_front", sensor_qos, std::bind(&QNode::psd_front_callback, this, _1));

    psd_left_sub_ = node->create_subscription<std_msgs::msg::Int32>(
        "psd_result_left", sensor_qos, std::bind(&QNode::psd_left_callback, this, _1));

    psd_right_sub_ = node->create_subscription<std_msgs::msg::Int32>(
        "psd_result_right", sensor_qos, std::bind(&QNode::psd_right_callback, this, _1));

    flag_sub_ = node->create_subscription<std_msgs::msg::Int32>(
        "master_jo_flag",
        sensor_qos,
        std::bind(&QNode::flag_callback, this, _1));
    vision_traffic_sub_ = node->create_subscription<autorace_interfaces::msg::VisionHyun>(
        "/vision/line_diff_info",
        sensor_qos,
        std::bind(&QNode::vision_traffic_callback, this, _1));

    yolo_sub_= node->create_subscription<sensor_msgs::msg::Image>("feed_YOLO",10,std::bind(&QNode::yolo_callback, this, std::placeholders::_1));

  publisher_drive = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", sensor_qos);
  publisher_ui2drive = node->create_publisher<autorace_interfaces::msg::Ui2Driving>("/ui2driving_topic", sensor_qos);

  connect(new_timer1, &QTimer::timeout, this, &QNode::drive_callback);
  connect(new_timer2, &QTimer::timeout, this, &QNode::ui2drive_callback);

  new_timer1->start();
  new_timer2->start();

  is_running_ = true;
  udp_thread_ = std::thread(&QNode::udp_receive_loop, this);

  this->start();
}

QNode::~QNode()
{
  is_running_ = false;
  if (sockfd_ > 0)
    close(sockfd_);
  if (udp_thread_.joinable())
    udp_thread_.join();

  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    imu_yaw = msg->z;
    // RCLCPP_INFO(this->get_logger(), "Yaw: %.2f 도", current_yaw_ * 180.0 / M_PI);
}

void QNode::psd_front_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    psd_flag[1] = msg->data;
    // RCLCPP_WARN(this->get_logger(), "정면 장애물 감지 여부: %d", is_front_danger_);
}

void QNode::psd_left_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    psd_flag[0] = msg->data;
    // RCLCPP_INFO(this->get_logger(), "왼쪽 장애물 감지 여부: %d", is_left_danger_);
}

void QNode::psd_right_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    psd_flag[2] = msg->data;
    // RCLCPP_INFO(this->get_logger(), "오른쪽 장애물 감지 여부: %d", is_right_danger_);
}

void QNode::flag_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    driving_state = msg->data;
    // RCLCPP_INFO(this->get_logger(), "플래그: %d", mission_flag_);
}

void QNode::vision_traffic_callback(const autorace_interfaces::msg::VisionHyun::SharedPtr msg)
{
    traffic_state = msg->traffic_light;
    brown_count =msg->brown_count;
    yellow_count =msg->yellowline_count;
    white_count =msg->whiteline_count;


    if (traffic_state == 1)
    {
        // RCLCPP_INFO(this->get_logger(), "빨간불 감지 %d", traffic_light_status_);
    }
    else if (traffic_state == 2)
    {
        // RCLCPP_INFO(this->get_logger(), "초록불 감지 %d", traffic_light_status_);
    }
    else
    {
        // RCLCPP_INFO(this->get_logger(), "감지 안됨 혹은 노란색 %d", traffic_light_status_);
    }
}


void QNode::yolo_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    img_id = 3;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(
            msg,
            sensor_msgs::image_encodings::RGB8
        );
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB);
    }
    catch (cv_bridge::Exception &e)
    {
        return;
    }

    cv::Mat &frame = cv_ptr->image;

    QImage qimage(
        frame.data,
        frame.cols,
        frame.rows,
        frame.step,
        QImage::Format_RGB888
    );
    emit imageReceived(QPixmap::fromImage(qimage.copy()), img_id);
}

// UDP 수신 및 이미지 처리
void QNode::udp_receive_loop()
{
  struct sockaddr_in servaddr, cliaddr;

  // 소켓 생성
  if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    RCLCPP_ERROR(node->get_logger(), "소켓 생성 실패");
    return;
  }

  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(UDP_PORT);

  // Bind
  if (bind(sockfd_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
  {
    RCLCPP_ERROR(node->get_logger(), "Bind 실패 Port %d", UDP_PORT);
    close(sockfd_);
    return;
  }

  RCLCPP_INFO(node->get_logger(), "UDP Start Port %d", UDP_PORT);

  while (is_running_ && rclcpp::ok())
  {
    int header[2]; // ID, Size
    socklen_t len = sizeof(cliaddr);

    // ID와 크기 수신
    int n = recvfrom(sockfd_, header, sizeof(header), 0, (struct sockaddr *)&cliaddr, &len);

    if (n == sizeof(header))
    {
      img_id = header[0];
      int total_size = header[1];

      if (total_size > 0 && total_size < 10000000)
      {
        std::vector<uchar> buffer(total_size);
        int received_bytes = 0;
        bool packet_loss = false;

        // 2. 이미지 데이터 조각 수신
        while (received_bytes < total_size)
        {
          int chunk_size = std::min(PACKET_SIZE, total_size - received_bytes);
          n = recvfrom(sockfd_, &buffer[received_bytes], chunk_size, 0, (struct sockaddr *)&cliaddr, &len);
          if (n < 0)
          {
            packet_loss = true;
            break;
          }
          received_bytes += n;
        }

        // 디코딩 및 UI 업데이트
        if (!packet_loss && received_bytes == total_size)
        {
          frame = cv::imdecode(buffer, cv::IMREAD_COLOR); // mat 형태로 변환..
          vision_helper(frame, img_id);
          if (!frame.empty())
          {
            last_udp_time_sec_.store(node->now().seconds()); //time stamp

            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
            QImage qimage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
            emit imageReceived(QPixmap::fromImage(qimage.copy()), img_id);
          }
        }
      }
    }
  }
}

void QNode::drive_callback()
{
  auto msg = geometry_msgs::msg::Twist();
  if (start_flag_ == 1)
  {
    if (forw_back_ == 1)
    {
      msg.linear.x = x_;
      msg.angular.z = 0;
    }
    else if (forw_back_ == -1)
    {
      msg.linear.x = -x_;
      msg.angular.z = 0;
    }
    else if (left_right_ == 1)
    {
      msg.linear.x = 0.0;
      msg.angular.z = z_;
    }
    else if (left_right_ == -1)
    {
      msg.linear.x = 0.0;
      msg.angular.z = -z_;
    }
    else
    {
      msg.linear.x = 0;
      msg.angular.z = 0;
    }
  }
  else
  {
    msg.linear.x = 0;
    msg.angular.z = 0;
  }
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  if (l_start_flag_ == 0)
    publisher_drive->publish(msg);
}

void QNode::ui2drive_callback()
{
  
  double now = node->now().seconds();

  if (now - last_udp_time_sec_.load() > 0.2) {
      udp_crack = 1;
  }
  else udp_crack=0;
  
  auto msg = autorace_interfaces::msg::Ui2Driving();
  msg.vision_lost_flag=udp_crack;
  msg.state_flag = l_state_flag_;
  msg.start_flag = start_flag_;
  msg.kp = kp_;
  msg.kd = kd_;
  msg.def_turn_x = l_x_;
  msg.def_turn_z = l_z_;
  msg.l_start_flag = l_start_flag_;
  msg.max_vel = max_vel_;
  publisher_ui2drive->publish(msg);
}

void QNode::vision_helper(cv::Mat image, int img_id)
{
  if (img_id == 1)
  {
    if (vision_hsv_state < 4)
    { // 스케일러 세팅
      if (vision_hsv_state == 1)
      {
        lower_l_white = cv::Scalar(HSV_low[0], HSV_low[1], HSV_low[2]);
        upper_l_white = cv::Scalar(HSV_high[0], HSV_high[1], HSV_high[2]);
      }
      else if (vision_hsv_state == 2)
      {
        lower_l_yellow = cv::Scalar(HSV_low[3], HSV_low[4], HSV_low[5]);
        upper_l_yellow = cv::Scalar(HSV_high[3], HSV_high[4], HSV_high[5]);
      }
      else if (vision_hsv_state == 3)
      {
        lower_l_red = cv::Scalar(HSV_low[6], HSV_low[7], HSV_low[8]);
        upper_l_red = cv::Scalar(HSV_high[6], HSV_high[7], HSV_high[8]);
      }
    }
    cv::cvtColor(image, birdeye_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(birdeye_hsv, lower_l_white, upper_l_white, line_white_mask);
    cv::inRange(birdeye_hsv, lower_l_yellow, upper_l_yellow, line_yellow_mask);
    cv::inRange(birdeye_hsv, lower_l_red, upper_l_red, red_l_mask);
    //
    if (vision_hsv_state == 1)
    {
      image = line_white_mask;
      img_id=4;
    }
    else if (vision_hsv_state == 2)
    {
      image = line_yellow_mask;
      img_id=5;
    }
    else if (vision_hsv_state == 3)
    {
      image = red_l_mask;
      img_id=6;
    }
    if (!image.empty())
{
    cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);

    QImage qimage(image.data,
                  image.cols,
                  image.rows,
                  image.step,
                  QImage::Format_RGB888);

    emit imageReceived(QPixmap::fromImage(qimage.copy()), img_id);
}

  }
  else if (img_id == 0)
  {
    if (vision_hsv_state < 6)
    {
      if (vision_hsv_state == 4)
      {
        lower_t_red = cv::Scalar(HSV_low[9], HSV_low[10], HSV_low[11]);
        upper_t_red = cv::Scalar(HSV_high[9], HSV_high[10], HSV_high[11]);
      }
      else if (vision_hsv_state == 5)
      {
        lower_t_yellow = cv::Scalar(HSV_low[12], HSV_low[13], HSV_low[14]);
        upper_t_yellow = cv::Scalar(HSV_high[12], HSV_high[13], HSV_high[14]);
      }
    }
    else
    {
      if (vision_hsv_state == 6)
      {
        lower_t_green = cv::Scalar(HSV_low[15], HSV_low[16], HSV_low[17]);
        upper_t_green = cv::Scalar(HSV_high[15], HSV_high[16], HSV_high[17]);
      }
      else if (vision_hsv_state == 7)
      {
        lower_brown = cv::Scalar(HSV_low[18], HSV_low[19], HSV_low[20]);
        upper_brown = cv::Scalar(HSV_high[18], HSV_high[19], HSV_high[20]);
      }
    }
    // frame_hsv = image.clone();;

    cv::cvtColor(image, frame_hsv, cv::COLOR_BGR2HSV);

    // t는 traffic
    cv::inRange(frame_hsv, lower_t_red, upper_t_red, tra_red_mask);
    cv::inRange(frame_hsv, lower_t_green, upper_t_green, tra_green_mask);
    cv::inRange(frame_hsv, lower_t_yellow, upper_t_yellow, tra_yellow_mask);
    cv::inRange(frame_hsv, lower_brown, upper_brown, brown_mask);
    if (vision_hsv_state == 4)
    {
      image = tra_red_mask;
      img_id=7;
    }
    else if (vision_hsv_state == 5)
    {
      image = tra_yellow_mask;
      img_id=8;
    }
    else if (vision_hsv_state == 6)
    {
      image = tra_green_mask;
      img_id=9;
    }
    else if (vision_hsv_state == 7)
    {
      image = brown_mask;
      img_id=10;
    }
    if (!image.empty())
{
    cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);

    QImage qimage(image.data,
                  image.cols,
                  image.rows,
                  image.step,
                  QImage::Format_RGB888);

    emit imageReceived(QPixmap::fromImage(qimage.copy()), img_id);
}

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