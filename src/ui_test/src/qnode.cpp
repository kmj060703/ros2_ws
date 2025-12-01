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
  if (sockfd_ > 0) close(sockfd_);
  if (udp_thread_.joinable()) udp_thread_.join();

  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

// UDP 수신 및 이미지 처리
void QNode::udp_receive_loop() {
    struct sockaddr_in servaddr, cliaddr;

    // 소켓 생성
    if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        RCLCPP_ERROR(node->get_logger(), "소켓 생성 실패");
        return;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(UDP_PORT);

    // Bind
    if (bind(sockfd_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        RCLCPP_ERROR(node->get_logger(), "Bind 실패 Port %d", UDP_PORT);
        close(sockfd_);
        return;
    }

    RCLCPP_INFO(node->get_logger(), "UDP Start Port %d", UDP_PORT);

    while (is_running_ && rclcpp::ok()) {
        int header[2]; // ID, Size
        socklen_t len = sizeof(cliaddr);

        // ID와 크기 수신
        int n = recvfrom(sockfd_, header, sizeof(header), 0, (struct sockaddr *)&cliaddr, &len);

        if (n == sizeof(header)) {
            int img_id = header[0];
            int total_size = header[1];

            if (total_size > 0 && total_size < 10000000) {
                std::vector<uchar> buffer(total_size);
                int received_bytes = 0;
                bool packet_loss = false;

                // 2. 이미지 데이터 조각 수신
                while (received_bytes < total_size) {
                    int chunk_size = std::min(PACKET_SIZE, total_size - received_bytes);
                    n = recvfrom(sockfd_, &buffer[received_bytes], chunk_size, 0, (struct sockaddr *)&cliaddr, &len);
                    if (n < 0) { packet_loss = true; break; }
                    received_bytes += n;
                }

                // 디코딩 및 UI 업데이트
                if (!packet_loss && received_bytes == total_size) {
                    cv::Mat frame = cv::imdecode(buffer, cv::IMREAD_COLOR);
                    if (!frame.empty()) {
                        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
                        QImage qimage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
                        emit imageReceived(QPixmap::fromImage(qimage.copy()), img_id);
                    }
                }
            }
        }
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
      msg.linear.x=0.0;
      msg.angular.z=z_;
    }
    else if(left_right_==-1){
      msg.linear.x=0.0;
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
    if(l_start_flag_==0) publisher_drive->publish(msg);
}

void QNode::ui2drive_callback(){
  auto msg = autorace_interfaces::msg::Ui2Driving();
  msg.state_flag=l_state_flag_;
  msg.start_flag=start_flag_;
  msg.kp=kp_;
  msg.kd=kd_;
  msg.def_turn_x=l_x_;
  msg.def_turn_z=l_z_;
  msg.l_start_flag=l_start_flag_;
  msg.max_vel=max_vel_;
  publisher_ui2drive->publish(msg);
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