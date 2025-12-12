#ifndef IMAGE_VIEWER_HPP
#define IMAGE_VIEWER_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>
#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include <mutex>
#include "autorace_interfaces/msg/vision_hyun.hpp"

// mutex 기반 글로벌 변수
extern std::mutex frame_mutex;
extern cv::Mat latest_frame;
extern cv::Mat latest_birdeye;
extern cv::Mat yellow_mask;
extern cv::Mat white_mask;
extern cv::Mat red_mask;
extern cv::Mat green_mask;
extern cv::Mat red_and_green_mask;
extern cv::Mat brown_mask,brown_mask_2;
extern cv::Mat frame_copy, bird_copy, yellow_mask_copy, white_mask_copy, red_and_green_mask_copy, brown_copy;

// 전역 변수 추가
extern int global_center_x;
extern int global_yellow_x;
extern int global_white_x;
extern int global_yellow_diff;
extern int global_white_diff;
extern int yellow_x;
extern int white_x;

// 신호등 감지 여부
extern int traffic_light_state;
extern int brown_pixel_count;

// red && green 탐지 범위
extern int detect_x_start;
extern int detect_x_end;
extern int detect_y_start;
extern int detect_y_end;

class ImageViewer : public rclcpp::Node
{
public:
  ImageViewer();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_2;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_3;
  rclcpp::Publisher<autorace_interfaces::msg::VisionHyun>::SharedPtr
      publisher_4;

  cv::Scalar lower_white = cv::Scalar(0, 0, 180);
  cv::Scalar upper_white = cv::Scalar(180, 50, 255);
  cv::Scalar lower_yellow = cv::Scalar(20, 100, 100); // H 179 // S 225 // V 225
  cv::Scalar upper_yellow = cv::Scalar(30, 255, 255);
  cv::Scalar lower_red = cv::Scalar(0, 80, 80);
  cv::Scalar upper_red = cv::Scalar(0, 100, 200);
  cv::Scalar lower_t_yellow = cv::Scalar(150, 3, 70);
  cv::Scalar upper_t_yellow = cv::Scalar(175, 8, 255);
  //  cv::Scalar lower_green = cv::Scalar(140, 35, 27);
  //  cv::Scalar upper_grㅑeen = cv::Scalar(170, 160, 190); // ㅈㄴ 신기하네이건

  cv::Scalar lower_green = cv::Scalar(160, 50, 50);
  cv::Scalar upper_green = cv::Scalar(179, 255, 255);

  cv::Scalar lower_brown = cv::Scalar(0, 10, 30);
  cv::Scalar upper_brown = cv::Scalar(15, 190, 195);
  cv::Scalar lower_brown_2 = cv::Scalar(165, 10, 30);
  cv::Scalar upper_brown_2 = cv::Scalar(180, 190, 195);

  // 사다리꼴 원본 좌표 (feed 640x360 기준)
  float distort_L_top_x = 160.0;
  float distort_L_top_y = 250.0;
  float distort_R_top_x = 640.0 - distort_L_top_x;
  float distort_R_top_y = distort_L_top_y;
  float distort_L_under_x = 72.0;
  float distort_L_under_y = 360.0;
  float distort_R_under_x = 640.0 - distort_L_under_x;
  float distort_R_under_y = distort_L_under_y;

  // 직사각형 목적지 좌표 (이미지 전체 640x360)
  float flat_L_top_x = 120.0;
  float flat_L_top_y = 0.0;
  float flat_R_top_x = 640 - flat_L_top_x;
  float flat_R_top_y = 0.0;
  float flat_L_under_x = 120.0;
  float flat_L_under_y = 360.0;
  float flat_R_under_x = 640 - flat_L_under_x;
  float flat_R_under_y = 360.0;

  // 라인 감지 기억용 변수
  int last_yellow_x = -1;
  int last_white_x = -1;
  bool yellow_detected_before = false;
  bool white_detected_before = false;

  int global_center_x = -1;
  int global_yellow_x = -1;
  int global_white_x = -1;
  int global_yellow_diff = 0;
  int global_white_diff = 0;

  // 기준 x좌표
  const int reference_x = 320;
  const int scan_y = 270;

  // detect 영역의 빨간색/초록색 픽셀 카운트
  int red_pixel_count = 0;
  int green_pixel_count = 0;
  int yellow_pixel_count = 0 ;

  int brown_pixel_count=0;
  // 신호등 상태 판단
  int red_threshold = 150;
  int green_threshold = 250;
  int yellow_threshold = 300;


};

#endif // IMAGE_VIEWER_HPP