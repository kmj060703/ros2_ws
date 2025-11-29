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
 
    cv::Scalar lower_white = cv::Scalar(0, 0, 120);
    cv::Scalar upper_white = cv::Scalar(180, 50, 255);
    cv::Scalar lower_yellow = cv::Scalar(20, 100, 100);
    cv::Scalar upper_yellow = cv::Scalar(30, 255, 255);

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
};

#endif // IMAGE_VIEWER_HPP