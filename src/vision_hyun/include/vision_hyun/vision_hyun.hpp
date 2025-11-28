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

// mutex 기반 글로벌 변수
extern std::mutex frame_mutex;
extern cv::Mat latest_frame;
extern cv::Mat latest_birdeye;

class ImageViewer : public rclcpp::Node
{
public:
    ImageViewer();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

#endif // IMAGE_VIEWER_HPP
