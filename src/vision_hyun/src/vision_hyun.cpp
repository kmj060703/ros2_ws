#include "../include/vision_hyun/vision_hyun.hpp"
#include <thread>
#include <mutex>

// 640 360
std::mutex frame_mutex;
cv::Mat latest_frame;
cv::Mat latest_birdeye;

/* 이미지가 YUYV 4:2:2 포맷으로 퍼블리시되고 있다
OpenCV cv::imshow에서 바로 BGR8으로 처리하려고 하면 채널 수 불일치 오류가 나
run : ros2 run vision_hyun vision_hyun_node */


void gui_thread()
{
    while (rclcpp::ok())
    {
        cv::Mat frame_copy, bird_copy;

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!latest_frame.empty())
                frame_copy = latest_frame.clone();
            if (!latest_birdeye.empty())
                bird_copy = latest_birdeye.clone();
        }

        if (!frame_copy.empty())
        {
            cv::line(frame_copy, cv::Point(270, 0), cv::Point(270, 640), cv::Scalar(0, 0, 255), 3);
            cv::imshow("Spedal Feed", frame_copy);
        }

        if (!bird_copy.empty())
        {
            cv::imshow("Bird's-Eye View", bird_copy);
        }

        cv::waitKey(1);
    }
}

ImageViewer::ImageViewer()
    : Node("vision_hyun_node")
{
    const std::string image_topic = "/default_camera/image_raw";

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 10,
        std::bind(&ImageViewer::image_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/vision/image_processed", 10);

    RCLCPP_INFO(this->get_logger(), "Image viewer node started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: /vision/image_processed");

    cv::namedWindow("Spedal Feed");
    cv::namedWindow("Bird's-Eye View");
}

void ImageViewer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Receiving images: %dx%d", msg->width, msg->height);

        cv::Mat yuyv(msg->height, msg->width, CV_8UC2, (void *)msg->data.data());
        cv::Mat frame;
        cv::cvtColor(yuyv, frame, cv::COLOR_YUV2HSV_YUY2);

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty frame!");
            return;
        }

        // 프레임 업데이트
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            latest_frame = frame.clone();

            // Bird's-Eye View
            cv::Mat hsv_frame;
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

            cv::Point2f src_pts[4] = {
                cv::Point2f(72.5, 103.125),
                cv::Point2f(247.5, 103.125),
                cv::Point2f(35, 161.0625),
                cv::Point2f(285, 161.0625)};

            cv::Point2f dst_pts[4] = {
                cv::Point2f(64.5, 45),
                cv::Point2f(255.5, 45),
                cv::Point2f(64.5, 135),
                cv::Point2f(255.5, 135)};

            cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
            cv::warpPerspective(hsv_frame, latest_birdeye, M, cv::Size(640, 360));
        }

        // Publish processed frame
        auto msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg_out);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (getenv("DISPLAY") == nullptr)
    {
        std::cerr << "ERROR: DISPLAY environment variable not set!" << std::endl;
        return 1;
    }

    auto node = std::make_shared<ImageViewer>();
    std::thread t(gui_thread);
    t.detach();

    RCLCPP_INFO(node->get_logger(), "Starting spin...");
    rclcpp::spin(node);

    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
