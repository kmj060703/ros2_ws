#include "../include/vision_hyun/vision_hyun.hpp"
#include "autorace_interfaces/msg/vision_hyun.hpp"

#include <thread>
#include <mutex>

/* 이미지가 YUYV 4:2:2 포맷으로 퍼블리시되고 있다
OpenCV cv::imshow에서 바로 BGR8으로 처리하려고 하면 채널 수 불일치 오류가 나
run : ros2 run vision_hyun vision_hyun_node */

std::mutex frame_mutex;
cv::Mat latest_frame;
cv::Mat latest_birdeye;
cv::Mat yellow_mask;
cv::Mat white_mask;

cv::Mat red_and_green_mask;

// 전역 변수 추가
int global_center_x = -1;
int global_yellow_x = -1;
int global_white_x = -1;
int global_yellow_diff = 0;
int global_white_diff = 0;
int yellow_x = -1;
int white_x = -1;
// 가로 640  세로 360
cv::Mat frame_copy, bird_copy, yellow_mask_copy, white_mask_copy, red_and_green_mask_copy;

void gui_thread()
{
    while (rclcpp::ok())
    {
        cv::Mat frame_display, bird_display, yellow_display, white_display, red_and_green_mask_copy;

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!latest_frame.empty())
                frame_display = latest_frame.clone();
            if (!bird_copy.empty())
                bird_display = bird_copy.clone();
            if (!yellow_mask.empty())
                yellow_display = yellow_mask.clone();
            if (!white_mask.empty())
                white_display = white_mask.clone();
            if (!red_and_green_mask.empty())
                red_and_green_mask_copy = red_and_green_mask.clone();
        }

        if (!frame_display.empty())
        {
            cv::line(frame_display, cv::Point(320, 0), cv::Point(320, 360), cv::Scalar(0, 0, 255), 1);
            cv::line(frame_display, cv::Point(0, 80), cv::Point(640, 80), cv::Scalar(0, 0, 255), 1);
            cv::imshow("Spedal Feed", frame_display);
        }

        if (!bird_display.empty())
        {
            cv::imshow("Bird's-Eye View", bird_display);
        }

        if (!yellow_display.empty())
        {
            cv::line(yellow_display, cv::Point(320, 0), cv::Point(320, 360), cv::Scalar(255, 255, 255), 1);
            cv::line(yellow_display, cv::Point(0, 270), cv::Point(640, 270), cv::Scalar(255, 255, 255), 1);
            cv::imshow("yellow_mask", yellow_display);
        }

        if (!white_display.empty())
        {
            cv::line(white_display, cv::Point(320, 0), cv::Point(320, 360), cv::Scalar(255, 255, 255), 1);
            cv::line(white_display, cv::Point(0, 270), cv::Point(640, 270), cv::Scalar(255, 255, 255), 1);
            cv::imshow("white_mask", white_display);
        }

        if (!red_and_green_mask.empty())
        {
            cv::line(red_and_green_mask, cv::Point(320, 0), cv::Point(320, 360), cv::Scalar(255, 255, 255), 1);
            cv::line(red_and_green_mask, cv::Point(0, 270), cv::Point(640, 270), cv::Scalar(255, 255, 255), 1);
            cv::imshow("traffic_mask", red_and_green_mask);
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
    publisher_2 = this->create_publisher<sensor_msgs::msg::Image>(
        "/vision/birdeye_raw", 10);
    publisher_3 = this->create_publisher<sensor_msgs::msg::Image>(
        "/vision/birdeye_total", 10);
    publisher_4 = this->create_publisher<autorace_interfaces::msg::VisionHyun>(
        "/vision/line_diff_info", 10);

    RCLCPP_INFO(this->get_logger(), "Image viewer node started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: /vision/image_processed");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /vision/birdeye_raw");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /vision/birdeye_total");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /vision/line_diff_info");

    cv::namedWindow("Spedal Feed");
    cv::namedWindow("Bird's-Eye View");
    cv::namedWindow("yellow_mask");
    cv::namedWindow("white_mask");
    cv::namedWindow("traffic_mask");
}

void ImageViewer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Receiving images: %dx%d, encoding: %s",
                             msg->width, msg->height, msg->encoding.c_str());

        cv::Mat frame;
        cv::Mat birdeye;
        cv::Mat birdeye_with_lines;
        cv::Mat total_birdeye;
        cv::Mat red_mask;
        cv::Mat green_mask;

        // RGB 또는 BGR 인코딩에 따라 처리
        if (msg->encoding == "rgb8")
        {
            // RGB8 데이터를 받아서 BGR로 변환
            cv::Mat rgb_frame(msg->height, msg->width, CV_8UC3, (void *)msg->data.data());
            cv::cvtColor(rgb_frame, frame, cv::COLOR_RGB2BGR);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }

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
            // 원본 이미지에서 사다리꼴 영역 (소실점 기준으로 도로/관심 영역)
            cv::Point2f src_pts[4] = {
                cv::Point2f(distort_L_top_x, distort_L_top_y),     // 왼쪽 위
                cv::Point2f(distort_R_top_x, distort_R_top_y),     // 오른쪽 위
                cv::Point2f(distort_L_under_x, distort_L_under_y), // 왼쪽 아래
                cv::Point2f(distort_R_under_x, distort_R_under_y)  // 오른쪽 아래
            };

            cv::Point2f dst_pts[4] = {
                cv::Point2f(flat_L_top_x, flat_L_top_y),     // 왼쪽 위
                cv::Point2f(flat_R_top_x, flat_R_top_y),     // 오른쪽 위
                cv::Point2f(flat_L_under_x, flat_L_under_y), // 왼쪽 아래
                cv::Point2f(flat_R_under_x, flat_R_under_y)  // 오른쪽 아래
            };

            cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
            cv::warpPerspective(frame, birdeye, M, cv::Size(640, 360));

            cv::Mat birdeye_blurred;
            cv::GaussianBlur(birdeye, birdeye_blurred, cv::Size(5, 5), 1.5);
            latest_birdeye = birdeye.clone();

            cv::Mat birdeye_hsv;
            cv::cvtColor(birdeye_blurred, birdeye_hsv, cv::COLOR_BGR2HSV);

            cv::Mat frame_hsv;
            cv::cvtColor(latest_frame, frame_hsv, cv::COLOR_BGR2HSV);

            cv::inRange(birdeye_hsv, lower_white, upper_white, white_mask);
            cv::inRange(birdeye_hsv, lower_yellow, upper_yellow, yellow_mask);
            cv::inRange(latest_frame, lower_red, upper_red, red_mask);
            cv::inRange(latest_frame, lower_green, upper_green, green_mask);

            cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            for (int i = 0; i < 5; i++)
            {
                cv::dilate(yellow_mask, yellow_mask, k);
                cv::dilate(white_mask, white_mask, k);
                cv::dilate(red_mask, red_mask, k);
                cv::dilate(green_mask, green_mask, k);
            }
            for (int i = 0; i < 10; i++)
            {
                cv::erode(yellow_mask, yellow_mask, k);
                cv::erode(white_mask, white_mask, k);
                cv::erode(red_mask, red_mask, k);
                cv::erode(green_mask, green_mask, k);
            }

            for (int i = detect_y_start; i < detect_y_end; i++)
            {
                for (int j = detect_x_start; j < detect_x_end; j++)
                {
                    if (red_mask.at<uchar>(i, j) > 0)
                    {
                        red_pixel_count++;
                    }
                    if (green_mask.at<uchar>(i, j) > 0)
                    {
                        green_pixel_count++;
                    }
                }
            }

            if (red_pixel_count > red_threshold)
            {
                traffic_light_state = 1; // 빨간불
            }
            else if (green_pixel_count > green_threshold)
            {
                traffic_light_state = 2; // 초록불
            }
            else
            {
                traffic_light_state = 0; // 신호등 없음
            }

            // y=scan_y 위치에서 노란색과 하얀색 선 찾기

            birdeye_with_lines = birdeye.clone();
            for (int i = 0; i < yellow_mask.rows; i++)
            {
                yellow_x = -1;
                white_x = -1;

                // 왼쪽 노란색 라인 찾기
                for (int j = 0; j < yellow_mask.cols; j++)
                {
                    if (yellow_mask.at<uchar>(i, j) > 0)
                    {
                        yellow_x = j;
                        cv::line(birdeye_with_lines, cv::Point(yellow_x, i), cv::Point(yellow_x, i), cv::Scalar(255, 255, 0), 1);
                        break;
                    }
                }

                // 오른쪽 흰색 라인 찾기
                for (int j = white_mask.cols - 1; j >= 0; j--)
                {
                    if (white_mask.at<uchar>(i, j) > 0)
                    {
                        white_x = j;
                        cv::line(birdeye_with_lines, cv::Point(white_x, i), cv::Point(white_x, i), cv::Scalar(255, 255, 255), 1);
                        break;
                    }
                }

                // 중앙선 계산 (수정됨)
                int center = -1;
                if (yellow_x != -1 && white_x != -1)
                {
                    center = (yellow_x + white_x) / 2;
                }
                else if (yellow_x != -1)
                {
                    center = yellow_x + 250; // 차선 폭 추정
                }
                else if (white_x != -1)
                {
                    center = white_x - 250;
                }

                // 중앙선 그리기 및 저장
                if (center != -1)
                {
                    cv::line(birdeye_with_lines, cv::Point(center, i), cv::Point(center, i), cv::Scalar(0, 255, 0), 1);
                    if (i == 270)
                    {
                        global_center_x = center;

                        global_yellow_x = yellow_x;
                        global_white_x = white_x;
                        global_yellow_diff = (yellow_x != -1) ? yellow_x - 320 : 0;
                        global_white_diff = (white_x != -1) ? white_x - 320 : 0;
                    }
                }
            }

            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 화면에 lineWrite 로직

            // 스캔 라인 표시 (y=270, 보라색)
            cv::line(birdeye_with_lines, cv::Point(0, 270), cv::Point(640, 270), cv::Scalar(255, 0, 255), 1);

            // 정보 표시
            if (global_center_x > 0)
            {
                cv::putText(birdeye_with_lines, "Yellow X: " + std::to_string(global_yellow_x) + " (diff: " + std::to_string(global_yellow_diff) + ")",
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
                cv::putText(birdeye_with_lines, "White X: " + std::to_string(global_white_x) + " (diff: " + std::to_string(global_white_diff) + ")",
                            cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
                cv::putText(birdeye_with_lines, "Center X: " + std::to_string(global_center_x),
                            cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }

            // bird_copy에 저장
            bird_copy = birdeye_with_lines.clone();

            // total_birdeye 생성
            total_birdeye = yellow_mask + white_mask;
            red_and_green_mask = red_mask + green_mask;
        }

        // Publish processed frames
        auto msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        auto msg_out2 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", birdeye_with_lines).toImageMsg();
        auto msg_out3 = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", total_birdeye).toImageMsg();
        auto msg_out4 = std::make_unique<autorace_interfaces::msg::VisionHyun>();
        msg_out4->header = msg->header;
        msg_out4->center_x = global_center_x;
        msg_out4->yellow_x = yellow_x;
        msg_out4->white_x = white_x;
        msg_out4->yellow_diff = global_yellow_diff;
        msg_out4->white_diff = global_white_diff;
        msg_out4->traffic_light = traffic_light_state; // 아무것도 아닐 때 0 || 빨간색 1 || 초록색 2

        publisher_->publish(*msg_out);
        publisher_2->publish(*msg_out2);
        publisher_3->publish(*msg_out3);
        publisher_4->publish(*msg_out4);
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
