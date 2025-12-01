#include "../include/vision_hyun/vision_hyun.hpp"
#include "autorace_interfaces/msg/vision_hyun.hpp"

#include <thread>
#include <mutex>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#define REMOTE_IP "223.194.43.127"
#define REMOTE_PORT 9999
#define PACKET_SIZE 4096

int udp_sock = -1;
struct sockaddr_in remote_addr;

// mutex 기반 글로벌 변수
std::mutex frame_mutex;
cv::Mat latest_frame;
cv::Mat latest_birdeye;
cv::Mat yellow_mask;
cv::Mat white_mask;
cv::Mat red_mask;
cv::Mat green_mask;
cv::Mat red_and_green_mask;
cv::Mat frame_copy, bird_copy, yellow_mask_copy, white_mask_copy, red_and_green_mask_copy;

// 전역 변수 추가
int global_center_x = -1;
int global_yellow_x = -1;
int global_white_x = -1;
int global_yellow_diff = 0;
int global_white_diff = 0;
int yellow_x = -1;
int white_x = -1;

// 신호등 감지 여부
int traffic_light_state = 0;

// red && green 탐지 범위
int detect_x_start = 320;
int detect_x_end = 640;
int detect_y_start = 0;
int detect_y_end = 80;

/* 이미지가 YUYV 4:2:2 포맷으로 퍼블리시되고 있다
OpenCV cv::imshow에서 바로 BGR8으로 처리하려고 하면 채널 수 불일치 오류가 나
run : ros2 run vision_hyun vision_hyun_node */

void send_udp_image(cv::Mat &img, int id, rclcpp::Logger logger)
{
    if (img.empty() || udp_sock < 0) {
        RCLCPP_WARN(logger, "이미지 X or UDP 준비 X");
        return;
    }

    std::vector<uchar> encoded;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
    bool success = cv::imencode(".jpg", img, encoded, params);

    if (!success) {
        RCLCPP_ERROR(logger, "cv::imencode 실패 image ID: %d", id);
        return;
    }

    int total_size = encoded.size();
    if (total_size == 0) {
        RCLCPP_WARN(logger, "엔코딩 사이즈: 0 ID: %d", id);
        return;
    }

    RCLCPP_INFO(logger, "이미지 pub ID: %d, 엔코딩 사이즈: %d bytes", id, total_size);

    int header[2] = {id, total_size};
    ssize_t sent_header_bytes = sendto(udp_sock, header, sizeof(header), 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
    if (sent_header_bytes < 0) {
        RCLCPP_ERROR(logger, "pub 실패 for header: %s", strerror(errno));
        return;
    }
    RCLCPP_INFO(logger, "header: %ld bytes", sent_header_bytes);


    int sent_bytes = 0;
    while (sent_bytes < total_size)
    {
        int chunk_size = std::min(PACKET_SIZE, total_size - sent_bytes);
        ssize_t sent_chunk_bytes = sendto(udp_sock, &encoded[sent_bytes], chunk_size, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
        
        if (sent_chunk_bytes < 0) {
            RCLCPP_ERROR(logger, "pub 실패 for chunk: %s", strerror(errno));
            break; 
        }
        sent_bytes += sent_chunk_bytes;
        // usleep(100); // 네트워크가 너무 빠르면 패킷 손실 방지용
    }
    RCLCPP_INFO(logger, "byte for ID %d: %d / %d", id, sent_bytes, total_size);
}

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
            std::stringstream ss;
            ss << "Traffic Light flag : " << traffic_light_state;
            std::string text_to_display = ss.str();

            cv::line(frame_display, cv::Point(320, 0), cv::Point(320, 360), cv::Scalar(0, 0, 255), 1);
            cv::line(frame_display, cv::Point(0, 80), cv::Point(640, 80), cv::Scalar(0, 0, 255), 1);
            cv::rectangle(
                frame_display,
                cv::Point(detect_x_start, detect_y_start),
                cv::Point(detect_x_end, detect_y_end),
                cv::Scalar(0, 255, 0), 2);

            cv::putText(
                frame_display,
                text_to_display,
                cv::Point(detect_x_start, detect_y_start - 5),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

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

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>( // raw_camera
        "/vision/image_processed", 10);
    publisher_2 = this->create_publisher<sensor_msgs::msg::Image>( // 가운데 선그리는 raw_birdeye
        "/vision/birdeye_raw", 10);
    publisher_3 = this->create_publisher<sensor_msgs::msg::Image>( // hsv 변환한 birdeye
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
        // 1. ROS 이미지 -> OpenCV Mat 변환
        cv::Mat frame;

        if (msg->encoding == "rgb8")
        {
            cv::Mat rgb_frame(msg->height, msg->width, CV_8UC3, (void *)msg->data.data());
            cv::cvtColor(rgb_frame, frame, cv::COLOR_RGB2BGR);
        }
        else if (msg->encoding == "bgr8")
        {
            frame = cv::Mat(msg->height, msg->width, CV_8UC3, (void *)msg->data.data()).clone();
        }
        else
        {
            return;
        }

        if (frame.empty())
            return;

        cv::Mat birdeye, birdeye_with_lines, total_birdeye;
        cv::Mat red_mask, green_mask, white_mask, yellow_mask;
        int red_pixel_count = 0;
        int green_pixel_count = 0;

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            latest_frame = frame.clone();

            cv::Point2f src_pts[4] = {
                cv::Point2f(distort_L_top_x, distort_L_top_y),
                cv::Point2f(distort_R_top_x, distort_R_top_y),
                cv::Point2f(distort_L_under_x, distort_L_under_y),
                cv::Point2f(distort_R_under_x, distort_R_under_y)
            };
            cv::Point2f dst_pts[4] = {
                cv::Point2f(flat_L_top_x, flat_L_top_y),
                cv::Point2f(flat_R_top_x, flat_R_top_y),
                cv::Point2f(flat_L_under_x, flat_L_under_y),
                cv::Point2f(flat_R_under_x, flat_R_under_y)
            };

            cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
            cv::warpPerspective(frame, birdeye, M, cv::Size(640, 360));

            cv::Mat birdeye_blurred, birdeye_hsv, frame_hsv;
            cv::GaussianBlur(birdeye, birdeye_blurred, cv::Size(5, 5), 1.5);
            cv::cvtColor(birdeye_blurred, birdeye_hsv, cv::COLOR_BGR2HSV);
            cv::cvtColor(latest_frame, frame_hsv, cv::COLOR_BGR2HSV);

            cv::inRange(birdeye_hsv, lower_white, upper_white, white_mask);
            cv::inRange(birdeye_hsv, lower_yellow, upper_yellow, yellow_mask);
            cv::inRange(latest_frame, lower_red, upper_red, red_mask);
            cv::inRange(latest_frame, lower_green, upper_green, green_mask);

            cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::dilate(yellow_mask, yellow_mask, k, cv::Point(-1, -1), 5);
            cv::dilate(white_mask, white_mask, k, cv::Point(-1, -1), 5);
            cv::dilate(red_mask, red_mask, k, cv::Point(-1, -1), 5);
            cv::dilate(green_mask, green_mask, k, cv::Point(-1, -1), 5);
            
            cv::erode(yellow_mask, yellow_mask, k, cv::Point(-1, -1), 10);
            cv::erode(white_mask, white_mask, k, cv::Point(-1, -1), 10);
            cv::erode(red_mask, red_mask, k, cv::Point(-1, -1), 10);
            cv::erode(green_mask, green_mask, k, cv::Point(-1, -1), 10);

            // 신호등 인식
            for (int i = detect_y_start; i < detect_y_end; i++) {
                for (int j = detect_x_start; j < detect_x_end; j++)
                {
                    if (red_mask.at<uchar>(i, j) > 0)
                        red_pixel_count++;
                    if (green_mask.at<uchar>(i, j) > 0)
                        green_pixel_count++;
                }
            }

            if (red_pixel_count > red_threshold) traffic_light_state = 1;
            else if (green_pixel_count > green_threshold) traffic_light_state = 2;
            else traffic_light_state = 0;

            // 차선 인식 및 그리기
            birdeye_with_lines = birdeye.clone();
            int scan_y = 295; 
            
            // 라인 스캔 로직
            yellow_x = -1; white_x = -1;
            
            // Yellow
            for (int j = 0; j < yellow_mask.cols; j++) {
                if (yellow_mask.at<uchar>(scan_y, j) > 0)
                {
                    yellow_x = j;
                    cv::line(birdeye_with_lines, cv::Point(yellow_x, scan_y), cv::Point(yellow_x, scan_y), cv::Scalar(255, 255, 0), 1);
                    break;
                }
            }
            // White
            for (int j = white_mask.cols - 1; j >= 0; j--) {
                if (white_mask.at<uchar>(scan_y, j) > 0)
                {
                    white_x = j;
                    cv::line(birdeye_with_lines, cv::Point(white_x, scan_y), cv::Point(white_x, scan_y), cv::Scalar(255, 255, 255), 1);
                    break;
                }
            }

            // 중앙값 계산
            int center = -1;
            if (yellow_x != -1 && white_x != -1) center = (yellow_x + white_x) / 2;
            else if (yellow_x != -1) center = yellow_x + 230;
            else if (white_x != -1) center = white_x - 230;

            // 결과 업데이트 및 그리기
            if (center != -1) {
                cv::line(birdeye_with_lines, cv::Point(center, scan_y), cv::Point(center, scan_y), cv::Scalar(0, 255, 0), 1);

                global_center_x = center;
                global_yellow_x = yellow_x;
                global_white_x = white_x;
                global_yellow_diff = (yellow_x != -1) ? yellow_x - 320 : 0;
                global_white_diff = (white_x != -1) ? white_x - 320 : 0;
            }

            // 스캔 라인 표시
            cv::line(birdeye_with_lines, cv::Point(0, scan_y), cv::Point(640, scan_y), cv::Scalar(255, 0, 255), 1);

            // 텍스트 정보 표시
            if (global_center_x > 0) {
                cv::putText(birdeye_with_lines, "C:" + std::to_string(global_center_x),
                            cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }

            // GUI용 변수 업데이트
            bird_copy = birdeye_with_lines.clone();
            total_birdeye = yellow_mask + white_mask;
            red_and_green_mask = red_mask + green_mask;
        }

        // ID 0: 원본 프레임
        send_udp_image(frame, 0, this->get_logger());
        // ID 1: 차선이 그려진 Bird-eye View
        send_udp_image(birdeye_with_lines, 1, this->get_logger());
        // ID 2: 마스크 합본
        cv::Mat total_color;
        cv::cvtColor(total_birdeye, total_color, cv::COLOR_GRAY2BGR);
        send_udp_image(total_color, 2, this->get_logger());

        // 좌표값 Publish
        auto msg_data = std::make_unique<autorace_interfaces::msg::VisionHyun>();
        msg_data->header = msg->header;
        msg_data->center_x = global_center_x;
        msg_data->yellow_x = yellow_x;
        msg_data->white_x = white_x;
        msg_data->yellow_diff = global_yellow_diff;
        msg_data->white_diff = global_white_diff;
        msg_data->traffic_light = traffic_light_state;

        publisher_4->publish(*msg_data);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // UDP 소켓 생성
    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        std::cerr << "소켓 생성 실패" << std::endl;
        return 1;
    }

    // 원격 주소 설정
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(REMOTE_PORT);
    if (inet_pton(AF_INET, REMOTE_IP, &remote_addr.sin_addr) <= 0) {
        std::cerr << "주소 오류" << std::endl;
        close(udp_sock);
        return 1;
    }

    if (getenv("DISPLAY") == nullptr)
    {
        std::cerr << "DISPLAY 환경 X" << std::endl;
        close(udp_sock);
        return 1;
    }

    auto node = std::make_shared<ImageViewer>();
    std::thread t(gui_thread);
    t.detach();
    rclcpp::spin(node);
    rclcpp::shutdown();
    close(udp_sock);
    cv::destroyAllWindows();
    return 0;
}
