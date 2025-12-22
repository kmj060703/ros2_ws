#include "../include/vision_hyun/vision_hyun.hpp"
#include "autorace_interfaces/msg/vision_hyun.hpp"

#include <thread>
#include <mutex>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <math.h>

#define REMOTE_IP "192.168.0.38"
#define REMOTE_PORT 9999 // UI용 포트 하나만 사용
#define PACKET_SIZE 4096

int udp_sock = -1;
struct sockaddr_in remote_addr;

// mutex 기반 글로벌 변수 :전역 변수를 여러 스레드가 동시에 접근하니까 충돌이나 데이터 깨짐을 막기 위해 보호해서 쓴다고 함
std::mutex frame_mutex;
cv::Mat latest_frame;
cv::Mat latest_birdeye;
cv::Mat yellow_mask;
cv::Mat white_mask;
cv::Mat red_mask;
cv::Mat green_mask;
cv::Mat red_and_green_mask;
cv::Mat brown_mask, brown_mask_2;
cv::Mat bar_red_mask, bar_red_mask2;
cv::Mat frame_copy, bird_copy, yellow_mask_copy, white_mask_copy, red_and_green_mask_copy;
cv::Mat bar_temp_frame1, bar_temp_frame2;
cv::Mat start_red_mask;
cv::Mat start_red_mask;
// 전역 변수 추가
int detect_line = 350;
int global_center_x = -1;
int global_yellow_x = -1;
int global_white_x = -1;
int yellow_x = -1;
int white_x = -1;

int center_y = -1;
int center_w = -1;
int center_yw = -1;
// 신호등 감지 여부
int traffic_light_state = 0;
int brown_pixel_count = 0;

// red && green 탐지 범위 // 신호등 탐지 범위
int detect_x_start = 320;
int detect_x_end = 640;
int detect_y_start = 0;
int detect_y_end = 150;
int detect_y_end = 150;

// 장애물용 line 탐지 범위
int line_d_top = 250;
int line_d_bottom = 110;

bool start_line = false;
bool detect_red_light = false;
int start_red_threshold = 3000;

void send_udp_image(cv::Mat &img, int id)
{
    if (img.empty() || udp_sock < 0)
    {
        return;
    }

    std::vector<uchar> encoded;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50}; // 속도를 위해 50
    bool success = cv::imencode(".jpg", img, encoded, params);

    if (!success)
        return;

    int total_size = encoded.size();

    if (total_size == 0)
        return;

    int header[2] = {id, total_size};
    sendto(udp_sock, header, sizeof(header), 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));

    int sent_bytes = 0;
    while (sent_bytes < total_size)
    {
        int chunk_size = std::min(PACKET_SIZE, total_size - sent_bytes);
        ssize_t sent_chunk_bytes = sendto(udp_sock, &encoded[sent_bytes], chunk_size, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));

        if (sent_chunk_bytes < 0)
            break;

        sent_bytes += sent_chunk_bytes;

        // 패킷 손실 방지를 위한 딜레이
        usleep(500);
    }
}

void gui_thread()
{
    while (rclcpp::ok())
    {
        cv::Mat frame_display, bird_display, yellow_display, white_display, red_and_green_mask_copy, brown_copy;

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!latest_frame.empty())
                frame_display = latest_frame.clone();
            if (!bird_copy.empty())
                bird_display = bird_copy.clone();
            // if (!yellow_mask.empty())
            //     yellow_display = yellow_mask.clone();
            // if (!white_mask.empty())
            //     white_display = white_mask.clone();
            if (!red_and_green_mask.empty())
                red_and_green_mask_copy = red_and_green_mask.clone();
            if (!brown_mask.empty())
                brown_copy = brown_mask.clone();
        }

        if (!frame_display.empty())
        {
            std::stringstream ss;
            ss << "Traffic Light flag : " << traffic_light_state;

            cv::line(frame_display, cv::Point(320, 0), cv::Point(320, 360), cv::Scalar(0, 0, 255), 1);
            cv::line(frame_display, cv::Point(0, 80), cv::Point(640, 80), cv::Scalar(0, 0, 255), 1);
            cv::rectangle(
                frame_display,
                cv::Point(detect_x_start, detect_y_start),
                cv::Point(detect_x_end, detect_y_end),
                cv::Scalar(0, 255, 0), 2);

            cv::putText(
                frame_display,
                ss.str(),
                cv::Point(detect_x_start, detect_y_start - 5),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

            cv::imshow("Spedal Feed", frame_display);
        }

        if (!bird_display.empty())
            cv::imshow("Bird's-Eye View", bird_display);

        if (!red_and_green_mask.empty())
            cv::imshow("traffic_mask", red_and_green_mask);
        if (!brown_mask.empty())
            cv::imshow("brown_mask", brown_mask);

        cv::waitKey(1);
    }
}

ImageViewer::ImageViewer()
    : Node("vision_hyun_node")
{
    auto qos_profile = rclcpp::SensorDataQoS();
    const std::string image_topic = "/default_camera/image_raw";

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, qos_profile,
        std::bind(&ImageViewer::image_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/image_processed", qos_profile);
    publisher_2 = this->create_publisher<sensor_msgs::msg::Image>("/vision/birdeye_raw", qos_profile);
    publisher_3 = this->create_publisher<sensor_msgs::msg::Image>("/vision/birdeye_total", qos_profile);
    publisher_4 = this->create_publisher<autorace_interfaces::msg::VisionHyun>("/vision/line_diff_info", qos_profile);

    // RCLCPP_INFO(this->get_logger(), "Image viewer node started.");

    cv::namedWindow("Spedal Feed");
    // cv::namedWindow("Bird-Eye View");
    // cv::namedWindow("yellow_mask");
    // cv::namedWindow("white_mask");
    cv::namedWindow("traffic_mask");
    cv::namedWindow("brown_mask");
}

void ImageViewer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat frame;

        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (frame.empty())
            return;

        cv::Mat birdeye, birdeye_with_lines, total_birdeye;

        int red_pixel_count = 0;
        int green_pixel_count = 0;

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            latest_frame = frame.clone();
            // 버드아이 변환
            cv::Point2f src_pts[4] = {
                cv::Point2f(distort_L_top_x, distort_L_top_y) /*왼쪽 위*/, cv::Point2f(distort_R_top_x, distort_R_top_y) /*오른쪽 위*/,
                cv::Point2f(distort_L_under_x, distort_L_under_y) /*왼쪽 아래*/, cv::Point2f(distort_R_under_x, distort_R_under_y) /*오른쪽 아래*/};
            cv::Point2f dst_pts[4] = {
                cv::Point2f(flat_L_top_x, flat_L_top_y), cv::Point2f(flat_R_top_x, flat_R_top_y),
                cv::Point2f(flat_L_under_x, flat_L_under_y), cv::Point2f(flat_R_under_x, flat_R_under_y)}; // Z 모양 순서

            cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
            cv::warpPerspective(frame, birdeye, M, cv::Size(640, 360));

            // 버드아이 > 가우시안 블러 적용
            cv::Mat birdeye_blurred, birdeye_hsv, frame_hsv;
            cv::GaussianBlur(birdeye, birdeye_blurred, cv::Size(5, 5), 20);

            // hsv 변환
            cv::cvtColor(birdeye_blurred, birdeye_hsv, cv::COLOR_BGR2HSV);
            cv::cvtColor(latest_frame, frame_hsv, cv::COLOR_BGR2HSV);

            cv::inRange(birdeye_hsv, lower_brown, upper_brown, brown_mask);
            cv::inRange(birdeye_hsv, lower_brown_2, upper_brown_2, brown_mask_2);
            cv::add(brown_mask, brown_mask_2, brown_mask);

            cv::inRange(birdeye_hsv, lower_white, upper_white, white_mask);
            cv::inRange(birdeye_hsv, lower_yellow, upper_yellow, yellow_mask);


            cv::inRange(latest_frame, lower_green, upper_green, green_mask);
            cv::inRange(frame_hsv, bar_lower_red, bar_upper_red, bar_temp_frame1);
            cv::inRange(frame_hsv, bar_lower_red_2, bar_upper_red_2, bar_temp_frame2);
            
            // 침식/팽창
            cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::dilate(yellow_mask, yellow_mask, k, cv::Point(-1, -1), 5);
            cv::dilate(white_mask, white_mask, k, cv::Point(-1, -1), 5);
            cv::dilate(brown_mask, brown_mask, k, cv::Point(-1, -1), 5);

            // 신호등 감지 범위 및 플래그

            /*
            channels[0]  Blue 채널

            channels[1]  Green 채널

            channels[2]  Red 채널
            */

            // 아니 이거 신호등 불빛이 너무 강해서 hsv로 못따옴 그래서 bgr로 받아옴 에라이 시간만 날렸내
            std::vector<cv::Mat> channels;
            cv::split(latest_frame, channels); // BGR 형식으로 분리할 때 cv::split()

            cv::Mat red_mask = (channels[2] > channels[0] + 30) &
                               (channels[2] > channels[1] + 30) &
                               (channels[2] > 150);

            // 노이즈 제거
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);

            cv::Mat green_temp_mask = (channels[1] > channels[0] + 30) &
                                      (channels[1] > channels[2] + 30) &
                                      (channels[1] > 100);

            green_temp_mask.convertTo(green_mask, CV_8U, 255);
            green_mask = green_temp_mask.clone();

            cv::split(birdeye_blurred, channels);
            cv::Mat start_red_mask = (channels[2] > channels[0] + 30) &
                                     (channels[2] > channels[1] + 30) &
                                     (channels[2] > 150);
            cv::morphologyEx(start_red_mask, start_red_mask, cv::MORPH_OPEN, kernel);

            cv::split(birdeye_blurred, channels);
            cv::Mat start_red_mask = (channels[2] > channels[0] + 30) &
                                     (channels[2] > channels[1] + 30) &
                                     (channels[2] > 150);
            cv::morphologyEx(start_red_mask, start_red_mask, cv::MORPH_OPEN, kernel);

            bar_red_pixel_count = 0;
            red_pixel_count = 0;
            yellow_pixel_count = 0;
            green_pixel_count = 0;
            red_and_green_mask = red_mask + green_mask;

            detect_x_start = 320;
            detect_x_end = 640;
            detect_y_start = 0;
            detect_y_end = 250;

            if (traffic_light_state != 1)
            {
                for (int i = 180; i < 360; i++)
                {
                    for (int j = 0; j < 640; j++)
                    {
                        if (start_red_mask.at<uchar>(i, j) > 0)
                            red_pixel_count++;
                    }
                }
                if (red_pixel_count > start_red_threshold) // 3000 픽셀 이상
                {
                    start_line = true;
                }
                else
                    start_line = false;
            }
            if (traffic_light_state)
            {

                for (int i = detect_y_start; i < detect_y_end; i++)
                {
                    for (int j = detect_x_start; j < detect_x_end; j++)
                    {
                        if (red_mask.at<uchar>(i, j) > 0)
                            red_pixel_count++;
                        if (green_mask.at<uchar>(i, j) > 0)
                            green_pixel_count++;
                    }
                }
                if (red_pixel_count > red_threshold)
                {
                    detect_red_light = true;
                }
            }
            // std::cerr << red_pixel_count << std::endl;
            cv::imshow("red_traffic", red_mask);
            detect_x_start = 0;
            detect_y_start = 90;
            detect_y_end = 360;

            cv::Mat red_bar_frame = bar_temp_frame1 + bar_temp_frame2;

            for (int i = detect_y_start; i < detect_y_end; i++)
            {
                for (int j = detect_x_start; j < detect_x_end; j++)
                {
                    if (red_bar_frame.at<uchar>(i, j) > 0)
                        bar_red_pixel_count++;
                }
            }

            if (green_pixel_count > green_threshold && detect_red_light) // 300 픽셀 이상
            if (green_pixel_count > green_threshold && detect_red_light) // 300 픽셀 이상
            {
                traffic_light_state = 2;
            }
            else if (start_line) // 150 픽셀 이상
            else if (start_line) // 150 픽셀 이상
            {
                traffic_light_state = 1;
            }
            else if (bar_red_pixel_count > bar_red_red_threshold)
            {
                traffic_light_state = 4;
            }
            else
            {
                traffic_light_state = 0;
            }
            //  std::cerr << traffic_light_state << std::endl;
            //  갈색
            brown_pixel_count = 0;
            yellowline_pixel_count_low = 0;
            yellowline_pixel_count_top = 0;
            whiteline_pixel_count_low = 0;
            whiteline_pixel_count_top = 0;
            for (int i = 0; i < brown_mask.cols; i++)
            {
                for (int j = 0; j < brown_mask.rows; j++)
                {

                    if (brown_mask.at<uchar>(j, i) > 0)
                    {
                        brown_pixel_count++;
                        cv::line(birdeye_with_lines, cv::Point(i, j), cv::Point(i, j), cv::Scalar(255, 125, 125), 1);
                    }
                    if (j > line_d_top)
                    {
                        if (yellow_mask.at<uchar>(j, i) > 0)
                        {
                            yellowline_pixel_count_low++;
                        }
                        if (white_mask.at<uchar>(j, i) > 0)
                        {
                            whiteline_pixel_count_low++;
                        }
                    }
                    if (j < line_d_bottom)
                    {
                        if (yellow_mask.at<uchar>(j, i) > 0)
                        {
                            yellowline_pixel_count_top++;
                        }
                        if (white_mask.at<uchar>(j, i) > 0)
                        {
                            whiteline_pixel_count_top++;
                        }
                    }
                }
            }


            // 버드아이 중앙선 추출
            birdeye_with_lines = birdeye.clone();
            global_center_x = -1;
            global_yellow_x = -1;
            global_white_x = -1;
            for (int i = 0; i < yellow_mask.rows; i++)
            {
                // 초기화
                yellow_x = -1;
                white_x = -1;

                for (int j = 0; j < yellow_mask.cols; j++)
                {
                    if (yellow_mask.at<uchar>(i, j) > 0)
                    {
                        yellow_x = j;
                        cv::line(birdeye_with_lines, cv::Point(yellow_x, i), cv::Point(yellow_x, i), cv::Scalar(255, 255, 0), 1);
                        break;
                    }
                }

                for (int j = white_mask.cols - 1; j >= 0; j--)
                {
                    if (white_mask.at<uchar>(i, j) > 0)
                    {
                        white_x = j;
                        cv::line(birdeye_with_lines, cv::Point(white_x, i), cv::Point(white_x, i), cv::Scalar(255, 255, 255), 1);
                        break;
                    }
                }
                if (yellow_x > white_x && white_x != -1)
                {
                    yellow_x = -1;
                    white_x = -1;
                }
                center_y = -1;
                center_w = -1;
                center_yw = -1;
                if (yellow_x != -1 && white_x != -1)
                    center_yw = (yellow_x + white_x) / 2;
                if (yellow_x != -1)
                    center_y = yellow_x + 235;
                if (white_x != -1)
                    center_w = white_x - 235;

                if (!(center_w == center_yw && center_y == center_yw && center_yw == -1))
                {
                    cv::line(birdeye_with_lines, cv::Point(center_yw, i), cv::Point(center_yw, i), cv::Scalar(0, 255, 0), 1);
                    cv::line(birdeye_with_lines, cv::Point(center_w, i), cv::Point(center_w, i), cv::Scalar(255, 255, 255), 1);
                    cv::line(birdeye_with_lines, cv::Point(center_y, i), cv::Point(center_y, i), cv::Scalar(255, 255, 0), 1);
                    // 제어 기준선 (y=350)
                    if (i == detect_line)
                    {
                        global_center_x = center_yw;
                        global_yellow_x = center_y;
                        global_white_x = center_w;
                    }
                }
            }

            cv::line(birdeye_with_lines, cv::Point(birdeye_with_lines.cols * 0.75, 0), cv::Point(birdeye_with_lines.cols * 0.75, birdeye_with_lines.rows), cv::Scalar(0, 0, 255), 1);
            cv::line(birdeye_with_lines, cv::Point(birdeye_with_lines.cols * 0.25, 0), cv::Point(birdeye_with_lines.cols * 0.25, birdeye_with_lines.rows), cv::Scalar(0, 0, 255), 1);

            cv::line(birdeye_with_lines, cv::Point(0, detect_line), cv::Point(640, detect_line), cv::Scalar(255, 0, 255), 3);

            if (global_center_x > 0)
            {
                cv::putText(birdeye_with_lines, "C:" + std::to_string(global_center_x),
                            cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                cv::circle(birdeye_with_lines, cv::Point(global_center_x, detect_line), 5, cv::Scalar(0, 0, 255), -1);
            }

            bird_copy = birdeye_with_lines.clone();
            total_birdeye = yellow_mask + white_mask;
        }

        send_udp_image(frame, 0);
        send_udp_image(birdeye_with_lines, 1);

        cv::Mat total_color;
        cv::cvtColor(total_birdeye, total_color, cv::COLOR_GRAY2BGR);
        send_udp_image(total_color, 2);

        auto msg_data = std::make_unique<autorace_interfaces::msg::VisionHyun>();
        msg_data->header = msg->header;
        /*참고용 header 안에는
        std_msgs/Header header
        {
        uint32 seq;         // 시퀀스 번호
        time stamp;         // 메시지가 생성된 시간
        string frame_id;    // TF 프레임 이름
                }

        이런게 있다*/

        msg_data->center_x = global_center_x;
        msg_data->yellow_x = global_yellow_x;
        msg_data->white_x = global_white_x;
        msg_data->traffic_light = traffic_light_state;
        msg_data->brown_count = brown_pixel_count;
        msg_data->yellowline_count_low = yellowline_pixel_count_low;
        msg_data->whiteline_count_low = whiteline_pixel_count_low;
        msg_data->yellowline_count_top = yellowline_pixel_count_top;
        msg_data->whiteline_count_top = whiteline_pixel_count_top;

        publisher_4->publish(*msg_data);
        sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        publisher_->publish(*processed_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0)
    {
        std::cerr << "소켓 생성 실패" << std::endl;
        return 1;
    }

    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(REMOTE_PORT);
    if (inet_pton(AF_INET, REMOTE_IP, &remote_addr.sin_addr) <= 0)
    {
        std::cerr << "UI 주소 오류" << std::endl;
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