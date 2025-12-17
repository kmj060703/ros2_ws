#ifndef CONTROLITEM_HPP
#define CONTROLITEM_HPP

#include "autorace_interfaces/msg/ui2_driving.hpp"
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>

extern int l_state_flag_;
extern int state_flag_;
extern int imshow_flag_1;
extern int imshow_flag_2;
extern int start_flag_;
extern int forw_back_;
extern int left_right_;
extern int l_start_flag_;
extern int camera_1_state;
extern int camera_2_state;
extern int vision_hsv_state;
extern int HSV_high[21];
extern int HSV_low[21];
extern int driving_state;
extern int currentIndex;
extern int img_id;
extern int psd_flag[3];
extern int traffic_state;
extern int brown_count;
extern int yellow_count_low;
extern int white_count_low;
extern int yellow_count_top;
extern int white_count_top;
extern int udp_crack;
extern double imu_yaw;
extern double imu_yaw_local;
extern double x_;
extern double z_;
extern double kp_;
extern double kd_;
extern double l_x_;
extern double l_z_;
extern double max_vel_;
extern cv::Mat line_white_mask;
extern cv::Mat line_yellow_mask;
extern cv::Mat line_red_mask;
extern cv::Mat tra_red_mask;
extern cv::Mat tra_yellow_mask;
extern cv::Mat tra_green_mask;
extern cv::Mat brown_mask;
extern cv::Mat frame;
extern cv::Mat birdeye_hsv;
extern cv::Mat frame_hsv;
extern cv::Scalar lower_l_white;
extern cv::Scalar upper_l_white;
extern cv::Scalar lower_l_yellow;
extern cv::Scalar upper_l_yellow;
extern cv::Scalar lower_l_red;
extern cv::Scalar upper_l_red;
extern cv::Scalar lower_t_red;
extern cv::Scalar upper_t_red;
extern cv::Scalar lower_t_yellow;
extern cv::Scalar upper_t_yellow;
extern cv::Scalar lower_t_green;
extern cv::Scalar upper_t_green;
extern cv::Scalar lower_brown;
extern cv::Scalar upper_brown;
extern cv::Mat red_l_mask;
#endif // CONTROLITEM_HPP