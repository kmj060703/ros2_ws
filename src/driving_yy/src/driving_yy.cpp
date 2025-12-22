#include "../include/driving_yy/driving_yy.hpp"

using std::placeholders::_1;

DrivingYY::DrivingYY() : Node("driving_yy")
{
    kp = 0.00;
    kd = 0.00;
    x = 0.0;
    z = 0.0;
    error = 0.0;
    error_yw = 0.0;
    error_w = 0.0;
    error_y = 0.0;
    last_error = 0.0;
    max_x = 0.0;
    def_turn_x = 0.0;
    def_turn_z = 0.0;

    auto sensor_qos = rclcpp::SensorDataQoS();
    imu_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "imu_angle",
        sensor_qos,
        std::bind(&DrivingYY::imu_callback, this, _1));

    psd_front_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "psd_result_front", sensor_qos, std::bind(&DrivingYY::psd_front_callback, this, _1));

    psd_left_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "psd_result_left", sensor_qos, std::bind(&DrivingYY::psd_left_callback, this, _1));

    psd_right_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "psd_result_right", sensor_qos, std::bind(&DrivingYY::psd_right_callback, this, _1));

    flag_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "master_jo_flag",
        sensor_qos,
        std::bind(&DrivingYY::flag_callback, this, _1));
    ui_sub_ = this->create_subscription<autorace_interfaces::msg::Ui2Driving>(
        "/ui2driving_topic",
        sensor_qos,
        std::bind(&DrivingYY::ui_callback, this, _1));

    pixel_diff_sub_ = this->create_subscription<autorace_interfaces::msg::MasterJo>(
        "pixel_diff",
        sensor_qos,
        std::bind(&DrivingYY::pixel_diff_callback, this, std::placeholders::_1));

    vision_traffic_sub_ = this->create_subscription<autorace_interfaces::msg::VisionHyun>(
        "/vision/line_diff_info",
        sensor_qos,
        std::bind(&DrivingYY::vision_traffic_callback, this, _1));

    publisher_drive = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", sensor_qos);
    drive_timer = this->create_wall_timer(100ms, std::bind(&DrivingYY::drive_callback, this));
    park_timer = this->create_wall_timer(100ms, std::bind(&DrivingYY::park_callback, this));

    RCLCPP_INFO(this->get_logger(), "DrivingYY Start");
    // RCLCPP_INFO(this->get_logger(), "초기 미션 플래그: %d", mission_flag_);
}
void DrivingYY::imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    current_yaw_ = msg->z;
    // RCLCPP_INFO(this->get_logger(), "Yaw: %.2f 도", current_yaw_ * 180.0 / M_PI);
}
void DrivingYY::psd_front_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    is_front_danger_ = msg->data;
    // RCLCPP_WARN(this->get_logger(), "정면 장애물 감지 여부: %d", is_front_danger_);
}
void DrivingYY::psd_left_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    is_left_danger_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "왼쪽 장애물 감지 여부: %d", is_left_danger_);
}
void DrivingYY::psd_right_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    is_right_danger_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "오른쪽 장애물 감지 여부: %d", is_right_danger_);
}
void DrivingYY::flag_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    mission_flag_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "플래그: %d", mission_flag_);
}
void DrivingYY::ui_callback(const autorace_interfaces::msg::Ui2Driving::SharedPtr msg)
{
    // 유아이
    vision_valid_ = msg->vision_lost_flag;
    max_x = msg->max_vel;
    l_start_flag = msg->l_start_flag;
    start_flag = msg->start_flag;
    kp = msg->kp;
    kd = msg->kd;
    x = msg->l_x;
    z = msg->l_z;
    def_turn_x = msg->def_turn_x;
    def_turn_z = msg->def_turn_z;
}
void DrivingYY::pixel_diff_callback(const autorace_interfaces::msg::MasterJo::SharedPtr msg)
{
    error_yw = msg->pixel_diff;
    error_y = msg->yellow_diff;
    error_w = msg->white_diff;

    // RCLCPP_INFO(this->get_logger(), "Pixel Diff: %f", error);
}
void DrivingYY::park_callback()
{
    park_time++;
}
void DrivingYY::vision_traffic_callback(const autorace_interfaces::msg::VisionHyun::SharedPtr msg)
{
    traffic_light_status_ = msg->traffic_light;
    brown_count = msg->brown_count;
    yellow_count_low = msg->yellowline_count_low;
    white_count_low = msg->whiteline_count_low;
    yellow_count_top = msg->yellowline_count_top;
    white_count_top = msg->whiteline_count_top;

    if (traffic_light_status_ == 1)
    {
        // RCLCPP_INFO(this->get_logger(), "빨간불 감지 %d", traffic_light_status_);
    }
    else if (traffic_light_status_ == 2)
    {
        // RCLCPP_INFO(this->get_logger(), "초록불 감지 %d", traffic_light_status_);
    }
    else
    {
        // RCLCPP_INFO(this->get_logger(), "감지 안됨 혹은 노란색 %d", traffic_light_status_);
    }
}
void DrivingYY::PD_control()
{
    // std::cout<<"오리지널 pd중"<<std::endl;
    if (error_yw != -321)
    {
        error = error_yw;
    }
    else if (error_y != -321)
    {
        error = error_y;
    }
    else if (error_w != -321)
    {
        error = error_w;
    }
    if (park_comp == 0 && mission_flag_ == 5)
    {
        error = error_y;
    }

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    //                      "[DEBUG] flags: start_flag=%d, l_start_flag=%d, error=%.2f",
    //                      start_flag, l_start_flag, error);

    z = kp * error + kd * (error - last_error);

    // 디버깅 출력 추가
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //                      "error=%.2f, last_error=%.2f, kp=%.2f, kd=%.2f, z=%.4f",
    //                      error, last_error, kp, kd, z);

    last_error = error;

    // x 계산 수정
    double ratio = std::max(1.0 - std::abs(error) / 360.0, 0.0);
    double x_raw = std::pow(max_x * ratio, 2.2);
    x = std::min(x_raw, 5.0);

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //     "ratio=%.2f, x_raw=%.4f, x=%.4f", ratio, x_raw, x);

    if (z < 0)
        z = -std::max(z, -def_turn_z);
    else
        z = -std::min(z, def_turn_z);
    //}

    driving_msg.linear.x = x;
    driving_msg.angular.z = z;

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //     "Publishing: x=%.4f, z=%.4f", driving_msg.linear.x, driving_msg.angular.z);
}

void DrivingYY::Fast_PD()
{
    // std::cout<<"오리지널 pd중"<<std::endl;
    if (error_yw != -321)
    {
        error = error_yw;
    }
    else if (error_y != -321)
    {
        error = error_y;
    }
    else if (error_w != -321)
    {
        error = error_w;
    }
    if (park_comp == 0 && mission_flag_ == 5)
    {
        error = error_y;
    }

    kp = 0.01;
    kd = 0.005;
    max_x = 0.6;
    def_turn_z = 0.8;
    z = kp * error + kd * (error - last_error);

    last_error = error;
    // x 계산 수정
    double ratio = std::max(1.0 - std::abs(error) / 360.0, 0.0);
    double x_raw = std::pow(max_x * ratio, 2.2);
    x = std::min(x_raw, 5.0);

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //     "ratio=%.2f, x_raw=%.4f, x=%.4f", ratio, x_raw, x);

    if (z < 0)
        z = -std::max(z, -def_turn_z);
    else
        z = -std::min(z, def_turn_z);

    driving_msg.linear.x = x;
    driving_msg.angular.z = z;

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //     "Publishing: x=%.4f, z=%.4f", driving_msg.linear.x, driving_msg.angular.z);
}

void DrivingYY::Traffic_light()
{
    if (traffic_mission_comp == 0)
    {
        if (mission_flag_ == 0)
        {

            if (traffic_light_status_ != 0)
            {
                switch (traffic_light_status_)
                {
                case 1:
                {
                    traffic_red = 1;
                    driving_msg.linear.x = 0;
                    driving_msg.angular.z = 0;
                }
                break;

                default:
                {
                    if (traffic_red == 1 && traffic_mission_comp == 0)
                        traffic_mission_comp = 1;
                } // 노란색 감지를 위해 비워 놓는 것이다...
                break;
                }
            }
        }
    }
}
void DrivingYY::Itersection()
{

    if (mission_flag_ == 2)
    { // 좌
        if (error_y > 290)
        {
            error_y = -321;
        }
        if ((error_y == -321 && error_w == -321) || (error_y == -321 && z <= 0.25))
        {
            driving_msg.linear.x = 0.2;
            driving_msg.angular.z = 0.7;
        }
    }
    else if (mission_flag_ == 3)
    { // 우
        if (error_w < -290)
        {
            error_w = -321;
        }
        if ((error_y == -321 && error_w == -321) || (error_w == -321 && z >= -0.25))
        {
            driving_msg.linear.x = 0.2;
            driving_msg.angular.z = -0.7;
        }
    }
}
double degreecal(double degree)
{
    while (degree >= 190 || degree < -190)
    {
        if (degree > 190)
        {
            degree -= 380;
        }
        else if (degree <= -190)
        {
            degree += 380;
        }
    }
    return degree;
}
void DrivingYY::Construction()
{

    if (mission_flag_ == 4)
    {
        RCLCPP_INFO(this->get_logger(), "degree_now: %f", current_yaw_);
        RCLCPP_INFO(this->get_logger(), "degree_goal: %f", local_yaw);
        RCLCPP_INFO(this->get_logger(), "degree_diff: %f", local_diff);
        if (timer < count)
        {
            if ((current_yaw_ - local_yaw) < 5 && (current_yaw_ - local_yaw) > -5)
            {
                timer++;
            }
            else
            {
                timer = 0;
                local_yaw = current_yaw_;
            }
        }
        if (timer == count)
        {
            local_diff = degreecal(local_yaw - current_yaw_ - 93);
        }
        if (state == 0)
        {
            if (white_count_low >= 3000)
            {
                Construction_mem = 1;
            }
            else if (yellow_count_low >= 3000)
            {
                Construction_mem = 0;
            }
            RCLCPP_INFO(this->get_logger(), "s0, Construction_mem:%d, ", Construction_mem);
            if (brown_count > 42000 || is_front_danger_ >= 720)
            {
                state = 1;
                ;
            }
        }

        else if (state == 1)
        {
            RCLCPP_INFO(this->get_logger(), "s2, Construction_mem:%d", Construction_mem);
            if (Construction_mem == 1)
            {
                if (local_diff < 90 && local_diff > 78)
                {
                    driving_msg.linear.x = 0.12;
                    driving_msg.angular.z = 0.0;
                    RCLCPP_INFO(this->get_logger(), "s3, yellow: %d", yellow_count_low);
                    if (yellow_count_low >= 4000)
                    {
                        Construction_mem = 0;
                        state = 2;
                    }
                }
                else if (local_diff > 85)
                {
                    driving_msg.linear.x = 0.0;
                    driving_msg.angular.z = -0.2 + ((85 - local_diff) / 70);
                }
                else if (local_diff < 85)
                {
                    driving_msg.linear.x = 0.0;
                    driving_msg.angular.z = 0.2 + ((85 - local_diff) / 70);
                }
            }
            else if (Construction_mem == 0)
            {
                if (local_diff > -90 && local_diff < -78)
                {
                    driving_msg.linear.x = 0.12;
                    driving_msg.angular.z = 0.0;
                    RCLCPP_INFO(this->get_logger(), "s4, white: %d", white_count_low);
                    if (white_count_low >= 4000)
                    {
                        Construction_mem = 1;
                        state = 3;
                    }
                }
                else if (local_diff > -85)
                {
                    driving_msg.linear.x = 0.00;
                    driving_msg.angular.z = -0.2 + ((-85 - local_diff) / 70);
                }
                else if (local_diff < -85)
                {
                    driving_msg.linear.x = 0.00;
                    driving_msg.angular.z = 0.2 + ((-85 - local_diff) / 70);
                }
            }
        }
        else if (state == 2)
        {
            if (local_diff < 37 && local_diff > 33)
            {
                state = 0;
            }
            else if (local_diff > 35)
            {
                driving_msg.linear.x = 0.0;
                driving_msg.angular.z = -0.2 + ((35 - local_diff) / 70);
            }
            else if (local_diff < 35)
            {
                driving_msg.linear.x = 0.0;
                driving_msg.angular.z = 0.2 + ((35 - local_diff) / 70);
            }
        }
        else if (state == 3)
        {
            if (local_diff > -37 && local_diff < -33)
            {
                state = 0;
            }
            else if (local_diff > -35)
            {
                driving_msg.linear.x = 0.00;
                driving_msg.angular.z = -0.2 + ((-35 - local_diff) / 70);
            }
            else if (local_diff < -35)
            {
                driving_msg.linear.x = 0.00;
                driving_msg.angular.z = 0.2 + ((-35 - local_diff) / 70);
            }
        }
    }
}
enum ParkingState
{
    PARK_PD,
    PARK_START,
    AVOID_RIGHT,
    AVOID_LEFT,
    GO_FRONT,
    GO_BACK,
    GO_OUT,
    GO_OUT2,
    PSD_AGAIN
};
ParkingState pstate_;

bool near(double a, double b, double eps = 2.0) { return fabs(a - b) < eps; }

void DrivingYY::Parking()
{
    if (mission_flag_ == 5)
    {
        // if (timer == count)
        // {
        //     local_diff = degreecal(local_yaw - current_yaw_ - 5);
        // }
        switch (pstate_)
        {
        case PARK_PD:
        {
            if (only_y == 0 && error_y != -321)
                PD_control();
            else
                only_y = 1;

            if (only_y == 1)
            {
                driving_msg.linear.x = 0.09;
                driving_msg.angular.z = 0.28;
                if (current_yaw_ < 111 && current_yaw_ > 0)
                {
                    only_y = 2;
                }
            }

            if (only_y == 2)
            {
                // if (time_flag == 0)
                // {
                //     last_time = park_time;
                // }
                // if(park_time-last_time<5){
                //     driving_msg.linear.x = 0.09;
                //     driving_msg.angular.z = 0.0;
                //     time_flag=1;
                // }
                // else time_flag=2;

                if (error_y < 310)
                {
                    std::cout << "이제 직진" << white_count_low << std::endl;
                    pstate_ = PARK_START;
                    only_y = 0;
                }
            }
        }
        break;

        case PARK_START:
        {
            if (error_y != -321 &&
                (std::abs(error_y - 0) > 7))
            {
                PD_control();
                std::cout << "지금은 PD제어 중333" << white_count_low << std::endl;
            }
            else
            {
                std::cout << "계속 직진" << white_count_low << std::endl;
                driving_msg.linear.x = 0.07;
                driving_msg.angular.z = 0.0;
            }
            if (near(current_yaw_, 100))
            {
                std::cout << "계속 직진" << white_count_low << std::endl;
                driving_msg.linear.x = 0.07;
                driving_msg.angular.z = 0.0;
            }
            // if(std::abs(error_y-0)<4)yaw_count++;
            // if(yaw_count>4)local_yaw=current_yaw_;
            if (white_count_top > 20000 && (is_left_danger_ > 400))
            {
                pstate_ = AVOID_RIGHT;
                std::cout << "오른쪽으로피하기 시작" << std::endl;
            }
            else if (white_count_top > 20000 && (is_right_danger_ > 400))
            {
                pstate_ = AVOID_LEFT;
                std::cout << "왼쪽으로피하기 시작" << std::endl;
            }
            if (white_count_top > 20000 && is_left_danger_ < 400 && is_right_danger_ < 400)
            {
                pstate_ = PSD_AGAIN;
            }
        }
        break;
        case PSD_AGAIN:
        {
            if (only_y == 1 && white_count_top > 0)
            {
                driving_msg.linear.x = 0.7;
                driving_msg.angular.z = 0.0;
            }
            else
                only_y = 1;

            if (only_y == 1)
            {
                if (white_count_top < 20000 || is_right_danger_ < 250 || is_left_danger_ < 250)
                {
                    driving_msg.linear.x = -0.7;
                    driving_msg.angular.z = 0.0;
                }
                else
                    only_y = 2;
            }
            if (only_y == 2)
            {
                if (is_right_danger_ > 400)
                {
                    pstate_ = AVOID_LEFT;
                    std::cout << "왼쪽으로피하기 시작" << std::endl;
                }
                else if (is_left_danger_ > 400)
                {
                    pstate_ = AVOID_RIGHT;
                    std::cout << "오른쪽으로피하기 시작" << std::endl;
                }
            }
        }
        break;
        case AVOID_RIGHT:
        {
            avoid_mem = -1;
            driving_msg.linear.x = -0.01;
            driving_msg.angular.z = 0.5;
            right_turn = 1;
            std::cout << "오른쪽으로피해" << std::endl;
            if (near(current_yaw_, 20))
                pstate_ = GO_FRONT;
        }
        break;
        case AVOID_LEFT:
        {
            avoid_mem = 1;
            driving_msg.linear.x = -0.01;
            driving_msg.angular.z = -0.5;
            left_turn = 1;
            std::cout << "왼쪽으로피해" << std::endl;
            if (near(current_yaw_, -185))
                pstate_ = GO_FRONT;
        }
        break;
        case GO_FRONT:
        {
            if (avoid_mem == -1)
            {
                if (time_flag == 0)
                {
                    last_time = park_time;
                    time_flag = 1;
                }
                std::cout << "pd안함 뒤로가기 경우1, " << is_front_danger_ << std::endl;
                driving_msg.linear.x = -0.07;
                driving_msg.angular.z = 0.0;
                if (park_time - last_time >= 35)
                {
                    last_time = park_time;
                    time_flag = 0;
                    pstate_ = GO_BACK;
                    for_count = 0;

                    pstate_ = GO_BACK;
                    for_count = 0;
                    return;
                }
            }
            else if (avoid_mem == 1) // 둘다 뒤로갔다 앞으로가기
            {

                if (time_flag == 0)
                {
                    last_time = park_time;
                    time_flag = 1;
                }
                std::cout << "pd안함 뒤로가기 경우1, " << is_front_danger_ << std::endl;
                driving_msg.linear.x = -0.07;
                driving_msg.angular.z = 0.0;
                if (park_time - last_time >= 35)
                {
                    last_time = park_time;
                    time_flag = 0;
                    pstate_ = GO_BACK;
                    for_count = 0;

                    pstate_ = GO_BACK;
                    for_count = 0;
                    return;
                }
            }
        }
        break;
        case GO_BACK: // 여기서 문제생기는듯
        {
            if (avoid_mem == -1)
            {
                if (time_flag == 0)
                {
                    last_time = park_time;
                    time_flag = 1;
                }
                if (turn_flag == 0 && park_time - last_time < 35)
                {
                    std::cout << "pd안함 오른쪽앞으로가기, " << park_time - last_time << std::endl;
                    driving_msg.linear.x = 0.07;
                    driving_msg.angular.z = 0.0;
                    return;
                }
                else if (turn_flag == 0 && park_time - last_time >= 35)
                {
                    turn_flag = 1;
                }
                if (turn_flag == 1)
                {
                    std::cout << "pd안함 오른쪽 돌고있다, " << is_front_danger_ << std::endl;
                    driving_msg.linear.x = 0.02;
                    driving_msg.angular.z = 0.5;
                }
                if (near(current_yaw_, -70))
                {
                    pstate_ = GO_OUT;
                    last_time = park_time;
                    time_flag = 0;
                }
            }
            else if (avoid_mem == 1)
            {

                if (time_flag == 0)
                {
                    last_time = park_time;
                    time_flag = 1;
                }
                if (turn_flag == 0 && park_time - last_time < 35)
                {
                    std::cout << "pd안함 왼쪽앞으로가기, " << park_time - last_time << std::endl;
                    driving_msg.linear.x = 0.07;
                    driving_msg.angular.z = 0.0;
                    return;
                }
                else if (turn_flag == 0 && park_time - last_time >= 35)
                {
                    turn_flag = 1;
                }
                if (turn_flag == 1)
                {
                    std::cout << "pd안함 왼쪽 돌고있다, " << is_front_danger_ << std::endl;
                    driving_msg.linear.x = 0.02;
                    driving_msg.angular.z = -0.5;
                }
                if (near(current_yaw_, -90))
                {
                    pstate_ = GO_OUT;
                    last_time = park_time;
                    time_flag = 0;
                }
            }
        }
        break;
        case GO_OUT:
        {
            if (time_flag == 0)
            {
                std::cout << "지금은 직진 중" << std::endl;
                driving_msg.linear.x = 0.09;
                driving_msg.angular.z = 0.04;
            }
            if (time_flag == 0 && park_time - last_time >= 25)
            {
                time_flag = 1;
            }
            if (time_flag == 1)
            {
                park_comp = 1;
                if (park_comp == 0 && error_y == -321 && gos_flag == 0)
                {
                    std::cout << "빠져나가, " << error_y << std::endl;
                    driving_msg.linear.x = 0.09;
                    if (avoid_mem == -1)
                        driving_msg.angular.z = 0.0;
                    else
                        driving_msg.angular.z = 0.04;
                    return;
                }
                else if (gos_flag == 0 && (error_y < 270 && error_y > 0))
                {
                    std::cout << "나가는 pd제어 중 " << error_y << std::endl;
                    PD_control();
                    if (error_y == -321)
                    {
                        driving_msg.linear.x = 0.0;
                        driving_msg.angular.z = 0.3;
                    }
                }
                else if (gos_flag == 0 && error_y > 290)
                {
                    driving_msg.linear.x = 0.04;
                    driving_msg.angular.z = 0.1;
                }
                else if (gos_flag == 0 && error_y > 290 && yellow_count_top == 0)
                {
                    driving_msg.linear.x = 0.0;
                    driving_msg.angular.z = 0.3;
                }
            }
        }
        break;
        default:
            break;
        }
    }
}

void DrivingYY::Parking_tune()
{
    if (mission_flag_ == 5)
    {
        switch (pstate_)
        {
        case PARK_PD:
        {
            if (error_y > 250)
            {
                error_y = -321;
            }
            if ((error_y == -321 && error_w == -321) || (error_y == -321 && z <= 0.25))
            {
                driving_msg.linear.x = 0.09;
                driving_msg.angular.z = 0.34;
            }
            if(yellow_count_top>8000&&yellow_count_low>8000){
                pstate_=PARK_START;
            }
        }
        break;

        case PARK_START:
        {
            if (error_y != -321 &&
                (std::abs(error_y - 0) > 7))
            {
                PD_control();
                std::cout << "지금은 PD제어 중333" << white_count_low << std::endl;
            }
            else
            {
                std::cout << "계속 직진" << white_count_low << std::endl;
                driving_msg.linear.x = 0.07;
                driving_msg.angular.z = 0.0;
            }
            if (near(current_yaw_, 100))
            {
                std::cout << "계속 직진" << white_count_low << std::endl;
                driving_msg.linear.x = 0.07;
                driving_msg.angular.z = 0.0;
            }
            // if(std::abs(error_y-0)<4)yaw_count++;
            // if(yaw_count>4)local_yaw=current_yaw_;
            if (white_count_top > 20000 && (is_left_danger_ > 300))
            {
                pstate_ = AVOID_RIGHT;
                std::cout << "오른쪽으로피하기 시작" << std::endl;
            }
            else if (white_count_top > 20000 && (is_right_danger_ > 300))
            {
                pstate_ = AVOID_LEFT;
                std::cout << "왼쪽으로피하기 시작" << std::endl;
            }
            if (white_count_top > 20000 && is_left_danger_ < 300 && is_right_danger_ < 300)
            {
                pstate_ = PSD_AGAIN;
            }
        }
        break;
        case PSD_AGAIN:
        {
            if (only_y == 1 && white_count_top > 0)
            {
                driving_msg.linear.x = 0.07;
                driving_msg.angular.z = 0.0;
            }
            else
                only_y = 1;

            if (only_y == 1)
            {
                if (white_count_top < 20000 || is_right_danger_ < 250 || is_left_danger_ < 250)
                {
                    driving_msg.linear.x = -0.07;
                    driving_msg.angular.z = 0.0;
                }
                else
                    only_y = 2;
            }
            if (only_y == 2)
            {
                if (is_right_danger_ > 400)
                {
                    pstate_ = AVOID_LEFT;
                    std::cout << "왼쪽으로피하기 시작" << std::endl;
                }
                else if (is_left_danger_ > 400)
                {
                    pstate_ = AVOID_RIGHT;
                    std::cout << "오른쪽으로피하기 시작" << std::endl;
                }
            }
        }
        break;
        case AVOID_RIGHT:
        {
            avoid_mem = -1;
            driving_msg.linear.x = -0.01;
            driving_msg.angular.z = 0.5;
            right_turn = 1;
            std::cout << "오른쪽으로피해" << std::endl;
            if (near(current_yaw_, 20))
                pstate_ = GO_FRONT;
        }
        break;
        case AVOID_LEFT:
        {
            avoid_mem = 1;
            driving_msg.linear.x = -0.01;
            driving_msg.angular.z = -0.5;
            left_turn = 1;
            std::cout << "왼쪽으로피해" << std::endl;
            if (near(current_yaw_, -185))
                pstate_ = GO_FRONT;
        }
        break;
        case GO_FRONT:
        {
            if (avoid_mem == -1)
            {
                if (time_flag == 0)
                {
                    last_time = park_time;
                    time_flag = 1;
                }
                std::cout << "pd안함 뒤로가기 경우1, " << is_front_danger_ << std::endl;
                driving_msg.linear.x = -0.07;
                driving_msg.angular.z = 0.0;
                if (park_time - last_time >= 35)
                {
                    last_time = park_time;
                    time_flag = 0;
                    pstate_ = GO_BACK;
                    for_count = 0;

                    pstate_ = GO_BACK;
                    for_count = 0;
                    return;
                }
            }
            else if (avoid_mem == 1) // 둘다 뒤로갔다 앞으로가기
            {

                if (time_flag == 0)
                {
                    last_time = park_time;
                    time_flag = 1;
                }
                std::cout << "pd안함 뒤로가기 경우1, " << is_front_danger_ << std::endl;
                driving_msg.linear.x = -0.07;
                driving_msg.angular.z = 0.0;
                if (park_time - last_time >= 35)
                {
                    last_time = park_time;
                    time_flag = 0;
                    pstate_ = GO_BACK;
                    for_count = 0;

                    pstate_ = GO_BACK;
                    for_count = 0;
                    return;
                }
            }
        }
        break;
        case GO_BACK: // 여기서 문제생기는듯
        {
            if (avoid_mem == -1)
            {
                if (time_flag == 0)
                {
                    last_time = park_time;
                    time_flag = 1;
                }
                if (turn_flag == 0 && park_time - last_time < 35)
                {
                    std::cout << "pd안함 오른쪽앞으로가기, " << park_time - last_time << std::endl;
                    driving_msg.linear.x = 0.07;
                    driving_msg.angular.z = 0.0;
                    return;
                }
                else if (turn_flag == 0 && park_time - last_time >= 35)
                {
                    turn_flag = 1;
                }
                if (turn_flag == 1)
                {
                    std::cout << "pd안함 오른쪽 돌고있다, " << is_front_danger_ << std::endl;
                    driving_msg.linear.x = 0.02;
                    driving_msg.angular.z = 0.5;
                }
                if (near(current_yaw_, -70))
                {
                    pstate_ = GO_OUT;
                    last_time = park_time;
                    time_flag = 0;
                }
            }
            else if (avoid_mem == 1)
            {

                if (time_flag == 0)
                {
                    last_time = park_time;
                    time_flag = 1;
                }
                if (turn_flag == 0 && park_time - last_time < 35)
                {
                    std::cout << "pd안함 왼쪽앞으로가기, " << park_time - last_time << std::endl;
                    driving_msg.linear.x = 0.07;
                    driving_msg.angular.z = 0.0;
                    return;
                }
                else if (turn_flag == 0 && park_time - last_time >= 35)
                {
                    turn_flag = 1;
                }
                if (turn_flag == 1)
                {
                    std::cout << "pd안함 왼쪽 돌고있다, " << is_front_danger_ << std::endl;
                    driving_msg.linear.x = 0.02;
                    driving_msg.angular.z = -0.5;
                }
                if (near(current_yaw_, -90))
                {
                    pstate_ = GO_OUT2;
                    last_time = park_time;
                    time_flag = 0;
                }
            }
        }
        break;
        case GO_OUT:
        {
            // if (time_flag == 0 && park_time - last_time >= 25)
            // {
            //     time_flag = 1;
            // }
            // if (time_flag == 1)
            // {
            park_comp = 1;
            if (gos_flag == 0 && (error_y < 270 && error_y > 0))
            {
                park_comp = 2;
                std::cout << "나가는 pd제어 중 " << error_y << std::endl;
                Fast_PD();
                if (error_y == -321)
                {
                    driving_msg.linear.x = 0.0;
                    driving_msg.angular.z = 0.3;
                }
            }
            if (gos_flag == 0 && (error_y > 270 ||error_y==-321)&&park_comp != 2)
            {
                std::cout << "지금은 직진 중" << std::endl;
                driving_msg.linear.x = 0.09;
                driving_msg.angular.z = 0.04;
            }
            // else if (gos_flag == 0 && error_y > 290)
            // {
            //     driving_msg.linear.x = 0.04;
            //     driving_msg.angular.z = 0.1;
            // }
            else if (gos_flag == 0 && error_y > 290 && yellow_count_top == 0)
            {
                driving_msg.linear.x = 0.0;
                driving_msg.angular.z = 0.3;
            }
            // }
        }
        break;
        case GO_OUT2:
        {
            if(yellow_count_low<4000){
                driving_msg.linear.x = 0.2;
                driving_msg.angular.z = 0.0;
            }
            else{
                 park_comp = 1;
            }
            Fast_PD();
            if(park_comp==1){
                if (error_y > 270)
                {
                    error_y = -321;
                }
                if ((error_y == -321 && error_w == -321) || (error_y == -321 && z <= 0.25))
                {
                    driving_msg.linear.x = 0.2;
                    driving_msg.angular.z = 0.65;
                }
            }
            
        }
        break;
        default:
            break;
        }
    }
}

void DrivingYY::Level_crossing()
{
    if (mission_flag_ == 5)
    {
        if (traffic_light_status_ == 4)
        {
            passed_Level = 1;
            driving_msg.linear.x = 0;
            driving_msg.angular.z = 0;
        }
    }
}
void DrivingYY::total_driving()
{
    // 여기가 전체 제어..일단 만들어둠
}
void DrivingYY::drive_callback()
{
    RCLCPP_INFO(this->get_logger(), "degree_now: %f", current_yaw_);
    if (l_start_flag == 1)
    {
        if (mission_flag_ < 4 || passed_Level == 1)
        {
            Fast_PD();
        }
        else
        {
            PD_control();
        }
        // std::cout << "pd안함 뒤로가기1, " << "white_low:" << white_count_low << "white_top:" << white_count_top << "yellow_low:" << yellow_count_low << "yellow_top:" << yellow_count_top << std::endl;
        std::cout << "yaw:" << current_yaw_ << std::endl;
        // Traffic_light();
        //  if(mission_flag_==1||mission_flag_==2||mission_flag_==3)
        Itersection();
        // if(mission_flag_==4)
        Construction();
        // else if(mission_flag_==5)
        Parking_tune();
        // else
        Level_crossing();
        //     Level_crossing();
        // }
    }
    else
    {
        driving_msg.linear.x = 0;
        driving_msg.angular.z = 0;
    }
    driving_msg.linear.y = 0;
    driving_msg.linear.z = 0;
    driving_msg.angular.x = 0;
    driving_msg.angular.y = 0;
    if (start_flag == 0)
        publisher_drive->publish(driving_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrivingYY>());
    rclcpp::shutdown();
    return 0;
}
