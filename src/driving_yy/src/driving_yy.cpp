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
    vision_valid_=msg->vision_lost_flag;
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

void DrivingYY::vision_traffic_callback(const autorace_interfaces::msg::VisionHyun::SharedPtr msg)
{
    traffic_light_status_ = msg->traffic_light;
    brown_count = msg->brown_count;
    yellow_count = msg->yellowline_count;
    white_count = msg->whiteline_count;

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

double degreecal(double degree){
    while(degree>=190||degree<-190){
        if(degree>190){degree-=380;}
        else if(degree<=-190){degree+=380;}
    }
    return degree;
}

void DrivingYY::PD_control()
{

    // if (!vision_valid_) { //비전 끊겼을 때 대처
    //     error = 0;
    //     last_error = 0;
    //     driving_msg.angular.z = 0.0;
    //     driving_msg.linear.x = 0.01;
    //     return;
    // } 
    //else{
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
    //}

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "[DEBUG] flags: start_flag=%d, l_start_flag=%d, error=%.2f",
                         start_flag, l_start_flag, error);

    z = kp * error + kd * (error - last_error);

    // 디버깅 출력 추가
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "error=%.2f, last_error=%.2f, kp=%.2f, kd=%.2f, z=%.4f",
                         error, last_error, kp, kd, z);

    last_error = error;

    // x 계산 수정
    double ratio = std::max(1.0 - std::abs(error) / 360.0, 0.0);
    double x_raw = std::pow(max_x * ratio, 2.2);
    x = std::min(x_raw, 5.0);

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //     "ratio=%.2f, x_raw=%.4f, x=%.4f", ratio, x_raw, x);

    if (z < 0)
        z = -std::max(z, -0.46);
    else
        z = -std::min(z, 0.46);
    //}

    driving_msg.linear.x = x;
    driving_msg.angular.z = z;

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //     "Publishing: x=%.4f, z=%.4f", driving_msg.linear.x, driving_msg.angular.z);
}

void DrivingYY::Traffic_light()
{
    if (traffic_light_status_ != 0)
    {
        switch (traffic_light_status_)
        {
        case 1:
        {
            driving_msg.linear.x = 0;
            driving_msg.angular.z = 0;
        }
        break;

        default: // 노란색 감지를 위해 비워 놓는 것이다...
            break;
        }
    }
}
void DrivingYY::Itersection()
{

    if (mission_flag_ == 2)
    { // 좌
        if ((error_y == -321 && error_w == -321) || (error_y == -321 && z <= 0.08))
        {
            driving_msg.linear.x = 0.09;
            driving_msg.angular.z = 0.36;
        }
    }
    else if (mission_flag_ == 3)
    { // 우
        if ((error_y == -321 && error_w == -321) || (error_w == -321 && z >= -0.08))
        {
            driving_msg.linear.x = 0.09;
            driving_msg.angular.z = -0.36;
        }
    }
}
void DrivingYY::Construction()
{
    RCLCPP_INFO(this->get_logger(), "degree_now: %f", current_yaw_);
    RCLCPP_INFO(this->get_logger(), "degree_goal: %f", local_yaw);
    if(mission_flag_==4){
        if(timer<count){
            if((current_yaw_-local_yaw)<5&&(current_yaw_-local_yaw)>-5){timer++;}
            else{
                timer=0;
                local_yaw=current_yaw_;
            }
        }
        if(timer==count){
            local_diff=degreecal(local_yaw-current_yaw_-95);
        }
        
        RCLCPP_INFO(this->get_logger(), "degree_diff: %f", local_diff);

        if(brown_count>42000&&state==0){state=1;}
        if(state==0){
            if(error_y==-321){Construction_mem=1;}
            else if(error_w==-321){Construction_mem=0;}
            RCLCPP_INFO(this->get_logger(), "s0"); 
            if(brown_count>42000){state=1;}
        }
        else if(state==1){   
            RCLCPP_INFO(this->get_logger(), "s1, Construction_mem:%d",Construction_mem); 
            if(local_diff<2&&local_diff>-2){
                if(brown_count>2000){state=2;}
                else{state=0;}
            }
            else{
                if(local_diff>0){
                    driving_msg.linear.x =0.0;
                    driving_msg.angular.z =  -0.25;
                }
                else if(local_diff<0){
                    driving_msg.linear.x =0.0;
                    driving_msg.angular.z =  0.25;
                }
            }
        }
        else if(state==2){
            RCLCPP_INFO(this->get_logger(), "s2, Construction_mem:%d",Construction_mem); 
            if(Construction_mem==1){
                if(local_diff<92&&local_diff>88){
                    driving_msg.linear.x =0.05;
                    driving_msg.angular.z =  0.0;
                    RCLCPP_INFO(this->get_logger(), "s3, yellow: %d",yellow_count); 
                    if(yellow_count>=6000){state=3;}
                }
                else if(local_diff>90){
                    driving_msg.linear.x =0.0;
                    driving_msg.angular.z =  -0.25;
                }
                else if(local_diff<90){
                    driving_msg.linear.x =0.0;
                    driving_msg.angular.z =  0.25;
                }
            }
            else if(Construction_mem==0){
                if(local_diff>-92&&local_diff<-88){
                    driving_msg.linear.x =0.05;
                    driving_msg.angular.z =  0.0;
                    RCLCPP_INFO(this->get_logger(), "s4, white: %d",white_count); 
                    if(white_count>=6000){state=4;}
                }
                else if(local_diff>-90){
                    driving_msg.linear.x =0.00;
                    driving_msg.angular.z =  -0.25;
                }
                else if(local_diff<-90){
                    driving_msg.linear.x =0.00;
                    driving_msg.angular.z =  0.25;
                }
            }

        }
        else if(state==3){
            if(local_diff<47&&local_diff>43){
                state=0;
            }
            else if(local_diff>45){
                driving_msg.linear.x =0.0;
                driving_msg.angular.z =  -0.25;
            }
            else if(local_diff<45){
                driving_msg.linear.x =0.0;
                driving_msg.angular.z =  0.25;
            }
        }
        else if(state==4){
            if(local_diff>-47&&local_diff<-43){
                state=0;
            }
                
            else if(local_diff>-45){
                driving_msg.linear.x =0.00;
                driving_msg.angular.z =  -0.25;
            }
            else if(local_diff<-45){
                driving_msg.linear.x =0.00;
                driving_msg.angular.z =  0.25;
            }
        }

    }

}

enum ParkingState
{
    PARK_PD,
    AVOID_RIGHT,
    AVOID_LEFT,
    GO_FRONT,
    GO_BACK
};

ParkingState pstate_;

bool near(double a, double b, double eps = 3.0)
{
    return fabs(a - b) < eps;
}

void DrivingYY::Parking()
{
    switch (pstate_)
    {
    case PARK_PD:
    {
        
        PD_control();
        if (error_yw == -321&&error_y<0)
        {
            driving_msg.linear.x = 0.09;
            driving_msg.angular.z = 0.36;
        }
        if (traffic_light_status_==4&&is_left_danger_>500){
            pstate_ = AVOID_RIGHT;
            std::cout<<"오른쪽으로피하기 시작"<<std::endl;
        }
        else if (traffic_light_status_==4&&is_right_danger_>500){
            pstate_ = AVOID_LEFT;
            std::cout<<"왼쪽으로피하기 시작"<<std::endl;
        }
    }
    break;

    case AVOID_RIGHT:
    {
        driving_msg.linear.x = 0.09;
        driving_msg.angular.z = -0.35;
        right_turn = 1;
        std::cout<<"오른쪽으로피해"<<std::endl;
        if (near(current_yaw_, -180) || near(current_yaw_, 180))
            pstate_ = GO_FRONT;
    }
    break;
    case AVOID_LEFT:
    {
        driving_msg.linear.x = 0.09;
        driving_msg.angular.z = 0.35;
        left_turn = 1;
        std::cout<<"왼쪽으로피해"<<std::endl;
        if (near(current_yaw_, 0))
            pstate_ = GO_FRONT;
    }
    break;
    case GO_FRONT:
    {
        driving_msg.linear.x = 0.04;
        driving_msg.angular.z = 0.0;
        std::cout<<"pd안함 앞으로가기"<<std::endl;
        if (traffic_light_status_==3) // 이거 디버깅하면서 확인할 것
            pstate_ = GO_BACK;
    }
    break;
    case GO_BACK:
    {
        std::cout<<"pd안함 뒤로가기"<<std::endl;
        if (left_turn){
            std::cout<<"pd안함 왼쪽뒤로가기"<<std::endl;
            driving_msg.linear.x = -0.09;
            driving_msg.angular.z = 0.35;}
        if (right_turn){
            std::cout<<"pd안함 오른쪽 뒤로가기"<<std::endl;
            driving_msg.linear.x = -0.09;
            driving_msg.angular.z = -0.35;}
        if (near(current_yaw_, 90))
            pstate_ = PARK_PD;
    }
    break;
    }
}
void DrivingYY::Level_crossing()
{
    driving_msg.linear.x = 0;
    driving_msg.angular.z = 0;
    if(traffic_light_status_!=4){
        PD_control();
    }
}
void DrivingYY::total_driving()
{
    // 여기가 전체 제어..일단 만들어둠
}

void DrivingYY::drive_callback()
{
    if (l_start_flag == 1)
    {
        PD_control();
        //Traffic_light();
        Itersection();
        Construction();
        // if (mission_flag_ == 2 || mission_flag_ == 3)
        //     Itersection();
        // else if (mission_flag_ == 4)
        //     Construction();
        // else if (mission_flag_ == 5)
        // Parking();
        // else if(traffic_light_status_==4){
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
