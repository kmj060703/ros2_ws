// std::array<int,5> arr; //array with length 5
	
// 	arr=func();

#include "../include/driving_yy/driving_yy.hpp"

DrivingYYNode::DrivingYYNode() : Node("driving_yy")
{
    Imu2Driving = this ->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 
        10,
        std::bind(&DrivingYYNode::topic_callback, this, std::placeholders::_1));
}

// double Driving_yy::turn(double angle, double angular_velocity, double step){

//         if(math.fabs(angle) > 0.01){  
//             if(angle >= math.pi){
//                 twist.angular.z = -angular_velocity}
//             else if(math.pi > angle and angle >= 0){
//                 twist.angular.z = angular_velocity}
//             else if(0 > angle and angle >= -math.pi){
//                 twist.angular.z = -angular_velocity}
//             else if(angle > -math.pi){
//                 twist.angular.z = angular_velocity}
//         }
//         else{
//             step += 1
//         }
//         std::array<double,8> result;
//         result=func();
//         //double ret[2]={twist, step};
//         //return ret;
// }

// double Driving_yy::go_straight(double distance, double linear_velocity, double step){
//         if(distance > 0.01){  // 0.01 is small enough value
//             twist.linear.x = linear_velocity
//         }
//         else{
//             step += 1
//         }
//         std::array<double,8> result;
//         result=func();
//         //return twist, step
// }

void DrivingYYNode::topic_callback(const sensor_msgs::msg::Imu msg)
{
    RCLCPP_INFO(this->get_logger(), "orientation.w: '%f'", msg.orientation.w);
    RCLCPP_INFO(this->get_logger(), "orientation.x: '%f'", msg.orientation.x);
    RCLCPP_INFO(this->get_logger(), "orientation.y: '%f'", msg.orientation.y);
    RCLCPP_INFO(this->get_logger(), "orientation.z: '%f'", msg.orientation.z);
    RCLCPP_INFO(this->get_logger(), "angular_velocity.x: '%f'", msg.angular_velocity.x);
    RCLCPP_INFO(this->get_logger(), "angular_velocity.y: '%f'", msg.angular_velocity.y);
    RCLCPP_INFO(this->get_logger(), "angular_velocity.z: '%f'", msg.angular_velocity.z);
    RCLCPP_INFO(this->get_logger(), "linear_acceleration.x: '%f'", msg.linear_acceleration.x);
    RCLCPP_INFO(this->get_logger(), "linear_acceleration.y: '%f'", msg.linear_acceleration.y);
    RCLCPP_INFO(this->get_logger(), "linear_acceleration.z: '%f'", msg.linear_acceleration.z);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrivingYYNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
