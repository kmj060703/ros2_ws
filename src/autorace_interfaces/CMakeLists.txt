cmake_minimum_required(VERSION 3.8)
project(autorace_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 메시지 파일 선언 (이 부분이 중요!)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MasterJo.msg"
  "msg/Ui2Driving.msg"
  "msg/VisionHyun.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(std_msgs geometry_msgs rosidl_default_generators)
ament_package()