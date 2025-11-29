cmake_minimum_required(VERSION 3.8)
project(master_jo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlebot3_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autorace_interfaces REQUIRED)

include_directories(include ${ament_INCLUDE_DIRS} ${autorace_interfaces_INCLUDE_DIRS})

add_executable(master_jo src/master_jo.cpp)

ament_target_dependencies(master_jo 
  rclcpp 
  std_msgs 
  turtlebot3_msgs 
  autorace_interfaces
)

add_executable(psd_jo src/psd_jo.cpp)

ament_target_dependencies(psd_jo 
  rclcpp 
  std_msgs 
  turtlebot3_msgs
)

add_executable(imu_jo src/imu_jo.cpp)

ament_target_dependencies(imu_jo
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  master_jo
  psd_jo
  imu_jo
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()