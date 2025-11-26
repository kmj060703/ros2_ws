cmake_minimum_required(VERSION 3.8)
project(master_jo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlebot3_msgs REQUIRED) #의존성 추가
include_directories(include)

add_executable(master_jo src/master_jo.cpp)
add_executable(psd_jo src/psd_jo.cpp)

ament_target_dependencies(master_jo rclcpp std_msgs turtlebot3_msgs)
ament_target_dependencies(psd_jo rclcpp std_msgs turtlebot3_msgs)

install(TARGETS
  master_jo
  psd_jo
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
