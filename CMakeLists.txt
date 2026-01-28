cmake_minimum_required(VERSION 3.8)
project(encoder_joint_state_publisher)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(encoder_to_joint_state_node src/rosbot-wheel-comm.cpp)

ament_target_dependencies(
  encoder_to_joint_state_node
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS
  encoder_to_joint_state_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
