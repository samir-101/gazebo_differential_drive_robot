cmake_minimum_required(VERSION 3.8)
project(rosbot-wheel-comm)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rosbot-wheel-comm src/rosbot-wheel-comm.cpp)

ament_target_dependencies(rosbot-wheel-comm
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  rosbot-wheel-comm
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
