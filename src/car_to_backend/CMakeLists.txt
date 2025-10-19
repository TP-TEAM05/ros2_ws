cmake_minimum_required(VERSION 3.5)
project(car_to_backend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)

add_executable(serial_pub src/publisher_from_serial.cpp)
add_executable(udp_sub src/subscriber_to_udp.cpp)
add_executable(udp_pub src/controls_from_udp.cpp)
add_executable(car_controls src/car_controls.cpp)
add_executable(gps_pub src/gps_publisher.cpp)
ament_target_dependencies(serial_pub rclcpp std_msgs)
ament_target_dependencies(udp_sub rclcpp std_msgs)
ament_target_dependencies(udp_pub rclcpp std_msgs)
ament_target_dependencies(car_controls rclcpp std_msgs)
ament_target_dependencies(gps_pub rclcpp std_msgs CURL)

install(TARGETS
  serial_pub
  udp_sub
  udp_pub
  car_controls
  gps_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
