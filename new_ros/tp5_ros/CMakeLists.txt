cmake_minimum_required(VERSION 3.8)
project(tp5_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)


add_executable(my_node src/my_node.cpp)
add_executable(my_node_carla src/my_node_carla.cpp)
add_executable(enemy_node src/enemy_node.cpp)
add_executable(gps_pub_sim src/gps_pub_sim.cpp)
add_executable(udp_to_pub src/udp_to_pub.cpp)


ament_target_dependencies(my_node rclcpp sensor_msgs nav_msgs)
ament_target_dependencies(my_node_carla rclcpp sensor_msgs nav_msgs)
ament_target_dependencies(gps_pub_sim rclcpp sensor_msgs)
ament_target_dependencies(udp_to_pub rclcpp ackermann_msgs)
ament_target_dependencies(enemy_node rclcpp sensor_msgs nav_msgs)

target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  my_node
  my_node_carla
  gps_pub_sim
  udp_to_pub
  enemy_node
  DESTINATION lib/${PROJECT_NAME})

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
