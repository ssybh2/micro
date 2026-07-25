cmake_minimum_required(VERSION 3.8)
project(micro_lqr_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(micro_lqr_node src/micro_lqr_node.cpp)
target_compile_features(micro_lqr_node PUBLIC cxx_std_17)
ament_target_dependencies(
  micro_lqr_node
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
  custom_msgs
)
target_link_libraries(micro_lqr_node Eigen3::Eigen)

add_executable(micro_lqr_pole_monitor src/micro_lqr_pole_monitor.cpp)
target_compile_features(micro_lqr_pole_monitor PUBLIC cxx_std_17)
ament_target_dependencies(
  micro_lqr_pole_monitor
  rclcpp
  rcl_interfaces
  std_msgs
)
target_link_libraries(micro_lqr_pole_monitor Eigen3::Eigen)

install(TARGETS
  micro_lqr_node
  micro_lqr_pole_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  tools
  reference
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_topics.sh
  scripts/pole_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
