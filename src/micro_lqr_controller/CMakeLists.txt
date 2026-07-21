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

install(TARGETS
  micro_lqr_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_topics.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
