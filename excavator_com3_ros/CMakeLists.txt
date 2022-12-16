cmake_minimum_required(VERSION 3.5)
project(excavator_com3_ros)

# Default to C++14
set (CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(com3_msgs REQUIRED)
find_package(canary REQUIRED)
find_package(Boost 1.70 COMPONENTS system thread timer REQUIRED)
find_package(RTTR CONFIG REQUIRED Core)

include_directories(
  include
)

add_executable(lever_cmd_relay src/lever_cmd_relay.cpp)
ament_target_dependencies(lever_cmd_relay
  rclcpp
  rclpy
  com3_msgs
  std_msgs
  geometry_msgs 
)
target_link_libraries(lever_cmd_relay
  canary::canary
  boost_system
  boost_thread
  boost_timer
  dbcppp
  RTTR::Core
)

add_executable(machine_setting_cmd_relay src/machine_setting_cmd_relay.cpp)
ament_target_dependencies(machine_setting_cmd_relay
  rclcpp
  rclpy
  com3_msgs
  std_msgs
  geometry_msgs 
)
target_link_libraries(machine_setting_cmd_relay
  canary::canary
  boost_system
  boost_thread
  boost_timer
  dbcppp
  RTTR::Core
)

install(TARGETS 
  lever_cmd_relay
  machine_setting_cmd_relay
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
