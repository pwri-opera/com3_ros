cmake_minimum_required(VERSION 3.0.2)
project(excavator_com3_ros)

set (CMAKE_CXX_STANDARD 17)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

find_package(canary REQUIRED)

find_package(Boost 1.70 REQUIRED)

find_package(RTTR CONFIG REQUIRED Core)


catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES operation_cmd_handler
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(excavator_com3_ros src/excavator_com3_ros.cpp)

target_link_libraries(excavator_com3_ros PUBLIC 
   ${catkin_LIBRARIES}
  canary::canary
  boost_system
  boost_thread
  boost_timer
  dbcppp
  RTTR::Core
)

# add_executable(joy2cmd src/joy2cmd.cpp)

# target_link_libraries(joy2cmd
#    ${catkin_LIBRARIES})
